#include "ZLAC8015MotorBus.h"
#include <chrono>
#include <thread>
#include <cstring> // memset

#include <iostream>
#define delaytime 2000
#define delaytime2 2000
ZLAC8015MotorBus::ZLAC8015MotorBus(
    const std::string &device,
    int baud,
    char parity,
    int data_bit,
    int stop_bit)
    : device_(device), baud_(baud), parity_(parity), data_bit_(data_bit), stop_bit_(stop_bit)
{
}

ZLAC8015MotorBus::~ZLAC8015MotorBus()
{
    deinit();
}

bool ZLAC8015MotorBus::init()
{
    if (ctx_)
    {
        modbus_close(ctx_);
        modbus_free(ctx_);
    }

    // ctx_ = (modbus_t *)1;
    ctx_ = modbus_new_rtu(device_.c_str(), baud_, parity_, data_bit_, stop_bit_);
    if (!ctx_)
    {
        reportError(0, 0x101, "Failed to create modbus RTU context");
        return false;
    }

    // 可选：设置响应超时（默认可能太长）
    modbus_set_response_timeout(ctx_, 0, 10000);
    modbus_set_byte_timeout(ctx_, 0, 10000);

    if (modbus_connect(ctx_) == -1)
    {
        reportError(0, 0x102, "Modbus RTU connect failed: " + std::string(modbus_strerror(errno)));
        modbus_free(ctx_);
        ctx_ = nullptr;
        return false;
    }

    connected_ = true;
    ready_ = true;
    return true;
}

void ZLAC8015MotorBus::deinit()
{
    ready_ = false;
    if (ctx_)
    {
        modbus_close(ctx_);
        modbus_free(ctx_);
        ctx_ = nullptr;
    }
    connected_ = false;
}

bool ZLAC8015MotorBus::isReady() const
{
    return ready_.load() && connected_ && ctx_ != nullptr;
}

bool ZLAC8015MotorBus::reconnect()
{
    if (ctx_)
    {
        modbus_close(ctx_);
        modbus_free(ctx_);
    }

    ctx_ = modbus_new_rtu(device_.c_str(), baud_, parity_, data_bit_, stop_bit_);
    if (!ctx_)
        return false;

    modbus_set_response_timeout(ctx_, 0, 4000);
    modbus_set_byte_timeout(ctx_, 0, 4000);

    if (modbus_connect(ctx_) == -1)
    {
        modbus_free(ctx_);
        ctx_ = nullptr;
        return false;
    }

    connected_ = true;
    ready_ = true;
    return true;
}

void ZLAC8015MotorBus::closeConnection()
{
    if (ctx_)
    {
        modbus_close(ctx_);
        modbus_free(ctx_);
        ctx_ = nullptr;
    }
    connected_ = false;
}

void ZLAC8015MotorBus::reportModbusError(const MotorDevice &motor, const std::string &operation)
{
    int err = errno;
    std::string msg = operation + " failed: " + modbus_strerror(err);
    reportError(motor.id, 0x200 + err, msg);
}

// ========================
// 核心：写入心跳指令
// ========================
bool ZLAC8015MotorBus::writeMotorBeat(const MotorDevice &motor)
{
    return true;
}

// ========================
// 核心：写入控制指令
// ========================
bool ZLAC8015MotorBus::writeMotorControl(const MotorDevice &motor)
{
    if (!isReady())
        return false;
    auto status = motor.getStatusSnapshot();
    auto cmd = motor.getControlCommand();

    int32_t slaveAddr = (static_cast<int>(motor.addressOrNodeId) >> 8);
    int32_t leftORright = static_cast<int>(motor.addressOrNodeId) & 0x000000FF;

    if (modbus_get_slave(ctx_) != slaveAddr)
    {
        if (getBusTimestamp() - lastCallTimeUs <= 2000)
            std::this_thread::sleep_for(std::chrono::microseconds(delaytime2));
        if (modbus_set_slave(ctx_, slaveAddr) == -1)
        {
            reportError(motor.id, 0x201, "Invalid slave address");
            return false;
        }
    }

    uint16_t registers[2] = {0};

    switch (cmd.controlMode)
    {
    case MotorDevice::Mode::VELOCITY_CONTROL:
    {
        if (workModMap[motor.id] != MotorDevice::Mode::VELOCITY_CONTROL)
        {
            if (modbus_write_register(ctx_, 0x200D, 3) == -1)
            {
                reportModbusError(motor, "Write mode registers");
                std::cout<<"!!"<<std::endl;
                return false;
            }
            if (modbus_write_register(ctx_, 0x200E, 0x08) == -1)
            {
                reportModbusError(motor, "Write mode registers");
                return false;
            }
            workModMap[motor.id] = MotorDevice::Mode::VELOCITY_CONTROL;
        }
    

        uint16_t velocity_raw = static_cast<uint16_t>(
            static_cast<int32_t>(cmd.targetVelocity / motor.config.velocityUnit));

        int32_t sign;
        if (leftORright == 0)
            sign = modbus_write_register(ctx_, 0x2088, velocity_raw);
        else
            sign = modbus_write_register(ctx_, 0x2089, velocity_raw);

        lastCallTimeUs = getBusTimestamp();
        if (sign == -1)
        {
            reportModbusError(motor, "Write control registers");
            return false;
        }
    }
    break;
    }
    // std::cout << "send: " << motor.timing.lastCtrlTimeMs << std::endl;

    return true;
}

// ========================
// 核心：读取状态
// ========================
bool ZLAC8015MotorBus::readMotorStatus(MotorDevice &motor)
{
    if (!isReady())
        return false;

    int slaveAddr = (static_cast<int>(motor.addressOrNodeId) >> 8);
    int leftORright = static_cast<int>(motor.addressOrNodeId) & 0x000000FF;
    if (modbus_get_slave(ctx_) != slaveAddr)
    {
        if (getBusTimestamp() - lastCallTimeUs <= 2000)
            std::this_thread::sleep_for(std::chrono::microseconds(delaytime2));
        if (modbus_set_slave(ctx_, slaveAddr) == -1)
        {
            reportError(motor.id, 0x201, "Invalid slave address");
            return false;
        }
    }

    uint16_t registers[2] = {0};
    int32_t sign;
    if (leftORright == 0)
        sign = modbus_read_registers(ctx_, 0x20AB, 1, registers);
    else
        sign = modbus_read_registers(ctx_, 0x20AC, 1, registers);

    lastCallTimeUs = getBusTimestamp();
    if (sign == -1)
    {
        reportModbusError(motor, "Read status registers");
        return false;
    }

    // 解析到 StatusData
    MotorDevice::StatusData status;
    status.actualVelocity = static_cast<int16_t>(registers[0]) *
                            motor.config.velocityUnit / 10.0;

    // 时间戳
    status.lastUpdateTimeUs = getBusTimestamp();

    // 更新到电机
    motor.updateStatus(status);

    // std::cout << "update: " << motor.timing.lastCtrlTimeMs << std::endl;
    // std::this_thread::sleep_for(std::chrono::microseconds(delaytime));
    return true;
}

bool ZLAC8015MotorBus::syncTrigger()
{
    // Modbus RTU 无硬件同步，可用广播写“同步触发寄存器”
    // 或简单返回 true，依赖软件时间戳同步
    return true;
}

uint64_t ZLAC8015MotorBus::getBusTimestamp() const
{
    auto now = std::chrono::steady_clock::now();
    return std::chrono::duration_cast<std::chrono::microseconds>(
               now.time_since_epoch())
        .count();
}