#include "OID485MotorBus.h"
#include <chrono>
#include <thread>
#include <cstring> // memset

#include <iostream>
#define delaytime 2000
#define delaytime2 2000
OID485MotorBus::OID485MotorBus(
    const std::string &device,
    int baud,
    char parity,
    int data_bit,
    int stop_bit)
    : device_(device), baud_(baud), parity_(parity), data_bit_(data_bit), stop_bit_(stop_bit)
{
}

OID485MotorBus::~OID485MotorBus()
{
    deinit();
}

bool OID485MotorBus::init()
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
    modbus_set_response_timeout(ctx_, 0, 5000);
    modbus_set_byte_timeout(ctx_, 0, 5000);

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

void OID485MotorBus::deinit()
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

bool OID485MotorBus::isReady() const
{
    return ready_.load() && connected_ && ctx_ != nullptr;
}

bool OID485MotorBus::reconnect()
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

void OID485MotorBus::closeConnection()
{
    if (ctx_)
    {
        modbus_close(ctx_);
        modbus_free(ctx_);
        ctx_ = nullptr;
    }
    connected_ = false;
}

void OID485MotorBus::reportModbusError(const MotorDevice &motor, const std::string &operation)
{
    int err = errno;
    std::string msg = operation + " failed: " + modbus_strerror(err);
    reportError(motor.id, 0x200 + err, msg);
}

// ========================
// 核心：写入心跳指令
// ========================
bool OID485MotorBus::writeMotorBeat(const MotorDevice &motor)
{
    if (!isReady())
        return false;
    int slaveAddr = static_cast<int>(motor.addressOrNodeId);

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

    uint16_t heartcount = ++heartcountMap[slaveAddr];
    int32_t sign = modbus_write_register(ctx_, 6000, heartcount);
    lastCallTimeUs = getBusTimestamp();
    if (sign == -1)
    {
        reportModbusError(motor, "Write control registers");
        return false;
    }
    // std::this_thread::sleep_for(std::chrono::microseconds(delaytime));
    return true;
}

// ========================
// 核心：写入控制指令
// ========================
bool OID485MotorBus::writeMotorControl(const MotorDevice &motor)
{
    if (!isReady())
        return false;
    auto status = motor.getStatusSnapshot();
    auto cmd = motor.getControlCommand();
    int slaveAddr = static_cast<int>(motor.addressOrNodeId);
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

    uint16_t registers[5] = {0};

    switch (cmd.controlMode)
    {
    case MotorDevice::Mode::POSITION_CONTROL:
    {
        if (workModMap[slaveAddr] != MotorDevice::Mode::POSITION_CONTROL)
        {
            if (modbus_write_register(ctx_, 6001, 3) == -1)
            {
                reportModbusError(motor, "Write mode registers");
                return false;
            }
            workModMap[slaveAddr] = MotorDevice::Mode::POSITION_CONTROL;
        }
        uint32_t position_raw = static_cast<uint32_t>(
            static_cast<int32_t>(cmd.targetPosition / motor.config.positionUnit));

        registers[0] = static_cast<uint16_t>(position_raw >> 16);
        registers[1] = static_cast<uint16_t>(position_raw);
        int32_t sign = modbus_write_registers(ctx_, 6006, 2, registers);
        lastCallTimeUs = getBusTimestamp();
        if (sign == -1)
        {
            reportModbusError(motor, "Write control registers");
            return false;
        }
    }
    break;
    case MotorDevice::Mode::VELOCITY_CONTROL:
    {
        if (workModMap[slaveAddr] != MotorDevice::Mode::VELOCITY_CONTROL)
        {
            if (modbus_write_register(ctx_, 6001, 1) == -1)
            {
                reportModbusError(motor, "Write mode registers");
                return false;
            }
            workModMap[slaveAddr] = MotorDevice::Mode::VELOCITY_CONTROL;
        }
        uint32_t velocity_raw = static_cast<uint32_t>(
            static_cast<int32_t>(cmd.targetVelocity / motor.config.velocityUnit));

        registers[0] = static_cast<uint16_t>(velocity_raw >> 16);
        registers[1] = static_cast<uint16_t>(velocity_raw);
        int32_t sign = modbus_write_registers(ctx_, 6003, 2, registers);
        lastCallTimeUs = getBusTimestamp();
        if (sign == -1)
        {
            reportModbusError(motor, "Write control registers");
            return false;
        }
    }
    break;
    case MotorDevice::Mode::TORQUE_CONTROL:
    {
        if (workModMap[slaveAddr] != MotorDevice::Mode::TORQUE_CONTROL)
        {
            if (modbus_write_register(ctx_, 6001, 0) == -1)
            {
                reportModbusError(motor, "Write mode registers");
                return false;
            }
            workModMap[slaveAddr] = MotorDevice::Mode::TORQUE_CONTROL;
        }
        uint16_t torque_raw = static_cast<uint16_t>(
            static_cast<int16_t>(cmd.targetTorque / motor.config.torqueUnit));
        int32_t sign = modbus_write_register(ctx_, 6002, torque_raw);
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
bool OID485MotorBus::readMotorStatus(MotorDevice &motor)
{
    if (!isReady())
        return false;

    int slaveAddr = static_cast<int>(motor.addressOrNodeId);
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

    uint16_t registers[12] = {0};
    int32_t sign = modbus_read_input_registers(ctx_, 5000, 12, registers);
    lastCallTimeUs = getBusTimestamp();
    if (sign == -1)
    {
        reportModbusError(motor, "Read status registers");
        return false;
    }

    // 解析到 StatusData
    MotorDevice::StatusData status;
    status.faultCode = registers[0];
    status.actualVelocity = static_cast<int32_t>((registers[1] << 16) | registers[2]) *
                            motor.config.velocityUnit;
    status.actualPosition = static_cast<int32_t>((registers[10] << 16) | registers[11]) *
                            motor.config.positionUnit;
    status.actualTorque = static_cast<int16_t>(registers[6]) * motor.config.torqueUnit;
    status.busVoltage = static_cast<int16_t>(registers[5]) * 0.01;
    status.temperature = static_cast<int16_t>(registers[8]) * 0.01;

    // 时间戳
    status.lastUpdateTimeUs = getBusTimestamp();

    // 更新到电机
    motor.updateStatus(status);

    // std::cout << "update: " << motor.timing.lastCtrlTimeMs << std::endl;
    // std::this_thread::sleep_for(std::chrono::microseconds(delaytime));
    return true;
}

bool OID485MotorBus::syncTrigger()
{
    // Modbus RTU 无硬件同步，可用广播写“同步触发寄存器”
    // 或简单返回 true，依赖软件时间戳同步
    return true;
}

uint64_t OID485MotorBus::getBusTimestamp() const
{
    auto now = std::chrono::steady_clock::now();
    return std::chrono::duration_cast<std::chrono::microseconds>(
               now.time_since_epoch())
        .count();
}