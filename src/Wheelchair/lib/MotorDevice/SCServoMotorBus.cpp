#include <chrono>
#include <thread>
#include <cstring> // memset
#include "SCServoMotorBus.h"

#include <iostream>
#define delaytime 2000
#define delaytime2 2000
SCServoMotorBus::SCServoMotorBus(
    const std::string &device,
    int baud,
    char parity,
    int data_bit,
    int stop_bit)
    : device_(device), baud_(baud), parity_(parity), data_bit_(data_bit), stop_bit_(stop_bit)
{
}

SCServoMotorBus::~SCServoMotorBus()
{
    deinit();
}

bool SCServoMotorBus::init()
{
    // if (ctx_)
    // {
    //     modbus_close(ctx_);
    //     modbus_free(ctx_);
    // }

    // // ctx_ = (modbus_t *)1;
    // ctx_ = modbus_new_rtu(device_.c_str(), baud_, parity_, data_bit_, stop_bit_);
    // if (!ctx_)
    // {
    //     reportError(0, 0x101, "Failed to create modbus RTU context");
    //     return false;
    // }

    // // 可选：设置响应超时（默认可能太长）
    // modbus_set_response_timeout(ctx_, 0, 10000);
    // modbus_set_byte_timeout(ctx_, 0, 10000);

    // if (modbus_connect(ctx_) == -1)
    // {
    //     reportError(0, 0x102, "Modbus RTU connect failed: " + std::string(modbus_strerror(errno)));
    //     modbus_free(ctx_);
    //     ctx_ = nullptr;
    //     return false;
    // }

    if (!sm_st.begin(baud_, device_.c_str()))
    {
        std::cout << "Failed to init scscl motor!" << std::endl;
        return 0;
    }

    connected_ = true;
    ready_ = true;
    return true;
}

void SCServoMotorBus::deinit()
{
    ready_ = false;
    sm_st.end();
    connected_ = false;
}

bool SCServoMotorBus::isReady() const
{
    return ready_.load() && connected_;
}

bool SCServoMotorBus::reconnect()
{
    return true;
}

void SCServoMotorBus::closeConnection()
{
    // if (ctx_)
    // {
    //     modbus_close(ctx_);
    //     modbus_free(ctx_);
    //     ctx_ = nullptr;
    // }
    connected_ = false;
}

void SCServoMotorBus::reportModbusError(const MotorDevice &motor, const std::string &operation)
{
    int err = errno;
    std::string msg = operation + " failed: ";
    reportError(motor.id, 0x200 + err, msg);
}

// ========================
// 核心：写入心跳指令
// ========================
bool SCServoMotorBus::writeMotorBeat(const MotorDevice &motor)
{
    return true;
}

// ========================
// 核心：写入控制指令
// ========================
bool SCServoMotorBus::writeMotorControl(const MotorDevice &motor)
{
    if (!isReady())
        return false;
    auto status = motor.getStatusSnapshot();
    auto cmd = motor.getControlCommand();

    int32_t slaveAddr = static_cast<int>(motor.addressOrNodeId);

    uint16_t registers[2] = {0};

    switch (cmd.controlMode)
    {
    case MotorDevice::Mode::POSITION_CONTROL:
    {
        if (workModMap[slaveAddr] != MotorDevice::Mode::POSITION_CONTROL)
        {
            workModMap[slaveAddr] = MotorDevice::Mode::POSITION_CONTROL;
        }

        int16_t position_raw = static_cast<uint16_t>(
            static_cast<int16_t>(cmd.targetPosition / motor.config.positionUnit));

        sm_st.WritePosEx((uint8_t)slaveAddr, position_raw + 2047, 0, 0);

        lastCallTimeUs = getBusTimestamp();
    }
    break;
    }
    // std::cout << "send: " << motor.timing.lastCtrlTimeMs << std::endl;

    return true;
}

// ========================
// 核心：读取状态
// ========================
bool SCServoMotorBus::readMotorStatus(MotorDevice &motor)
{
    if (!isReady())
        return false;

    int slaveAddr = static_cast<int>(motor.addressOrNodeId);

    sm_st.FeedBack((uint8_t)slaveAddr);

    // 解析到 StatusData
    MotorDevice::StatusData status;
    status.actualPosition = static_cast<int32_t>(sm_st.ReadPos(-1) - 2047) *
                            motor.config.positionUnit;

    // 时间戳
    status.lastUpdateTimeUs = getBusTimestamp();

    // 更新到电机
    motor.updateStatus(status);

    // std::cout << "update: " << motor.timing.lastCtrlTimeMs << std::endl;
    // std::this_thread::sleep_for(std::chrono::microseconds(delaytime));
    return true;
}

bool SCServoMotorBus::syncTrigger()
{
    // Modbus RTU 无硬件同步，可用广播写“同步触发寄存器”
    // 或简单返回 true，依赖软件时间戳同步
    return true;
}

uint64_t SCServoMotorBus::getBusTimestamp() const
{
    auto now = std::chrono::steady_clock::now();
    return std::chrono::duration_cast<std::chrono::microseconds>(
               now.time_since_epoch())
        .count();
}