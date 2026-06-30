#include "MotorBus.h"
#include <algorithm>
#include <thread>
#include <chrono>

MotorBus::MotorBus() = default;

MotorBus::~MotorBus()
{
    stop(); // 确保线程安全退出
}

// ========================
// 线程控制
// ========================

bool MotorBus::start()
{
    std::lock_guard<std::mutex> lock(threadMutex_);
    if (running_.load())
        return false;

    running_ = true;
    workerThread_ = std::thread(&MotorBus::run, this);

    // // 可选：设置实时优先级（Linux）测试时wsl中存在bug
    // #ifdef __linux__
    //     setThreadPriority(50); // 中等实时优先级
    // #endif

    return true;
}

void MotorBus::stop()
{
    {
        std::lock_guard<std::mutex> lock(threadMutex_);
        if (!running_.load())
            return;
        running_ = false;
    }

    if (workerThread_.joinable())
    {
        workerThread_.join();
    }
}

// ========================
// 线程主循环 run()
// ========================

void MotorBus::run()
{
    while (running_.load())
    {
        auto start = std::chrono::steady_clock::now();

        if (ready_.load())
        {
            updateAllMotors(); // 每1ms调度一次
        }

        // 精确休眠到1ms周期
        auto end = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
        auto sleepTime = std::chrono::microseconds(1000) - elapsed; // 1ms = 1000us

        if (sleepTime > std::chrono::microseconds(0))
        {
            std::this_thread::sleep_for(sleepTime);
        }
        else
        {
            // 周期超时，可记录警告（不影响功能）
            // reportError(0, 0xFE, "Cycle overrun");
        }
    }
}

// ========================
// 设备管理（基类实现）
// ========================

bool MotorBus::addMotor(std::shared_ptr<MotorDevice> motor)
{
    if (!motor)
        return false;
    if (motors_.find(motor->id) != motors_.end())
    {
        reportError(motor->id, 0x100, "Motor ID already exists");
        return false;
    }
    motors_[motor->id] = motor;
    return true;
}

bool MotorBus::removeMotor(uint16_t motorId)
{
    auto it = motors_.find(motorId);
    if (it == motors_.end())
        return false;
    motors_.erase(it);
    return true;
}

size_t MotorBus::getMotorCount() const
{
    return motors_.size();
}

std::shared_ptr<MotorDevice> MotorBus::getMotorById(uint16_t id)
{
    auto it = motors_.find(id);
    if (it == motors_.end())
        return nullptr;
    return it->second;
}

// ========================
// updateAllMotors() 调度器（基类实现）
// ========================

bool MotorBus::updateAllMotors()
{
    auto now = std::chrono::steady_clock::now();
    auto nowMs = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count();

    bool anySuccess = true;

    for (auto &[id, motor] : motors_)
    {
        if (!motor)
            continue;

        // ============ 心跳写入周期检查 ============
        if (motor->config.updateBeatTimeMs > 0)
        {
            if (nowMs - motor->timing.lastBeatTimeMs >= motor->config.updateBeatTimeMs)
            {
                motor->timing.lastBeatTimeMs = nowMs; // 更新触发时间
                if (!writeMotorBeat(*motor))
                {
                    reportError(id, 0x01, "Write beat failed");
                    anySuccess = false;
                }
            }
        }

        // ============ 控制写入周期检查 ============
        if (motor->config.updateCtrlTimeMs > 0)
        {
            if (nowMs - motor->timing.lastCtrlTimeMs >= motor->config.updateCtrlTimeMs)
            {
            
                motor->timing.lastCtrlTimeMs = nowMs; // 更新触发时间
                motor->swapControlBuffer();
                if (!writeMotorControl(*motor))
                {
                    reportError(id, 0x01, "Write control failed");
                    anySuccess = false;
                }
            }
        }

        // ============ 状态读取周期检查 ============
        if (motor->config.updateReadTimeMs > 0)
        {
            if (nowMs - motor->timing.lastReadTimeMs >= motor->config.updateReadTimeMs)
            {
                motor->timing.lastReadTimeMs = nowMs; // 更新触发时间
                if (!readMotorStatus(*motor))
                {
                    reportError(id, 0x02, "Read status failed");
                    anySuccess = false;
                }
                else
                {
                    motor->swapStatusBuffer();
                }
            }
        }
    }

    return anySuccess;
}

// ========================
// 错误管理
// ========================

void MotorBus::clearErrors()
{
    errors_.clear();
}

std::vector<BusError> MotorBus::getErrors() const
{
    return errors_;
}

void MotorBus::registerErrorCallback(ErrorCallback cb)
{
    errorCallback_ = cb;
}

void MotorBus::reportError(uint16_t motorId, uint32_t code, const std::string &msg)
{
    BusError err;
    err.motorId = motorId;
    err.errorCode = code;
    err.message = msg;
    err.timestamp = getBusTimestamp();

    {
        std::lock_guard<std::mutex> lock(threadMutex_); // 保护 errors_
        errors_.push_back(err);
    }

    if (errorCallback_)
    {
        errorCallback_(err);
    }
}

// ========================
// Linux 实时优先级设置（可选）
// ========================

#ifdef __linux__
#include <sched.h>
#include <sys/resource.h>

void MotorBus::setThreadPriority(int priority)
{
    struct sched_param param;
    param.sched_priority = priority;

    if (pthread_setschedparam(workerThread_.native_handle(), SCHED_FIFO, &param) != 0)
    {
        reportError(0, 0xFD, "Failed to set real-time priority");
    }
}
#endif