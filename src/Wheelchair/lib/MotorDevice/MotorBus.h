#pragma once

#include <vector>
#include <memory>
#include <map>
#include <thread>
#include <atomic>
#include <chrono>
#include <mutex>
#include <functional>
#include "MotorDevice.h"

struct BusError {
    uint16_t motorId = 0;
    uint32_t errorCode = 0;
    std::string message;
    uint64_t timestamp = 0;
};

using ErrorCallback = std::function<void(const BusError&)>;

class MotorBus {
public:
    MotorBus();
    virtual ~MotorBus();

    // === 生命周期 ===
    virtual bool init() = 0;
    virtual void deinit() = 0;
    virtual bool isReady() const = 0;

    // === 线程控制 ===
    bool start();   // 启动内部线程
    void stop();    // 停止线程（阻塞直到线程结束）

    // === 设备管理（基类实现）===
    bool addMotor(std::shared_ptr<MotorDevice> motor);
    bool removeMotor(uint16_t motorId);
    size_t getMotorCount() const;
    std::shared_ptr<MotorDevice> getMotorById(uint16_t id);

    // === 核心调度函数（每1ms调用一次）===
    bool updateAllMotors(); // 基类实现调度逻辑

    // === 读写分离接口（由子类实现）===
    virtual bool writeMotorBeat(const MotorDevice& motor) = 0;
    virtual bool writeMotorControl(const MotorDevice& motor) = 0;
    virtual bool readMotorStatus(MotorDevice& motor) = 0;

    // === 同步支持 ===
    virtual bool syncTrigger() = 0;
    virtual uint64_t getBusTimestamp() const = 0;

    // === 错误管理 ===
    virtual void clearErrors();
    virtual std::vector<BusError> getErrors() const;
    virtual void registerErrorCallback(ErrorCallback cb);

protected:
    std::map<uint16_t, std::shared_ptr<MotorDevice>> motors_;
    mutable std::vector<BusError> errors_;
    ErrorCallback errorCallback_ = nullptr;
    std::atomic<bool> ready_{false};

    // 时间基准
    uint64_t baseTimeMs_ = 0;

    // 线程管理
    std::thread workerThread_;
    std::atomic<bool> running_{false};
    std::mutex threadMutex_; // 保护线程启停

    // 内部：线程主循环
    void run();

    // 内部：报告错误
    void reportError(uint16_t motorId, uint32_t code, const std::string& msg);

    // 可选：设置线程优先级（Linux/POSIX）
    #ifdef __linux__
    void setThreadPriority(int priority); // SCHED_FIFO, priority 1-99
    #endif
};