#pragma once

#include <vector>
#include <memory>
#include <map>
#include <mutex>
#include <functional>
#include "MotorBus.h"

class MotorDeviceManager {
public:
    // === 构造与析构 ===
    MotorDeviceManager();
    ~MotorDeviceManager();

    // === 总线管理 ===
    bool addBus(std::shared_ptr<MotorBus> bus);
    bool removeBus(MotorBus* bus);
    size_t getBusCount() const;

    // === 电机统一管理（跨总线）===
    bool registerMotor(std::shared_ptr<MotorDevice> motor, std::shared_ptr<MotorBus> bus);
    bool unregisterMotor(uint16_t motorId);
    std::shared_ptr<MotorDevice> getMotorById(uint16_t id) const;
    size_t getTotalMotorCount() const;

    // === 全局控制 ===
    bool broadcastEmergencyStop();      // 向所有电机发送急停
    bool broadcastEnable(bool enable);  // 批量使能/禁用
    bool syncAllBuses();                // 触发所有总线同步（用于多轴插补起点对齐）

    // === 状态快照（线程安全）===
    struct GlobalStatusSnapshot {
        uint64_t timestampUs;
        std::vector<std::pair<uint16_t, MotorDevice::StatusData>> motorStatuses;
        bool allReadyForSync; // 是否所有电机都准备好同步
    };

    GlobalStatusSnapshot getStatusSnapshot() const;

    // // === 周期任务（可选：由外部定时器调用，或内部线程）===
    // bool updateAllBuses(); // 调用所有总线的 updateAllMotors()

    // === 错误管理 ===
    void clearAllErrors();
    std::vector<BusError> getAllErrors() const;
    void registerGlobalErrorCallback(ErrorCallback cb);

    // === 调试与信息 ===
    std::string getSystemInfo() const; // 返回JSON或文本格式的系统状态

private:
    mutable std::mutex mtx_; // 保护 motors_ 和 buses_

    // 存储所有总线
    std::vector<std::shared_ptr<MotorBus>> buses_;

    // 全局电机注册表（ID → 电机对象 + 所属总线）
    std::map<uint16_t, std::pair<std::shared_ptr<MotorDevice>, std::shared_ptr<MotorBus>>> motors_;

    // 全局错误回调
    ErrorCallback globalErrorCallback_ = nullptr;

    // 辅助：广播错误
    void reportGlobalError(const BusError& err);
};