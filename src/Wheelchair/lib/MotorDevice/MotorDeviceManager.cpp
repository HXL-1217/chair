#include "MotorDeviceManager.h"
#include <sstream>
#include <algorithm>

MotorDeviceManager::MotorDeviceManager() = default;
MotorDeviceManager::~MotorDeviceManager() = default;

// ========================
// 总线管理
// ========================

bool MotorDeviceManager::addBus(std::shared_ptr<MotorBus> bus) {
    if (!bus || !bus->init()) return false;
    std::lock_guard<std::mutex> lock(mtx_);
    buses_.push_back(bus);
    return true;
}

bool MotorDeviceManager::removeBus(MotorBus* busPtr) {
    std::lock_guard<std::mutex> lock(mtx_);
    auto it = std::find_if(buses_.begin(), buses_.end(),
        [busPtr](const std::shared_ptr<MotorBus>& b) { return b.get() == busPtr; });
    if (it == buses_.end()) return false;
    (*it)->deinit();
    buses_.erase(it);
    return true;
}

size_t MotorDeviceManager::getBusCount() const {
    std::lock_guard<std::mutex> lock(mtx_);
    return buses_.size();
}

// ========================
// 电机注册（跨总线）
// ========================

bool MotorDeviceManager::registerMotor(std::shared_ptr<MotorDevice> motor, std::shared_ptr<MotorBus> bus) {
    if (!motor || !bus) return false;

    // 先添加到总线
    if (!bus->addMotor(motor)) return false;

    // 再注册到全局表
    std::lock_guard<std::mutex> lock(mtx_);
    if (motors_.find(motor->id) != motors_.end()) {
        // 已存在，先移除旧的
        unregisterMotor(motor->id);
    }
    motors_[motor->id] = {motor, bus};
    return true;
}

bool MotorDeviceManager::unregisterMotor(uint16_t motorId) {
    std::lock_guard<std::mutex> lock(mtx_);
    auto it = motors_.find(motorId);
    if (it == motors_.end()) return false;

    auto& [motor, bus] = it->second;
    bus->removeMotor(motorId);
    motors_.erase(it);
    return true;
}

std::shared_ptr<MotorDevice> MotorDeviceManager::getMotorById(uint16_t id) const {
    std::lock_guard<std::mutex> lock(mtx_);
    auto it = motors_.find(id);
    if (it == motors_.end()) return nullptr;
    return it->second.first;
}

size_t MotorDeviceManager::getTotalMotorCount() const {
    std::lock_guard<std::mutex> lock(mtx_);
    return motors_.size();
}

// ========================
// 全局控制
// ========================

bool MotorDeviceManager::broadcastEmergencyStop() {
    bool allSuccess = true;
    std::lock_guard<std::mutex> lock(mtx_);
    for (auto& [id, motor_bus] : motors_) {
        auto& [motor, bus] = motor_bus;
        MotorDevice::ControlData cmd = motor->getControlCommand();
        cmd.emergencyStop = true;
        motor->updateControl(cmd); // 通过双缓冲安全写入
    }
    return allSuccess;
}

bool MotorDeviceManager::broadcastEnable(bool enable) {
    bool allSuccess = true;
    std::lock_guard<std::mutex> lock(mtx_);
    for (auto& [id, motor_bus] : motors_) {
        auto& [motor, bus] = motor_bus;
        MotorDevice::ControlData cmd = motor->getControlCommand();
        cmd.enable = enable;
        motor->updateControl(cmd);
    }
    return allSuccess;
}

bool MotorDeviceManager::syncAllBuses() {
    bool allSuccess = true;
    std::lock_guard<std::mutex> lock(mtx_);
    for (auto& bus : buses_) {
        if (!bus->syncTrigger()) {
            allSuccess = false;
        }
    }
    return allSuccess;
}

// ========================
// 状态快照
// ========================

MotorDeviceManager::GlobalStatusSnapshot MotorDeviceManager::getStatusSnapshot() const {
    GlobalStatusSnapshot snapshot;
    snapshot.timestampUs = std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
    snapshot.allReadyForSync = true;

    std::lock_guard<std::mutex> lock(mtx_);
    for (auto& [id, motor_bus] : motors_) {
        auto& [motor, bus] = motor_bus;
        auto status = motor->getStatusSnapshot(); // 线程安全读取Front缓冲
        snapshot.motorStatuses.emplace_back(id, status);
        if (!motor->isReadyForSync()) {
            snapshot.allReadyForSync = false;
        }
    }

    return snapshot;
}

// // ========================
// // 周期任务协调
// // ========================

// bool MotorDeviceManager::updateAllBuses() {
//     bool allSuccess = true;
//     std::lock_guard<std::mutex> lock(mtx_);
//     for (auto& bus : buses_) {
//         if (!bus->updateAllMotors()) {
//             allSuccess = false;
//         }
//     }
//     return allSuccess;
// }

// ========================
// 错误管理
// ========================

void MotorDeviceManager::clearAllErrors() {
    std::lock_guard<std::mutex> lock(mtx_);
    for (auto& bus : buses_) {
        bus->clearErrors();
    }
}

std::vector<BusError> MotorDeviceManager::getAllErrors() const {
    std::vector<BusError> allErrors;
    std::lock_guard<std::mutex> lock(mtx_);
    for (auto& bus : buses_) {
        auto errors = bus->getErrors();
        allErrors.insert(allErrors.end(), errors.begin(), errors.end());
    }
    return allErrors;
}

void MotorDeviceManager::registerGlobalErrorCallback(ErrorCallback cb) {
    globalErrorCallback_ = cb;
    // 为每个总线注册转发回调
    std::lock_guard<std::mutex> lock(mtx_);
    for (auto& bus : buses_) {
        bus->registerErrorCallback([this](const BusError& err) {
            if (globalErrorCallback_) {
                globalErrorCallback_(err);
            }
        });
    }
}

void MotorDeviceManager::reportGlobalError(const BusError& err) {
    if (globalErrorCallback_) {
        globalErrorCallback_(err);
    }
}

// ========================
// 调试信息
// ========================

std::string MotorDeviceManager::getSystemInfo() const {
    std::ostringstream oss;
    std::lock_guard<std::mutex> lock(mtx_);

    oss << "=== Motor System Info ===\n";
    oss << "Total Buses: " << buses_.size() << "\n";
    oss << "Total Motors: " << motors_.size() << "\n";

    for (auto& bus : buses_) {
        oss << "Bus Type: " << MotorDevice::getBusTypeName(bus->getMotorById(1) ? bus->getMotorById(1)->busType : BusType::UNKNOWN)
            << ", Motors: " << bus->getMotorCount() << "\n";
    }

    return oss.str();
}