#pragma once

#include <cstdint>
#include <string>
#include <atomic>
#include <cstring> // for memcpy

enum class BusType : uint8_t
{
    UNKNOWN = 0,
    CAN,
    CANOPEN,
    MODBUS_RTU,
    MODBUS_TCP,
    PULSE_DIRECTION,
    ETHERCAT,
    PROFINET,
    CUSTOM
};

struct MotorDevice
{
    enum class Mode : uint8_t
    {
        DISABLED = 0,
        POSITION_CONTROL,
        VELOCITY_CONTROL,
        TORQUE_CONTROL,
        HOMING,
        PROFILE_POSITION,
        PROFILE_VELOCITY,
        CYCLIC_SYNC_POSITION,
        CYCLIC_SYNC_VELOCITY,
        CYCLIC_SYNC_TORQUE
    };
    // === 基础信息（不变量，无需缓冲）===
    const uint16_t id;
    const std::string name;
    const BusType busType;
    const uint16_t addressOrNodeId;

    // 构造函数（基础信息在创建后不可变）
    MotorDevice(uint16_t _id, const std::string &_name, BusType _busType, uint16_t _addr)
        : id(_id), name(_name), busType(_busType), addressOrNodeId(_addr)
    {
        reset();
    }

    // === 控制数据结构体（同前）===
    struct ControlData
    {
        Mode controlMode = Mode::DISABLED;
        float targetPosition = 0.0f;
        float targetVelocity = 0.0f;
        float targetTorque = 0.0f;
        float maxVelocity = 100.0f;
        float maxAcceleration = 500.0f;
        float maxDeceleration = 500.0f;
        float jerk = 0.0f;
        float gearRatio = 1.0f;
        int16_t gearMasterId = -1;
        bool enable = false;
        bool emergencyStop = false;
        uint32_t userTag = 0;

        void reset()
        {
            std::memset(this, 0, sizeof(ControlData));
            controlMode = Mode::DISABLED;
        }
    };

    // === 状态数据结构体（同前）===
    struct StatusData
    {
        // Mode controlMode = Mode::DISABLED;
        float actualPosition = 0.0f;
        float actualVelocity = 0.0f;
        float actualTorque = 0.0f;
        float positionError = 0.0f;
        float velocityError = 0.0f;
        bool isEnabled = false;
        bool isInMotion = false;
        bool isInPosition = false;
        bool isHomed = false;
        bool isFault = false;
        uint32_t faultCode = 0;
        float temperature = 0.0f;
        float busVoltage = 0.0f;
        // uint32_t ErrorCode = 0;
        uint64_t lastUpdateTimeUs = 0;
        uint64_t syncTimestampUs = 0;

        void reset() { std::memset(this, 0, sizeof(StatusData)); }
    };

    // === 配置数据（初始化后不变）===
    struct Config
    {
        float positionUnit = 1.0f;
        float velocityUnit = 1.0f;
        float torqueUnit = 1.0f;
        float positionTolerance = 0.1f;
        bool useEncoder = true;
        bool reverseDirection = false;
        std::string model;
        uint16_t updateBeatTimeMs = 0; // 心跳指令写入周期（ms），默认0ms
        uint16_t updateCtrlTimeMs = 10;  // 控制指令写入周期（ms），默认10ms
        uint16_t updateReadTimeMs = 10;  // 状态读取周期（ms），默认10ms

        void reset()
        {
            positionUnit = 1.0f;
            velocityUnit = 1.0f;
            torqueUnit = 1.0f;
            positionTolerance = 0.1f;
            useEncoder = true;
            reverseDirection = false;
            model.clear();
        }
    } config;

    // === 时间管理数据 ===
    struct TimingData
    {
        uint64_t lastBeatTimeMs = 0; // 上次心跳写入时间（ms）
        uint64_t lastCtrlTimeMs = 0; // 上次控制写入时间（ms）
        uint64_t lastReadTimeMs = 0; // 上次状态读取时间（ms）
    } timing;

    // ============ 双缓冲区设计 ============

    // 控制数据双缓冲
    ControlData controlFront;                      // 总线线程读取（当前生效的控制指令）
    ControlData controlBack;                       // 控制线程写入（下一周期生效）
    std::atomic<bool> controlSwapRequested{false}; // 请求交换缓冲区

    // 状态数据双缓冲
    StatusData statusFront;                       // 控制/监控线程读取（最新状态快照）
    StatusData statusBack;                        // 总线线程写入（当前采集的状态）
    std::atomic<bool> statusSwapRequested{false}; // 请求交换缓冲区

    // 交换控制缓冲区（由总线线程在周期开始时调用）
    void swapControlBuffer()
    {
        if (controlSwapRequested.load(std::memory_order_acquire))
        {
            std::memcpy(&controlFront, &controlBack, sizeof(ControlData));
            controlSwapRequested.store(false, std::memory_order_release);
        }
    }

    // 交换状态缓冲区（由总线线程在周期结束时调用）
    void swapStatusBuffer()
    {
        if (statusSwapRequested.load(std::memory_order_acquire))
        {
            std::memcpy(&statusFront, &statusBack, sizeof(StatusData));
            statusSwapRequested.store(false, std::memory_order_release);
        }
    }

    // 控制线程调用：更新控制目标（写入Back缓冲）
    void updateControl(const ControlData &newControl)
    {
        std::memcpy(&controlBack, &newControl, sizeof(ControlData));
        controlSwapRequested.store(true, std::memory_order_release);
    }

    // 控制线程调用：获取当前状态快照（读Front缓冲）
    StatusData getStatusSnapshot() const
    {
        StatusData snapshot;
        std::memcpy(&snapshot, &statusFront, sizeof(StatusData));
        return snapshot;
    }

    // 总线线程调用：获取当前控制指令（读Front缓冲）
    ControlData getControlCommand() const
    {
        ControlData cmd;
        std::memcpy(&cmd, &controlFront, sizeof(ControlData));
        return cmd;
    }

    // 总线线程调用：更新采集到的状态（写入Back缓冲）
    void updateStatus(const StatusData &newStatus)
    {
        std::memcpy(&statusBack, &newStatus, sizeof(StatusData));
        statusSwapRequested.store(true, std::memory_order_release);
    }

    // 重置所有数据
    void reset()
    {
        timing.lastBeatTimeMs = 0;
        timing.lastCtrlTimeMs = 0;
        timing.lastReadTimeMs = 0;
        controlFront.reset();
        controlBack.reset();
        statusFront.reset();
        statusBack.reset();
        config.reset();
        controlSwapRequested.store(false);
        statusSwapRequested.store(false);
    }

    // === 辅助方法（基于Front缓冲区，线程安全）===

    [[nodiscard]] bool isReadyForSync() const
    {
        return statusFront.isEnabled && !statusFront.isFault && controlFront.enable;
    }

    [[nodiscard]] float getPositionError() const
    {
        return controlFront.targetPosition - statusFront.actualPosition;
    }

    [[nodiscard]] bool isInPositionRange() const
    {
        return std::abs(getPositionError()) <= config.positionTolerance;
    }

    static const char *getModeName(Mode mode);
    static const char *getBusTypeName(BusType type);
};

// 实现静态方法（可放在 .cpp 或内联）
inline const char *MotorDevice::getModeName(Mode mode)
{
    switch (mode)
    {
    case Mode::DISABLED:
        return "DISABLED";
    case Mode::POSITION_CONTROL:
        return "POSITION_CONTROL";
    case Mode::VELOCITY_CONTROL:
        return "VELOCITY_CONTROL";
    case Mode::TORQUE_CONTROL:
        return "TORQUE_CONTROL";
    case Mode::HOMING:
        return "HOMING";
    case Mode::PROFILE_POSITION:
        return "PROFILE_POSITION";
    case Mode::PROFILE_VELOCITY:
        return "PROFILE_VELOCITY";
    case Mode::CYCLIC_SYNC_POSITION:
        return "CYCLIC_SYNC_POSITION";
    case Mode::CYCLIC_SYNC_VELOCITY:
        return "CYCLIC_SYNC_VELOCITY";
    case Mode::CYCLIC_SYNC_TORQUE:
        return "CYCLIC_SYNC_TORQUE";
    default:
        return "UNKNOWN";
    }
}

inline const char *MotorDevice::getBusTypeName(BusType type)
{
    switch (type)
    {
    case BusType::CAN:
        return "CAN";
    case BusType::CANOPEN:
        return "CANOPEN";
    case BusType::MODBUS_RTU:
        return "MODBUS_RTU";
    case BusType::MODBUS_TCP:
        return "MODBUS_TCP";
    case BusType::PULSE_DIRECTION:
        return "PULSE_DIRECTION";
    case BusType::ETHERCAT:
        return "ETHERCAT";
    case BusType::PROFINET:
        return "PROFINET";
    case BusType::CUSTOM:
        return "CUSTOM";
    default:
        return "UNKNOWN";
    }
}