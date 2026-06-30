#pragma once

#include "MotorBus.h"
#include <modbus.h>
#include <string>

class ZLAC8015MotorBus : public MotorBus
{
public:
    ZLAC8015MotorBus(
        const std::string &device, // 如 "/dev/ttyS1"
        int baud,                  // 波特率
        char parity,               // 'N', 'E', 'O'
        int data_bit,              // 通常 8
        int stop_bit               // 通常 1
    );

    ~ZLAC8015MotorBus();

    // === MotorBus 接口实现 ===
    bool init() override;
    void deinit() override;
    bool isReady() const override;

    bool writeMotorBeat(const MotorDevice &motor) override;
    bool writeMotorControl(const MotorDevice &motor) override;
    bool readMotorStatus(MotorDevice &motor) override;

    bool syncTrigger() override;
    uint64_t getBusTimestamp() const override;

private:
    std::string device_;
    int baud_;
    char parity_;
    int data_bit_;
    int stop_bit_;

    std::map<int32_t, int16_t> heartcountMap;
    std::map<int32_t, MotorDevice::Mode> workModMap;
    uint64_t lastCallTimeUs = 0;

    modbus_t *ctx_ = nullptr;
    bool connected_ = false;

    // 辅助函数
    bool reconnect();
    void closeConnection();
    void reportModbusError(const MotorDevice &motor, const std::string &operation);
};