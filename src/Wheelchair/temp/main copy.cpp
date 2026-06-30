#include "OID485MotorBus.h"
#include <iostream>
int main()
{

    // 创建总线
    auto bus = std::make_shared<OID485MotorBus>("/dev/ttyAS7", 500000, 'N', 8, 1);

    if (!bus->init())
    {
        printf("Modbus RTU init failed!\n");
        return -1;
    }

    // 创建电机（从站地址=1）
    auto motor1 = std::make_shared<MotorDevice>(1, "Axis-1", BusType::MODBUS_RTU, 1);
    motor1->config.velocityUnit = 0.1;
    motor1->config.updateBeatTimeMs = 500;
    motor1->config.updateCtrlTimeMs = 50;
    motor1->config.updateReadTimeMs = 200;
    auto motor2 = std::make_shared<MotorDevice>(2, "Axis-2", BusType::MODBUS_RTU, 2);
    motor2->config.velocityUnit = 1;
    motor2->config.updateBeatTimeMs = 500;
    motor2->config.updateCtrlTimeMs = 50;
    motor2->config.updateReadTimeMs = 200;

    bus->addMotor(motor1);
    bus->addMotor(motor2);
    printf("addMotor\r\n");

    // 设置速度
    MotorDevice::ControlData cmd;
    cmd.controlMode = MotorDevice::Mode::VELOCITY_CONTROL; // PROFILE_VELOCITY
    cmd.targetVelocity = 0;
    cmd.enable = true;
    motor1->updateControl(cmd);
    motor2->updateControl(cmd);

    bus->start(); // 启动内部1ms调度线程
    printf("start\r\n");
    // 运行...
    std::cout << "run!" << std::endl;
    for (int32_t i = 0; i < 200; i++)
    {
        cmd.targetVelocity = 20.0 * (100 - i);

        auto md1 = bus->getMotorById(1);
        md1->updateControl(cmd);
        auto status1 = md1->getStatusSnapshot();

        auto md2 = bus->getMotorById(2);
        md2->updateControl(cmd);
        auto status2 = md2->getStatusSnapshot();

        std::cout << "Axis-1: " << status1.actualVelocity << "; Axis-2:" << status2.actualVelocity << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    std::cout << "end!" << std::endl;

    bus->stop();
    return 0;
}