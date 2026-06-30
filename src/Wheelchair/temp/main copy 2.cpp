#include "ZLAC8015MotorBus.h"
#include "SCServoMotorBus.h"
#include <iostream>
#include "UdpComm.h"
#include "DualSteeredWheelchair.h"
#include "OdometryPublisher.h"
#ifdef USE_ROS2
#include <rclcpp/rclcpp.hpp>
#endif

#include <nlohmann/json.hpp>
#include <chrono>
#include <thread>

using json = nlohmann::json;

uint8_t update_key = 0;  // 修正拼写
int8_t mod = -1;
double speed_vx = 0;
double speed_vy = 0;
double speed_rz = 0;

void onMessageReceived(const std::string &message, const std::string &ip, int port)
{
    try
    {
        json jsonData = json::parse(message);
        if (jsonData.contains("speed")) {
            auto& data = jsonData["speed"];
            speed_vx = data[0];
            speed_vy = data[1];
            speed_rz = data[2];
        }
        if (jsonData.contains("key")) {
            update_key = jsonData["key"];
        }
        if (jsonData.contains("mod")) {
            mod = jsonData["mod"];
        }
    }
    catch (const json::parse_error &e) {
        std::cerr << "JSON解析错误: " << e.what() << std::endl;
        std::cerr << "错误数据: " << message << std::endl;
    }
}

int main()
{
    UdpComm udpComm(5353, onMessageReceived);
    if (!udpComm.start()) {
        std::cerr << "启动UDP通信失败" << std::endl;
        return -1;
    }

    const double wheel_track = 0.75;
    DualSteeredWheelchair chair(wheel_track, 0.005);

    // 使用简化版 OdometryPublisher（无 wheel_positions）
    OdometryPublisher odom_pub(20.0, "odom", "base_link");

#ifdef USE_ROS2
    rclcpp::init(0, nullptr);
    auto node = std::make_shared<rclcpp::Node>("wheelchair_motor_ctrl");
    odom_pub.attachNode(node);
#endif

    auto bus = std::make_shared<ZLAC8015MotorBus>("/dev/ttyUSB1", 115200, 'N', 8, 1);
    auto bus2 = std::make_shared<SCServoMotorBus>("/dev/ttyUSB0", 500000, 'N', 8, 1);

    if (!bus->init() || !bus2->init()) {
        std::cerr << "总线初始化失败!" << std::endl;
        return -1;
    }

    auto motor1 = std::make_shared<MotorDevice>(1, "Wheel-R", BusType::MODBUS_RTU, (1 << 8) | 0);
    motor1->config.velocityUnit = 7.27802286e-3;
    motor1->config.updateCtrlTimeMs = 50;
    motor1->config.updateReadTimeMs = 50;

    auto motor2 = std::make_shared<MotorDevice>(2, "Wheel-L", BusType::MODBUS_RTU, (1 << 8) | 1);
    motor2->config.velocityUnit = -7.27802286e-3;
    motor2->config.updateCtrlTimeMs = 50;
    motor2->config.updateReadTimeMs = 50;

    auto motor3 = std::make_shared<MotorDevice>(3, "Servo-1", BusType::CUSTOM, 1);
    motor3->config.positionUnit = 3.1415926 / 2048;
    motor3->config.updateCtrlTimeMs = 33;
    motor3->config.updateReadTimeMs = 33;

    auto motor4 = std::make_shared<MotorDevice>(4, "Servo-2", BusType::CUSTOM, 2);
    motor4->config.positionUnit = 3.1415926 / 2048;
    motor4->config.updateCtrlTimeMs = 33;
    motor4->config.updateReadTimeMs = 33;

    bus->addMotor(motor1);
    bus->addMotor(motor2);
    bus2->addMotor(motor3);
    bus2->addMotor(motor4);
    std::cout << "addMotor" << std::endl;

    MotorDevice::ControlData cmd;
    cmd.controlMode = MotorDevice::Mode::VELOCITY_CONTROL;
    cmd.targetVelocity = 0;
    cmd.enable = true;
    motor1->updateControl(cmd);
    motor2->updateControl(cmd);

    MotorDevice::ControlData cmd2;
    cmd2.controlMode = MotorDevice::Mode::POSITION_CONTROL;
    cmd2.targetPosition = 0;
    cmd2.enable = true;
    motor3->updateControl(cmd2);
    motor4->updateControl(cmd2);

    bus->start();
    bus2->start();
    std::cout << "start" << std::endl;

    auto start_time = std::chrono::steady_clock::now();

    while (update_key == 0)
    {
        WheelCommands chair_cmd;
        switch (mod)
        {
        case 0: // 差速
            chair.resetSteeringAngles();
            chair_cmd = chair.computeWheelCommands(speed_vx, 0, speed_rz);
            break;
        case 1: // 蟹行
            chair_cmd = chair.computeWheelCommands(speed_vx, speed_vy, 0);
            break;
        case 2: // 旋转
            chair.resetSteeringAngles();
            chair_cmd = chair.computeWheelCommands(0, 0, speed_rz);
            break;
        case 3: // 混合
            chair_cmd = chair.computeWheelCommands(speed_vx, speed_vy, speed_rz);
            break;
        case 4: // 平移
            chair.resetToLateral();
            chair_cmd = chair.computeWheelCommands(0, speed_vy, 0);
            break;
        default:
            chair.resetSteeringAngles();
            chair_cmd = chair.computeWheelCommands(0, 0, 0);
            break;
        }

        // 发送控制指令
        cmd.targetVelocity = chair_cmd.vl;
        motor2->updateControl(cmd);
        cmd.targetVelocity = chair_cmd.vr;
        motor1->updateControl(cmd);
        cmd2.targetPosition = chair_cmd.thl;
        motor3->updateControl(cmd2);
        cmd2.targetPosition = chair_cmd.thr;
        motor4->updateControl(cmd2);

        // 获取反馈（actualVelocity/Position 应已为 SI 单位）
        double vl = motor2->getStatusSnapshot().actualVelocity; // 左轮
        double vr = motor1->getStatusSnapshot().actualVelocity; // 右轮
        double thl = motor3->getStatusSnapshot().actualPosition; // 左舵角
        double thr = motor4->getStatusSnapshot().actualPosition; // 右舵角

        // 正运动学
        auto bv = chair.computeForwardKinematics(vl, vr, thl, thr);

        // 获取当前时间（秒）
        auto now = std::chrono::steady_clock::now();
        double current_time_sec = std::chrono::duration<double>(now.time_since_epoch()).count();

        // 积分并发布
        odom_pub.integrateAndPublish(bv.vx, bv.vy, bv.omega, current_time_sec);

#ifdef USE_ROS2
        rclcpp::spin_some(node);
#endif

        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    std::cout << "end!" << std::endl;

    // 安全停车
    cmd.targetVelocity = 0;
    motor1->updateControl(cmd);
    motor2->updateControl(cmd);
    cmd2.targetPosition = 0;
    motor3->updateControl(cmd2);
    motor4->updateControl(cmd2);

    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    bus->stop();
    bus2->stop();

#ifdef USE_ROS2
    rclcpp::shutdown();
#endif
    return 0;
}