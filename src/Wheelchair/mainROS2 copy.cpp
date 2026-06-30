#include "ZLAC8015MotorBus.h"
#include "SCServoMotorBus.h"
#include <iostream>
#include "UdpComm.h"
#include "DualSteeredWheelchair.h"
#include <nlohmann/json.hpp>
#include <chrono>
#include <thread>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

using namespace std::chrono_literals;
using json = nlohmann::json;

class DSWChair : public rclcpp::Node
{
public:
  DSWChair(
    std::shared_ptr<MotorDevice> wr,
    std::shared_ptr<MotorDevice> wl,
    std::shared_ptr<MotorDevice> sr,
    std::shared_ptr<MotorDevice> sl)
  : Node("odom_publisher"),
    motor_WR(wr), motor_WL(wl), motor_SR(sr), motor_SL(sl), chair(0.55, 0.005)
  {
    // 初始化变量
    x_ = 0.0;
    y_ = 0.0;
    theta_ = 0.0;
    // last_time_ = this->now();
    initialized_ = false;



    cmd_W.controlMode = MotorDevice::Mode::VELOCITY_CONTROL;
    cmd_W.targetVelocity = 0;
    cmd_W.enable = true;
    cmd_S.controlMode = MotorDevice::Mode::POSITION_CONTROL;
    cmd_S.targetPosition = 0;
    cmd_S.enable = true;

    // 订阅 cmd_vel
    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10,
      std::bind(&DSWChair::cmdVelCallback, this, std::placeholders::_1));

    // 发布 odom
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);

    // 定时器：每 50ms 发布一次 odom（20Hz）
    timer_ = this->create_wall_timer(50ms,
      std::bind(&DSWChair::publishOdometry, this));

    // TF 广播器（可选，用于发布 odom -> base_link 的变换）
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    RCLCPP_INFO(this->get_logger(), "DSWChair node started.");
  }

private:
  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    // 存储最新的速度命令
    ctrl_vel_.linear.x = msg->linear.x;
    ctrl_vel_.linear.y = msg->linear.y;
    ctrl_vel_.angular.z = msg->angular.z;

    WheelCommands chair_cmd;
    // chair.resetSteeringAngles();
    chair_cmd = chair.computeWheelCommands(ctrl_vel_.linear.x, ctrl_vel_.linear.y, ctrl_vel_.angular.z);

    // 发送控制指令
    cmd_W.targetVelocity = chair_cmd.vl;
    motor_WL->updateControl(cmd_W);
    cmd_W.targetVelocity = chair_cmd.vr;
    motor_WR->updateControl(cmd_W);
    cmd_S.targetPosition = chair_cmd.thl;
    motor_SL->updateControl(cmd_S);
    cmd_S.targetPosition = chair_cmd.thr;
    motor_SR->updateControl(cmd_S);
  }

  void publishOdometry()
  {
    auto current_time = this->now();
    // double dt = (current_time - last_time_).seconds();

    // >>>>>>>>>> 关键修改：估计电机数据的真实时间 <<<<<<<<<<
    // 轮电机更新周期: 50ms, 舵机: 33ms → 取最大值的一半作为延迟
    const double MAX_MOTOR_LATENCY_SEC = 0.050; // 50ms
    const double LATENCY_COMPENSATION_SEC = MAX_MOTOR_LATENCY_SEC / 2.0; // 25ms

    rclcpp::Time odom_time = current_time - rclcpp::Duration::from_seconds(LATENCY_COMPENSATION_SEC);

    // 防止时间倒退（首次运行）
    if (!initialized_) {
        last_odom_time_ = odom_time;
        initialized_ = true;
        return;
    }

    double dt = (odom_time - last_odom_time_).seconds();
    if (dt <= 0.0) {
        // 时间异常，跳过本次
        return;
    }


    // 获取反馈（actualVelocity/Position 应已为 SI 单位）
    double vl = motor_WL->getStatusSnapshot().actualVelocity; // 左轮
    double vr = motor_WR->getStatusSnapshot().actualVelocity; // 右轮
    double thl = motor_SL->getStatusSnapshot().actualPosition; // 左舵角
    double thr = motor_SR->getStatusSnapshot().actualPosition; // 右舵角

    // 正运动学
    auto bv = chair.computeForwardKinematics(vl, vr, thl, thr);

    double sin_th = sin(theta_);
    double cos_th = cos(theta_);

    // 积分计算位置（简单里程计模型）
    x_ += (bv.vx * cos_th - bv.vy * sin_th) * dt;
    y_ += (bv.vx * sin_th + bv.vy * cos_th) * dt;
    theta_ += (bv.omega * dt);
    // 归一化角度到 [-pi, pi]
    if (theta_ > M_PI) {
      theta_ -= 2.0 * M_PI;
    } else if (theta_ < -M_PI) {
      theta_ += 2.0 * M_PI;
    }

    // 创建 Odometry 消息
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = current_time; //修改
    // odom.header.stamp = odom_time;
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_link";

    // 位置
    odom.pose.pose.position.x = x_;
    odom.pose.pose.position.y = y_;
    odom.pose.pose.position.z = 0.0;

    // 方向（四元数）
    tf2::Quaternion q;
    q.setRPY(0, 0, theta_);
    odom.pose.pose.orientation.x = q.x();
    odom.pose.pose.orientation.y = q.y();
    odom.pose.pose.orientation.z = q.z();
    odom.pose.pose.orientation.w = q.w();

    // 速度（在 base_link 坐标系中）
    odom.twist.twist.linear.x =bv.vx;
    odom.twist.twist.linear.y =bv.vy;
    odom.twist.twist.angular.z = bv.omega;

    // 发布
    odom_pub_->publish(odom);

    // 广播 TF（可选但推荐）
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = current_time;  //修改
    // transform.header.stamp = odom_time;
    transform.header.frame_id = "odom";
    transform.child_frame_id = "base_link";
    transform.transform.translation.x = x_;
    transform.transform.translation.y = y_;
    transform.transform.translation.z = 0.0;
    transform.transform.rotation.x = q.x();
    transform.transform.rotation.y = q.y();
    transform.transform.rotation.z = q.z();
    transform.transform.rotation.w = q.w();

    tf_broadcaster_->sendTransform(transform);

    // last_time_ = current_time;
    last_odom_time_ = odom_time;
  }
  std::shared_ptr<MotorDevice> motor_WR;
  std::shared_ptr<MotorDevice> motor_WL;
  std::shared_ptr<MotorDevice> motor_SR;
  std::shared_ptr<MotorDevice> motor_SL;
  MotorDevice::ControlData cmd_W;
  MotorDevice::ControlData cmd_S;
  DualSteeredWheelchair chair;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  geometry_msgs::msg::Twist ctrl_vel_;
  double x_, y_, theta_;
  // rclcpp::Time last_time_;
  rclcpp::Time last_odom_time_;   // ✅ 替代原来的 last_time_
  bool initialized_ = false;      // ✅ 新增
};

int main(int argc, char * argv[])
{
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

    auto motor3 = std::make_shared<MotorDevice>(3, "Servo-1", BusType::CUSTOM, 2);
    motor3->config.positionUnit = 3.1415926 / 2048;
    motor3->config.updateCtrlTimeMs = 33;
    motor3->config.updateReadTimeMs = 33;

    auto motor4 = std::make_shared<MotorDevice>(4, "Servo-2", BusType::CUSTOM, 1);
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

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DSWChair>(motor1, motor2, motor3, motor4));
  rclcpp::shutdown();
  return 0;
}