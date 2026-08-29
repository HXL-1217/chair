//融合imu的里程计节点。TF 发布(odom->base_link)改为 publish_tf 参数控制：
//默认关闭（在线栈由 EKF 发 TF），raw_record 录包/纯odom实验等无 EKF 场景由 launch 显式打开。
#include "ZLAC8015MotorBus.h"
#include <iostream>
#include "UdpComm.h"
#include "DualSteeredWheelchair.h"
#include <nlohmann/json.hpp>
#include <chrono>
#include <thread>
#include <memory>
#include <cmath> // 添加 cmath 头文件以使用 M_PI, sin, cos 等

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
// [恢复] TF 广播器（publish_tf 参数为 true 时才发布 odom->base_link）
#include <tf2_ros/transform_broadcaster.h>

using namespace std::chrono_literals;
using json = nlohmann::json;

class DSWChair : public rclcpp::Node
{
public:
  DSWChair(
      std::shared_ptr<MotorDevice> wr,
      std::shared_ptr<MotorDevice> wl
    )
      : Node("odom_publisher"),
        motor_WR(wr), motor_WL(wl), chair(0.495, 0.005)
  {
    // 初始化变量
    x_ = 0.0;
    y_ = 0.0;
    theta_ = 0.0;
    initialized_ = false;

    cmd_W.controlMode = MotorDevice::Mode::VELOCITY_CONTROL;
    cmd_W.targetVelocity = 0;
    cmd_W.enable = true;

    // 订阅 cmd_vel
    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel", 10,
        std::bind(&DSWChair::cmdVelCallback, this, std::placeholders::_1));

    // 发布 odom
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);

    // 定时器：每 50ms 发布一次 odom（20Hz）
    timer_ = this->create_wall_timer(50ms,
                                     std::bind(&DSWChair::publishOdometry, this));

    // [恢复] 可选 TF 广播：publish_tf=true 时发布 odom->base_link。
    // 注意：与 EKF（ekf_filter_node 也发 odom->base_link）同时打开会互相打架，
    // 因此默认 false，仅在 raw_record 等无 EKF 场景由 launch 显式开启。
    publish_tf_ = this->declare_parameter<bool>("publish_tf", false);
    if (publish_tf_)
    {
      tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    }

    RCLCPP_INFO(this->get_logger(),
                publish_tf_ ? "DSWChair node started. TF publishing: odom->base_link (ON)"
                            : "DSWChair node started. TF broadcasting disabled (delegated to EKF).");
  }

private:
  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    // 存储最新的速度命令
    ctrl_vel_.linear.x = msg->linear.x;
    ctrl_vel_.linear.y = msg->linear.y;
    ctrl_vel_.angular.z = msg->angular.z;  // 翻转角速度方向以修正左转右转

    WheelCommands chair_cmd = chair.computeWheelCommands(ctrl_vel_.linear.x, ctrl_vel_.linear.y, ctrl_vel_.angular.z);

    // send wheel velocity commands only
    // [修改] 由于前进后退反向，这里对目标速度取反
    // 如果左右转也变反了，说明只需要改其中一个，但根据描述“左右转正常”，通常意味着差动关系正确，仅共模信号（前进）错误
    // 对于差速驱动，前进是 vl+vr，转向是 vl-vr。如果只反向前进，通常需要对两个轮子都取反
    
    cmd_W.targetVelocity = -chair_cmd.vl; // 左轮速度取反
    motor_WL->updateControl(cmd_W);
    
    cmd_W.targetVelocity = -chair_cmd.vr; // 右轮速度取反
    motor_WR->updateControl(cmd_W);
  }

  void publishOdometry()
  {
    auto current_time = this->now();

    // 估计电机数据的真实时间
    const double MAX_MOTOR_LATENCY_SEC = 0.050;                              // 50ms
    const double LATENCY_COMPENSATION_SEC = MAX_MOTOR_LATENCY_SEC / 2.0; // 25ms
    rclcpp::Time odom_time = current_time - rclcpp::Duration::from_seconds(LATENCY_COMPENSATION_SEC);

    if (!initialized_)
    {
      last_odom_time_ = odom_time;
      initialized_ = true;
      return;
    }

    double dt = (odom_time - last_odom_time_).seconds();
    if (dt <= 0.0)
    {
      return;
    }

    // 获取反馈
    // [修改] 读取到的实际速度也需要取反，以匹配我们发送命令时的取反操作，保证里程计计算的方向与实际物理运动一致
    vl_ = -motor_WL->getStatusSnapshot().actualVelocity;  // 左轮反馈取反
    vr_ = -motor_WR->getStatusSnapshot().actualVelocity;  // 右轮反馈取反
    
    double thl = 0.0;
    double thr = 0.0;

    // 正运动学
    auto bv = chair.computeForwardKinematics(vl_, vr_, thl, thr);

    // [修复·左右转向反] 库的 omega 本就是逆时针为正（computeWheelCommands 与之互为逆运算，
    // 控制方向已验证正确）。旧代码此处多取了一次负号（旧版在 cmd_vel 输入端取反的配对遗留，
    // 输入端取反删除后此补偿就成了 bug）：导致 theta_ 积分方向与实际转向相反——
    // 前进正常、转弯时 odom 位姿左右镜像。现直接用 bv.omega，与 twist.angular.z 同号。
    double sin_th = sin(theta_);
    double cos_th = cos(theta_);

    // 积分计算位置
    // 假设 bv.vx 是车身坐标系下的前向速度，bv.vy 是侧向速度
    x_ += (bv.vx * cos_th - bv.vy * sin_th) * dt;
    y_ += (bv.vx * sin_th + bv.vy * cos_th) * dt;
    theta_ += (bv.omega * dt);

    // 角度归一化到 -PI ~ PI
    if (theta_ > M_PI) { theta_ -= 2.0 * M_PI; }
    else if (theta_ < -M_PI) { theta_ += 2.0 * M_PI; }

    // 创建 Odometry 消息
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = current_time; 
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

    // 速度
    // 这里的 bv 是基于取反后的 vl_/vr_ 计算的，所以方向应该是正确的
    odom.twist.twist.linear.x = bv.vx;
    odom.twist.twist.linear.y = bv.vy;
    odom.twist.twist.angular.z = bv.omega; // 注意：此处保留原始计算值，或按需使用 corrected_omega

    // >>>>>>>>>> 新增：配置协方差矩阵 (Covariance) <<<<<<<<<<
    // 对角线元素代表方差。数值越小，代表你认为该项数据越精准。
    // 这将极大帮助 EKF 算法在轮子和 IMU 之间分配权重。
    
    // Pose 协方差 (x, y, z, roll, pitch, yaw)
    odom.pose.covariance[0]  = 0.001; // x 稍自信
    odom.pose.covariance[7]  = 0.001; // y 稍自信
    odom.pose.covariance[14] = 1e6;   // z 无效，设为极大值
    odom.pose.covariance[21] = 1e6;   // roll 无效
    odom.pose.covariance[28] = 1e6;   // pitch 无效
    odom.pose.covariance[35] = 0.01;  // yaw (航向角由于轮子易打滑，方差设大一点，让 EKF 更信任 IMU)

    // Twist 协方差 (vx, vy, vz, vroll, vpitch, vyaw)
    odom.twist.covariance[0]  = 0.001; // vx 轮子测速很准
    odom.twist.covariance[7]  = 0.001; // vy 轮子侧滑测速很准
    odom.twist.covariance[14] = 1e6;   // vz 无效
    odom.twist.covariance[21] = 1e6;   // vroll 无效
    odom.twist.covariance[28] = 1e6;   // vpitch 无效
    odom.twist.covariance[35] = 0.01;  // vyaw 角速度同样让 EKF 更偏向 IMU 的陀螺仪

    // 发布里程计信息给 EKF
    odom_pub_->publish(odom);

    // [恢复] 参数开启时广播 odom->base_link TF（位姿与 /odom 消息一致）
    if (publish_tf_ && tf_broadcaster_)
    {
      geometry_msgs::msg::TransformStamped transform;
      transform.header.stamp = current_time;
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
    }

    last_odom_time_ = odom_time;
  }

  std::shared_ptr<MotorDevice> motor_WR;
  std::shared_ptr<MotorDevice> motor_WL;
  MotorDevice::ControlData cmd_W;
  DualSteeredWheelchair chair;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  // [恢复] publish_tf 参数控制的 TF 广播器
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  bool publish_tf_ = false;

  geometry_msgs::msg::Twist ctrl_vel_;
  double x_, y_, theta_;
  double vl_ = 0, vr_ = 0;
  rclcpp::Time last_odom_time_; 
  bool initialized_ = false;    
};

int main(int argc, char *argv[])
{
  auto bus = std::make_shared<ZLAC8015MotorBus>("/dev/ttyUSB0", 115200, 'N', 8, 1);

  if (!bus->init())
  {
    std::cerr << "总线初始化失败!" << std::endl;
    return -1;
  }

  auto motor1 = std::make_shared<MotorDevice>(1, "Wheel-R", BusType::MODBUS_RTU, (1 << 8) | 0);
  motor1->config.velocityUnit = 9.0582588e-3;
  motor1->config.updateCtrlTimeMs = 50;
  motor1->config.updateReadTimeMs = 50;

  auto motor2 = std::make_shared<MotorDevice>(2, "Wheel-L", BusType::MODBUS_RTU, (1 << 8) | 1);
  motor2->config.velocityUnit = -9.0582588e-3;
  motor2->config.updateCtrlTimeMs = 50;
  motor2->config.updateReadTimeMs = 50;

  bus->addMotor(motor1);
  bus->addMotor(motor2);
  std::cout << "addMotor" << std::endl;

  MotorDevice::ControlData cmd;
  cmd.controlMode = MotorDevice::Mode::VELOCITY_CONTROL;
  cmd.targetVelocity = 0;
  cmd.enable = true;
  motor1->updateControl(cmd);
  motor2->updateControl(cmd);

  bus->start();
  std::cout << "start" << std::endl;

  rclcpp::init(argc, argv);
  auto node = std::make_shared<DSWChair>(motor1, motor2);
  
  rclcpp::spin(node);

  cmd.targetVelocity = 0.0;
  cmd.enable = false; 
  motor1->updateControl(cmd);
  motor2->updateControl(cmd);
  
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  bus->stop(); 
  rclcpp::shutdown();
  
  std::cout << "[System] 节点已退出。" << std::endl;
  return 0;
}



// //融合imu的里程计节点，删除了TF发布功能，改为直接发布里程计消息给robot_localization的EKF节点进行融合
// #include "ZLAC8015MotorBus.h"
// #include <iostream>
// #include "UdpComm.h"
// #include "DualSteeredWheelchair.h"
// #include <nlohmann/json.hpp>
// #include <chrono>
// #include <thread>
// #include <memory>

// #include <rclcpp/rclcpp.hpp>
// #include <geometry_msgs/msg/twist.hpp>
// #include <nav_msgs/msg/odometry.hpp>
// #include <tf2/LinearMath/Quaternion.h>
// // [删除] 不再需要 TF 广播器头文件
// // #include <tf2_ros/transform_broadcaster.h>

// using namespace std::chrono_literals;
// using json = nlohmann::json;

// class DSWChair : public rclcpp::Node
// {
// public:
//   DSWChair(
//       std::shared_ptr<MotorDevice> wr,
//       std::shared_ptr<MotorDevice> wl
//     )
//       : Node("odom_publisher"),
//         motor_WR(wr), motor_WL(wl), chair(0.495, 0.005)
//   {
//     // 初始化变量
//     x_ = 0.0;
//     y_ = 0.0;
//     theta_ = 0.0;
//     initialized_ = false;

//     cmd_W.controlMode = MotorDevice::Mode::VELOCITY_CONTROL;
//     cmd_W.targetVelocity = 0;
//     cmd_W.enable = true;

//     // 订阅 cmd_vel
//     cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
//         "/cmd_vel", 10,
//         std::bind(&DSWChair::cmdVelCallback, this, std::placeholders::_1));

//     // 发布 odom
//     odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);

//     // 定时器：每 50ms 发布一次 odom（20Hz）
//     timer_ = this->create_wall_timer(50ms,
//                                      std::bind(&DSWChair::publishOdometry, this));

//     // [削权] 删除 TF 广播器的初始化代码，把发 TF 的任务交给 robot_localization
//     // tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

//     RCLCPP_INFO(this->get_logger(), "DSWChair node started. TF broadcasting disabled (delegated to EKF).");
//   }

// private:
//   void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
//   {
//     // 存储最新的速度命令
//     ctrl_vel_.linear.x = msg->linear.x;
//     ctrl_vel_.linear.y = msg->linear.y;
//     ctrl_vel_.angular.z = -msg->angular.z;  // 翻转角速度方向以修正左转右转

//     WheelCommands chair_cmd = chair.computeWheelCommands(ctrl_vel_.linear.x, ctrl_vel_.linear.y, ctrl_vel_.angular.z);

//     // send wheel velocity commands only
//     cmd_W.targetVelocity = chair_cmd.vl;
//     motor_WL->updateControl(cmd_W);
//     cmd_W.targetVelocity = chair_cmd.vr;
//     motor_WR->updateControl(cmd_W);
//   }

//   void publishOdometry()
//   {
//     auto current_time = this->now();

//     // 估计电机数据的真实时间
//     const double MAX_MOTOR_LATENCY_SEC = 0.050;                              // 50ms
//     const double LATENCY_COMPENSATION_SEC = MAX_MOTOR_LATENCY_SEC / 2.0; // 25ms
//     rclcpp::Time odom_time = current_time - rclcpp::Duration::from_seconds(LATENCY_COMPENSATION_SEC);

//     if (!initialized_)
//     {
//       last_odom_time_ = odom_time;
//       initialized_ = true;
//       return;
//     }

//     double dt = (odom_time - last_odom_time_).seconds();
//     if (dt <= 0.0)
//     {
//       return;
//     }

//     // 获取反馈
//     vl_ = motor_WL->getStatusSnapshot().actualVelocity;  // 左轮
//     vr_ = motor_WR->getStatusSnapshot().actualVelocity;  // 右轮
//     double thl = 0.0;
//     double thr = 0.0;

//     // 正运动学
//     auto bv = chair.computeForwardKinematics(vl_, vr_, thl, thr);
//     double corrected_omega = -bv.omega;
//     double sin_th = sin(theta_);
//     double cos_th = cos(theta_);

//     // 积分计算位置
//     x_ += (bv.vx * cos_th - bv.vy * sin_th) * dt;
//     y_ += (bv.vx * sin_th + bv.vy * cos_th) * dt;
//     theta_ += (corrected_omega * dt);
    
//     if (theta_ > M_PI) { theta_ -= 2.0 * M_PI; }
//     else if (theta_ < -M_PI) { theta_ += 2.0 * M_PI; }

//     // 创建 Odometry 消息
//     nav_msgs::msg::Odometry odom;
//     odom.header.stamp = current_time; 
//     odom.header.frame_id = "odom";
//     odom.child_frame_id = "base_link";

//     // 位置
//     odom.pose.pose.position.x = x_;
//     odom.pose.pose.position.y = y_;
//     odom.pose.pose.position.z = 0.0;

//     // 方向（四元数）
//     tf2::Quaternion q;
//     q.setRPY(0, 0, theta_);
//     odom.pose.pose.orientation.x = q.x();
//     odom.pose.pose.orientation.y = q.y();
//     odom.pose.pose.orientation.z = q.z();
//     odom.pose.pose.orientation.w = q.w();

//     // 速度
//     odom.twist.twist.linear.x = bv.vx;
//     odom.twist.twist.linear.y = bv.vy;
//     odom.twist.twist.angular.z = bv.omega; // 注意：此处保留原始计算值，或按需使用 corrected_omega

//     // >>>>>>>>>> 新增：配置协方差矩阵 (Covariance) <<<<<<<<<<
//     // 对角线元素代表方差。数值越小，代表你认为该项数据越精准。
//     // 这将极大帮助 EKF 算法在轮子和 IMU 之间分配权重。
    
//     // Pose 协方差 (x, y, z, roll, pitch, yaw)
//     odom.pose.covariance[0]  = 0.001; // x 稍自信
//     odom.pose.covariance[7]  = 0.001; // y 稍自信
//     odom.pose.covariance[14] = 1e6;   // z 无效，设为极大值
//     odom.pose.covariance[21] = 1e6;   // roll 无效
//     odom.pose.covariance[28] = 1e6;   // pitch 无效
//     odom.pose.covariance[35] = 0.01;  // yaw (航向角由于轮子易打滑，方差设大一点，让 EKF 更信任 IMU)

//     // Twist 协方差 (vx, vy, vz, vroll, vpitch, vyaw)
//     odom.twist.covariance[0]  = 0.001; // vx 轮子测速很准
//     odom.twist.covariance[7]  = 0.001; // vy 轮子侧滑测速很准
//     odom.twist.covariance[14] = 1e6;   // vz 无效
//     odom.twist.covariance[21] = 1e6;   // vroll 无效
//     odom.twist.covariance[28] = 1e6;   // vpitch 无效
//     odom.twist.covariance[35] = 0.01;  // vyaw 角速度同样让 EKF 更偏向 IMU 的陀螺仪

//     // 发布里程计信息给 EKF
//     odom_pub_->publish(odom);

//     // [削权] 彻底删除了 TF 发布相关的代码 (geometry_msgs::msg::TransformStamped ... tf_broadcaster_->sendTransform)

//     last_odom_time_ = odom_time;
//   }

//   std::shared_ptr<MotorDevice> motor_WR;
//   std::shared_ptr<MotorDevice> motor_WL;
//   MotorDevice::ControlData cmd_W;
//   DualSteeredWheelchair chair;

//   rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
//   rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
//   rclcpp::TimerBase::SharedPtr timer_;
//   // [删除] std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

//   geometry_msgs::msg::Twist ctrl_vel_;
//   double x_, y_, theta_;
//   double vl_ = 0, vr_ = 0;
//   rclcpp::Time last_odom_time_; 
//   bool initialized_ = false;    
// };

// int main(int argc, char *argv[])
// {
//   auto bus = std::make_shared<ZLAC8015MotorBus>("/dev/ttyUSB0", 115200, 'N', 8, 1);

//   if (!bus->init())
//   {
//     std::cerr << "总线初始化失败!" << std::endl;
//     return -1;
//   }

//   auto motor1 = std::make_shared<MotorDevice>(1, "Wheel-R", BusType::MODBUS_RTU, (1 << 8) | 0);
//   motor1->config.velocityUnit = 9.0582588e-3;
//   motor1->config.updateCtrlTimeMs = 50;
//   motor1->config.updateReadTimeMs = 50;

//   auto motor2 = std::make_shared<MotorDevice>(2, "Wheel-L", BusType::MODBUS_RTU, (1 << 8) | 1);
//   motor2->config.velocityUnit = -9.0582588e-3;
//   motor2->config.updateCtrlTimeMs = 50;
//   motor2->config.updateReadTimeMs = 50;

//   bus->addMotor(motor1);
//   bus->addMotor(motor2);
//   std::cout << "addMotor" << std::endl;

//   MotorDevice::ControlData cmd;
//   cmd.controlMode = MotorDevice::Mode::VELOCITY_CONTROL;
//   cmd.targetVelocity = 0;
//   cmd.enable = true;
//   motor1->updateControl(cmd);
//   motor2->updateControl(cmd);

//   bus->start();
//   std::cout << "start" << std::endl;

//   rclcpp::init(argc, argv);
//   auto node = std::make_shared<DSWChair>(motor1, motor2);
  
//   rclcpp::spin(node);

//   cmd.targetVelocity = 0.0;
//   cmd.enable = false; 
//   motor1->updateControl(cmd);
//   motor2->updateControl(cmd);
  
//   std::this_thread::sleep_for(std::chrono::milliseconds(100));

//   bus->stop(); 
//   rclcpp::shutdown();
  
//   std::cout << "[System] 节点已退出。" << std::endl;
//   return 0;
// }

// //只有驱动轮版本（去掉舵机相关代码）

// #include "ZLAC8015MotorBus.h"
// // #include "SCServoMotorBus.h"  // servo bus not needed when only driving two wheels
// #include <iostream>
// #include "UdpComm.h"
// #include "DualSteeredWheelchair.h"
// #include <nlohmann/json.hpp>
// #include <chrono>
// #include <thread>
// #include <memory>

// #include <rclcpp/rclcpp.hpp>
// #include <geometry_msgs/msg/twist.hpp>
// #include <nav_msgs/msg/odometry.hpp>
// #include <tf2/LinearMath/Quaternion.h>
// #include <tf2_ros/transform_broadcaster.h>

// using namespace std::chrono_literals;
// using json = nlohmann::json;

// class DSWChair : public rclcpp::Node
// {
// public:
//   DSWChair(
//       std::shared_ptr<MotorDevice> wr,
//       std::shared_ptr<MotorDevice> wl
//       // std::shared_ptr<MotorDevice> sr,
//       // std::shared_ptr<MotorDevice> sl
//     )
//       : Node("odom_publisher"),
//         motor_WR(wr), motor_WL(wl), chair(0.5, 0.005)
//   {
//     // 初始化变量
//     x_ = 0.0;
//     y_ = 0.0;
//     theta_ = 0.0;
//     // last_time_ = this->now();
//     initialized_ = false;

//     cmd_W.controlMode = MotorDevice::Mode::VELOCITY_CONTROL;
//     cmd_W.targetVelocity = 0;
//     cmd_W.enable = true;
//     // servo control removed – only wheel commands used


//     // 订阅 cmd_vel
//     cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
//         "/cmd_vel", 10,
//         std::bind(&DSWChair::cmdVelCallback, this, std::placeholders::_1));

//     // 发布 odom
//     odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);

//     // 定时器：每 50ms 发布一次 odom（20Hz）
//     timer_ = this->create_wall_timer(50ms,
//                                      std::bind(&DSWChair::publishOdometry, this));

//     // TF 广播器（可选，用于发布 odom -> base_link 的变换）
//     tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

//     RCLCPP_INFO(this->get_logger(), "DSWChair node started.");
//   }

// private:
//   void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
//   {
//     // 存储最新的速度命令
//     ctrl_vel_.linear.x = msg->linear.x;
//     ctrl_vel_.linear.y = msg->linear.y;
//     ctrl_vel_.angular.z = -msg->angular.z;  // 翻转角速度方向以修正左转右转

//     WheelCommands chair_cmd = chair.computeWheelCommands(ctrl_vel_.linear.x, ctrl_vel_.linear.y, ctrl_vel_.angular.z);

//     // send wheel velocity commands only
//     cmd_W.targetVelocity = chair_cmd.vl;
//     motor_WL->updateControl(cmd_W);
//     cmd_W.targetVelocity = chair_cmd.vr;
//     motor_WR->updateControl(cmd_W);
//   }

//   void publishOdometry()
//   {
//     auto current_time = this->now();
//     // double dt = (current_time - last_time_).seconds();

//     // >>>>>>>>>> 关键修改：估计电机数据的真实时间 <<<<<<<<<<
//     // 轮电机更新周期: 50ms, 舵机: 33ms → 取最大值的一半作为延迟
//     const double MAX_MOTOR_LATENCY_SEC = 0.050;                          // 50ms
//     const double LATENCY_COMPENSATION_SEC = MAX_MOTOR_LATENCY_SEC / 2.0; // 25ms

//     rclcpp::Time odom_time = current_time - rclcpp::Duration::from_seconds(LATENCY_COMPENSATION_SEC);

//     // 防止时间倒退（首次运行）
//     if (!initialized_)
//     {
//       last_odom_time_ = odom_time;
//       initialized_ = true;
//       return;
//     }

//     double dt = (odom_time - last_odom_time_).seconds();
//     if (dt <= 0.0)
//     {
//       // 时间异常，跳过本次
//       return;
//     }

//     // 获取反馈（actualVelocity 应已为 SI 单位）
//     vl_ = motor_WL->getStatusSnapshot().actualVelocity;  // 左轮
//     vr_ = motor_WR->getStatusSnapshot().actualVelocity;  // 右轮
//     double thl = 0.0; // steering ignored
//     double thr = 0.0;

//     // 正运动学
//     auto bv = chair.computeForwardKinematics(vl_, vr_, thl, thr);
//     double corrected_omega = -bv.omega; // 翻转角速度方向以修正左转右转
//     double sin_th = sin(theta_);
//     double cos_th = cos(theta_);

//     // 积分计算位置（简单里程计模型）
//     x_ += (bv.vx * cos_th - bv.vy * sin_th) * dt;
//     y_ += (bv.vx * sin_th + bv.vy * cos_th) * dt;
//     // theta_ += (bv.omega * dt);
//     theta_ += (corrected_omega * dt); // 使用修正后的角速度积分
//     // 归一化角度到 [-pi, pi]
//     if (theta_ > M_PI)
//     {
//       theta_ -= 2.0 * M_PI;
//     }
//     else if (theta_ < -M_PI)
//     {
//       theta_ += 2.0 * M_PI;
//     }

//     // 创建 Odometry 消息
//     nav_msgs::msg::Odometry odom;
//     odom.header.stamp = current_time; // 修改
//     // odom.header.stamp = odom_time;
//     odom.header.frame_id = "odom";
//     odom.child_frame_id = "base_link";

//     // 位置
//     odom.pose.pose.position.x = x_;
//     odom.pose.pose.position.y = y_;
//     odom.pose.pose.position.z = 0.0;

//     // 方向（四元数）
//     tf2::Quaternion q;
//     q.setRPY(0, 0, theta_);
//     odom.pose.pose.orientation.x = q.x();
//     odom.pose.pose.orientation.y = q.y();
//     odom.pose.pose.orientation.z = q.z();
//     odom.pose.pose.orientation.w = q.w();

//     // 速度（在 base_link 坐标系中）
//     odom.twist.twist.linear.x = bv.vx;
//     odom.twist.twist.linear.y = bv.vy;
//     odom.twist.twist.angular.z = bv.omega;

//     // 发布
//     odom_pub_->publish(odom);

//     // 广播 TF（可选但推荐）
//     geometry_msgs::msg::TransformStamped transform;
//     transform.header.stamp = current_time; // 修改
//     // transform.header.stamp = odom_time;
//     transform.header.frame_id = "odom";
//     transform.child_frame_id = "base_link";
//     transform.transform.translation.x = x_;
//     transform.transform.translation.y = y_;
//     transform.transform.translation.z = 0.0;
//     transform.transform.rotation.x = q.x();
//     transform.transform.rotation.y = q.y();
//     transform.transform.rotation.z = q.z();
//     transform.transform.rotation.w = q.w();

//     tf_broadcaster_->sendTransform(transform);

//     // last_time_ = current_time;
//     last_odom_time_ = odom_time;
//   }
//   std::shared_ptr<MotorDevice> motor_WR;
//   std::shared_ptr<MotorDevice> motor_WL;
//   MotorDevice::ControlData cmd_W;
//   DualSteeredWheelchair chair;

//   rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
//   rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
//   rclcpp::TimerBase::SharedPtr timer_;
//   std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

//   geometry_msgs::msg::Twist ctrl_vel_;
//   double x_, y_, theta_;
//   double vl_ = 0, vr_ = 0;
//   // rclcpp::Time last_time_;
//   rclcpp::Time last_odom_time_; // 替代原来的 last_time_
//   bool initialized_ = false;    // 新增
// };

// int main(int argc, char *argv[])
// {
//   auto bus = std::make_shared<ZLAC8015MotorBus>("/dev/ttyUSB0", 115200, 'N', 8, 1);

//   if (!bus->init())
//   {
//     std::cerr << "总线初始化失败!" << std::endl;
//     return -1;
//   }

//   auto motor1 = std::make_shared<MotorDevice>(1, "Wheel-R", BusType::MODBUS_RTU, (1 << 8) | 0);
//   motor1->config.velocityUnit = 9.0582588e-3;
//   motor1->config.updateCtrlTimeMs = 50;
//   motor1->config.updateReadTimeMs = 50;

//   auto motor2 = std::make_shared<MotorDevice>(2, "Wheel-L", BusType::MODBUS_RTU, (1 << 8) | 1);
//   motor2->config.velocityUnit = -9.0582588e-3;
//   motor2->config.updateCtrlTimeMs = 50;
//   motor2->config.updateReadTimeMs = 50;

//   bus->addMotor(motor1);
//   bus->addMotor(motor2);
//   std::cout << "addMotor" << std::endl;

//   MotorDevice::ControlData cmd;
//   cmd.controlMode = MotorDevice::Mode::VELOCITY_CONTROL;
//   cmd.targetVelocity = 0;
//   cmd.enable = true;
//   motor1->updateControl(cmd);
//   motor2->updateControl(cmd);

//   bus->start();
//   std::cout << "start" << std::endl;

//   rclcpp::init(argc, argv);
//   auto node = std::make_shared<DSWChair>(motor1, motor2);
  
//   rclcpp::spin(node);

//   // 1. 让轮椅安全停止：下发 0 速度，并断开使能
//   cmd.targetVelocity = 0.0;
//   cmd.enable = false; // 让电机卸载力度（视驱动器支持情况）
//   motor1->updateControl(cmd);
//   motor2->updateControl(cmd);
  
//   std::this_thread::sleep_for(std::chrono::milliseconds(100));

//   // 2. 停止总线后台线程
//   bus->stop(); 
//   rclcpp::shutdown();
  
//   std::cout << "[System] 节点已退出。" << std::endl;
//   return 0;
// }

// //舵机+轮子版本（保留舵机相关代码，实际不下发舵机命令）

// #include "ZLAC8015MotorBus.h"
// #include "SCServoMotorBus.h"
// #include <iostream>
// #include "UdpComm.h"
// #include "DualSteeredWheelchair.h"
// #include <nlohmann/json.hpp>
// #include <chrono>
// #include <thread>
// #include <memory>
// #include <mutex>
// #include <cmath>

// #include <rclcpp/rclcpp.hpp>
// #include <geometry_msgs/msg/twist.hpp>
// #include <nav_msgs/msg/odometry.hpp>
// #include <geometry_msgs/msg/transform_stamped.hpp>
// #include <tf2/LinearMath/Quaternion.h>
// #include <tf2_ros/transform_broadcaster.h>

// using namespace std::chrono_literals;
// using json = nlohmann::json;

// class DSWChair : public rclcpp::Node
// {
// public:
//   DSWChair(
//       std::shared_ptr<MotorDevice> wr,
//       std::shared_ptr<MotorDevice> wl,
//       std::shared_ptr<MotorDevice> sr,
//       std::shared_ptr<MotorDevice> sl)
//       : Node("odom_publisher"),
//         motor_WR(wr), motor_WL(wl), motor_SR(sr), motor_SL(sl),
//         chair(0.55, 0.005)
//   {
    
//     max_lin_accel_ = 0.4;   // m/s^2 线加速度（vx/vy）
//     max_ang_accel_ = 0.60;   // rad/s^2 角加速度（wz）
//     cmd_timeout_   = 0.30;   // s 超过这个时间没收到 /cmd_vel 就平滑减速到 0

//     // -------------------------
//     // 状态初始化
//     // -------------------------
//     x_ = 0.0;
//     y_ = 0.0;
//     theta_ = 0.0;
//     initialized_ = false;

//     target_vel_ = geometry_msgs::msg::Twist();
//     smooth_vel_ = geometry_msgs::msg::Twist();
//     last_ctrl_time_ = this->now();
//     last_cmd_time_  = this->now();

//     cmd_W.controlMode = MotorDevice::Mode::VELOCITY_CONTROL;
//     cmd_W.targetVelocity = 0;
//     cmd_W.enable = true;

//     cmd_S.controlMode = MotorDevice::Mode::POSITION_CONTROL;
//     cmd_S.targetPosition = 0;
//     cmd_S.enable = true;

//     // 订阅 cmd_vel（只保存目标速度，不直接下发电机）
//     cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
//         "/cmd_vel", 10,
//         std::bind(&DSWChair::cmdVelCallback, this, std::placeholders::_1));

//     // 发布 odom
//     odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);

//     // 里程计定时器：20Hz
//     odom_timer_ = this->create_wall_timer(
//       50ms, std::bind(&DSWChair::publishOdometry, this));

//     // 控制定时器：20Hz（平滑加速 + 下发电机）
//     control_timer_ = this->create_wall_timer(
//       50ms, std::bind(&DSWChair::controlLoop, this));

//     // TF 广播器
//     tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

//     chair.resetSteeringAngles(
//       motor_SL->getStatusSnapshot().actualPosition,
//       motor_SR->getStatusSnapshot().actualPosition
//     );

//     RCLCPP_INFO(this->get_logger(), "DSWChair node started (with accel ramp).");
//   }

// private:
//   // -------------------------
//   // /cmd_vel 回调：只更新目标速度
//   // -------------------------
//   void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
//   {
//     std::lock_guard<std::mutex> lk(vel_mtx_);
//     target_vel_ = *msg;
//     last_cmd_time_ = this->now();
//   }

//   // 每次最多变化 max_delta（slew rate limit）
//   static double slew(double current, double target, double max_delta)
//   {
//     double diff = target - current;
//     if (diff >  max_delta) diff =  max_delta;
//     if (diff < -max_delta) diff = -max_delta;
//     return current + diff;
//   }

//   // -------------------------
//   // 控制循环：平滑速度 + 下发轮/舵控制
//   // -------------------------
//   void controlLoop()
//   {
//     auto now = this->now();
//     double dt = (now - last_ctrl_time_).seconds();
//     if (dt <= 0.0) return;
//     last_ctrl_time_ = now;

//     geometry_msgs::msg::Twist target;
//     {
//       std::lock_guard<std::mutex> lk(vel_mtx_);
//       target = target_vel_;
//       // 超时：平滑减速到 0（安全）
//       if ((now - last_cmd_time_).seconds() > cmd_timeout_) {
//         target = geometry_msgs::msg::Twist();
//       }
//     }

//     // 本周期允许的最大变化量
//     const double max_dv = max_lin_accel_ * dt; // m/s per tick
//     const double max_dw = max_ang_accel_ * dt; // rad/s per tick

//     // 读取当前舵角
//     const double thl_now = motor_SL->getStatusSnapshot().actualPosition;
//     const double thr_now = motor_SR->getStatusSnapshot().actualPosition;

//     // 计算基于当前 smooth_vel_ 的临时 chair_cmd，用于检测舵角变化
//     WheelCommands chair_cmd_temp =
//       chair.computeWheelCommands(smooth_vel_.linear.x, smooth_vel_.linear.y, smooth_vel_.angular.z);

//     // 检查是否需要舵机变化保护
//     if (std::abs(chair_cmd_temp.thl - thl_now) > 0.52333 || std::abs(chair_cmd_temp.thr - thr_now) > 0.52333)
//     {
//       steering_changing_ = true;
//       temp_target_ = geometry_msgs::msg::Twist();  // 目标速度设为0
//     }
//     else
//     {
//       steering_changing_ = false;
//       temp_target_ = target;  // 恢复正常目标速度
//     }

//     // 平滑后的输出速度（向 temp_target_ 平滑）
//     smooth_vel_.linear.x  = slew(smooth_vel_.linear.x,  temp_target_.linear.x,  max_dv);
//     smooth_vel_.linear.y  = slew(smooth_vel_.linear.y,  temp_target_.linear.y,  max_dv);
//     smooth_vel_.angular.z = slew(smooth_vel_.angular.z, temp_target_.angular.z, max_dw);

//     // 用平滑速度计算轮/舵指令
//     WheelCommands chair_cmd =
//       chair.computeWheelCommands(smooth_vel_.linear.x, smooth_vel_.linear.y, smooth_vel_.angular.z);

//     // 下发舵机（位置）
//     cmd_S.targetPosition = chair_cmd.thl;
//     motor_SL->updateControl(cmd_S);
//     cmd_S.targetPosition = chair_cmd.thr;
//     motor_SR->updateControl(cmd_S);

//     // 下发轮子（速度）
//     cmd_W.targetVelocity = chair_cmd.vl;
//     motor_WL->updateControl(cmd_W);
//     cmd_W.targetVelocity = chair_cmd.vr;
//     motor_WR->updateControl(cmd_W);
//   }

//   // -------------------------
//   // 里程计发布（你原逻辑基本不动）
//   // -------------------------
//   void publishOdometry()
//   {
//     auto current_time = this->now();

//     // 估计电机数据的真实时间（你原来的延迟补偿）
//     const double MAX_MOTOR_LATENCY_SEC = 0.050;
//     const double LATENCY_COMPENSATION_SEC = MAX_MOTOR_LATENCY_SEC / 2.0; // 25ms
//     rclcpp::Time odom_time = current_time - rclcpp::Duration::from_seconds(LATENCY_COMPENSATION_SEC);

//     if (!initialized_)
//     {
//       last_odom_time_ = odom_time;
//       initialized_ = true;
//       return;
//     }

//     double dt = (odom_time - last_odom_time_).seconds();
//     if (dt <= 0.0) return;

//     // 读取反馈（假设 actualVelocity/actualPosition 已经是 SI 单位）
//     const double vl = motor_WL->getStatusSnapshot().actualVelocity;
//     const double vr = motor_WR->getStatusSnapshot().actualVelocity;
//     const double thl = motor_SL->getStatusSnapshot().actualPosition;
//     const double thr = motor_SR->getStatusSnapshot().actualPosition;

//     // 正运动学
//     auto bv = chair.computeForwardKinematics(vl, vr, thl, thr);

//     const double sin_th = std::sin(theta_);
//     const double cos_th = std::cos(theta_);

//     // 积分
//     x_     += (bv.vx * cos_th - bv.vy * sin_th) * dt;
//     y_     += (bv.vx * sin_th + bv.vy * cos_th) * dt;
//     theta_ += (bv.omega * dt);

//     // 角度归一化
//     if (theta_ > M_PI)       theta_ -= 2.0 * M_PI;
//     else if (theta_ < -M_PI) theta_ += 2.0 * M_PI;

//     // Odometry
//     nav_msgs::msg::Odometry odom;
//     odom.header.stamp = current_time;  // 你原来选 current_time（保持）
//     // odom.header.stamp = odom_time;   // 若需要更严格时序，可用 odom_time
//     odom.header.frame_id = "odom";
//     odom.child_frame_id  = "base_link";

//     odom.pose.pose.position.x = x_;
//     odom.pose.pose.position.y = y_;
//     odom.pose.pose.position.z = 0.0;

//     tf2::Quaternion q;
//     q.setRPY(0, 0, theta_);
//     odom.pose.pose.orientation.x = q.x();
//     odom.pose.pose.orientation.y = q.y();
//     odom.pose.pose.orientation.z = q.z();
//     odom.pose.pose.orientation.w = q.w();

//     odom.twist.twist.linear.x  = bv.vx;
//     odom.twist.twist.linear.y  = bv.vy;
//     odom.twist.twist.angular.z = bv.omega;

//     odom_pub_->publish(odom);

//     // TF
//     geometry_msgs::msg::TransformStamped transform;
//     transform.header.stamp = current_time;
//     // transform.header.stamp = odom_time;
//     transform.header.frame_id = "odom";
//     transform.child_frame_id  = "base_link";
//     transform.transform.translation.x = x_;
//     transform.transform.translation.y = y_;
//     transform.transform.translation.z = 0.0;
//     transform.transform.rotation.x = q.x();
//     transform.transform.rotation.y = q.y();
//     transform.transform.rotation.z = q.z();
//     transform.transform.rotation.w = q.w();

//     tf_broadcaster_->sendTransform(transform);

//     last_odom_time_ = odom_time;
//   }

// private:
//   // motors
//   std::shared_ptr<MotorDevice> motor_WR;
//   std::shared_ptr<MotorDevice> motor_WL;
//   std::shared_ptr<MotorDevice> motor_SR;
//   std::shared_ptr<MotorDevice> motor_SL;

//   MotorDevice::ControlData cmd_W;
//   MotorDevice::ControlData cmd_S;
//   DualSteeredWheelchair chair;

//   // ros
//   rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
//   rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
//   rclcpp::TimerBase::SharedPtr odom_timer_;
//   rclcpp::TimerBase::SharedPtr control_timer_;
//   std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

//   // odom state
//   double x_{0.0}, y_{0.0}, theta_{0.0};
//   rclcpp::Time last_odom_time_;
//   bool initialized_{false};

//   // accel ramp state
//   std::mutex vel_mtx_;
//   geometry_msgs::msg::Twist target_vel_;
//   geometry_msgs::msg::Twist smooth_vel_;
//   rclcpp::Time last_ctrl_time_;
//   rclcpp::Time last_cmd_time_;
//   double max_lin_accel_{0.25};
//   double max_ang_accel_{0.60};
//   double cmd_timeout_{0.30};

//   // steering change state
//   bool steering_changing_{false};
//   geometry_msgs::msg::Twist temp_target_;
// };

// int main(int argc, char *argv[])
// {
//   auto bus  = std::make_shared<ZLAC8015MotorBus>("/dev/ttyUSB1", 115200, 'N', 8, 1);
//   auto bus2 = std::make_shared<SCServoMotorBus>("/dev/ttyUSB0", 500000, 'N', 8, 1);

//   if (!bus->init() || !bus2->init())
//   {
//     std::cerr << "总线初始化失败!" << std::endl;
//     return -1;
//   }

//   auto motor1 = std::make_shared<MotorDevice>(1, "Wheel-R", BusType::MODBUS_RTU, (1 << 8) | 0);
//   motor1->config.velocityUnit = 7.27802286e-3;
//   motor1->config.updateCtrlTimeMs = 50;
//   motor1->config.updateReadTimeMs = 50;

//   auto motor2 = std::make_shared<MotorDevice>(2, "Wheel-L", BusType::MODBUS_RTU, (1 << 8) | 1);
//   motor2->config.velocityUnit = -7.27802286e-3;
//   motor2->config.updateCtrlTimeMs = 50;
//   motor2->config.updateReadTimeMs = 50;

//   auto motor3 = std::make_shared<MotorDevice>(3, "Servo-1", BusType::CUSTOM, 2);
//   motor3->config.positionUnit = 3.1415926 / 2048;
//   motor3->config.updateCtrlTimeMs = 33;
//   motor3->config.updateReadTimeMs = 33;

//   auto motor4 = std::make_shared<MotorDevice>(4, "Servo-2", BusType::CUSTOM, 1);
//   motor4->config.positionUnit = 3.1415926 / 2048;
//   motor4->config.updateCtrlTimeMs = 33;
//   motor4->config.updateReadTimeMs = 33;

//   bus->addMotor(motor1);
//   bus->addMotor(motor2);
//   bus2->addMotor(motor3);
//   bus2->addMotor(motor4);
//   std::cout << "addMotor" << std::endl;

//   // 初始控制
//   MotorDevice::ControlData cmd;
//   cmd.controlMode = MotorDevice::Mode::VELOCITY_CONTROL;
//   cmd.targetVelocity = 0;
//   cmd.enable = true;
//   motor1->updateControl(cmd);
//   motor2->updateControl(cmd);

//   MotorDevice::ControlData cmd2;
//   cmd2.controlMode = MotorDevice::Mode::POSITION_CONTROL;
//   cmd2.targetPosition = 0;
//   cmd2.enable = true;
//   motor3->updateControl(cmd2);
//   motor4->updateControl(cmd2);

//   bus->start();
//   bus2->start();
//   std::cout << "start" << std::endl;

//   rclcpp::init(argc, argv);
//   rclcpp::spin(std::make_shared<DSWChair>(motor1, motor2, motor3, motor4));
//   rclcpp::shutdown();
//   return 0;
// }
