// lib/OdometryPublisher.h
#pragma once

#include <string>
#include <mutex>
#include <memory>

#ifdef USE_ROS2
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#endif

class OdometryPublisher {
public:
    // 构造函数：publish_hz 仅用于 ROS 2 QoS（可选）
    OdometryPublisher(
        double publish_hz = 20.0,
        const std::string& frame = "odom",
        const std::string& child_frame = "base_link");

    // 主要接口：传入车体速度和当前时间（秒）
    void integrateAndPublish(double vx, double vy, double omega, double current_time_sec);

    // 重置位姿
    void resetPose(double x = 0.0, double y = 0.0, double yaw = 0.0);

    // 获取当前位姿
    void getCurrentPose(double& x, double& y, double& yaw) const;

#ifdef USE_ROS2
    // 绑定 ROS 2 节点（必须在 integrateAndPublish 前调用）
    void attachNode(std::shared_ptr<rclcpp::Node> node);
#endif

private:
    void publishOdometry(double vx, double vy, double omega, double current_time_sec);
    double normalizeAngle(double angle) const;

    // 位姿状态
    double pose_x_ = 0.0;
    double pose_y_ = 0.0;
    double pose_yaw_ = 0.0;
    double last_time_sec_ = -1.0; // -1 表示未初始化

    // 配置
    std::string frame_;
    std::string child_frame_;
    double publish_dt_;

    mutable std::mutex mutex_;

#ifdef USE_ROS2
    std::shared_ptr<rclcpp::Node> node_ = nullptr;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_ = nullptr;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_ = nullptr;
#endif
};