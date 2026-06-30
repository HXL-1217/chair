// lib/OdometryPublisher.cpp
#include "OdometryPublisher.h"
#include <cmath>
#include <iostream>

#ifndef M_PI
    #define M_PI 3.14159265358979323846
#endif

OdometryPublisher::OdometryPublisher(
    double publish_hz,
    const std::string& frame,
    const std::string& child_frame)
    : frame_(frame), child_frame_(child_frame), publish_dt_(1.0 / publish_hz) {
}

void OdometryPublisher::integrateAndPublish(
    double vx, double vy, double omega, 
    double current_time_sec)
{
    std::lock_guard<std::mutex> lock(mutex_);

    if (last_time_sec_ < 0) {
        last_time_sec_ = current_time_sec;
        return;
    }

    double dt = current_time_sec - last_time_sec_;
    if (dt <= 0.0 || dt > 0.1) {
        last_time_sec_ = current_time_sec;
        return;
    }

    // 一阶欧拉积分（车体速度 → 世界坐标系位姿）
    pose_x_ += (vx * std::cos(pose_yaw_) - vy * std::sin(pose_yaw_)) * dt;
    pose_y_ += (vx * std::sin(pose_yaw_) + vy * std::cos(pose_yaw_)) * dt;
    pose_yaw_ += omega * dt;
    pose_yaw_ = normalizeAngle(pose_yaw_);

    last_time_sec_ = current_time_sec;

    // 发布（ROS 2 或打印）
    publishOdometry(vx, vy, omega, current_time_sec);
}

void OdometryPublisher::resetPose(double x, double y, double yaw) {
    std::lock_guard<std::mutex> lock(mutex_);
    pose_x_ = x;
    pose_y_ = y;
    pose_yaw_ = normalizeAngle(yaw);
    last_time_sec_ = -1.0; // 下次 integrate 会重置时间
}

void OdometryPublisher::getCurrentPose(double& x, double& y, double& yaw) const {
    std::lock_guard<std::mutex> lock(mutex_);
    x = pose_x_;
    y = pose_y_;
    yaw = pose_yaw_;
}

double OdometryPublisher::normalizeAngle(double angle) const {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

void OdometryPublisher::publishOdometry(
    double vx, double vy, double omega,
    double current_time_sec)
{
#ifdef USE_ROS2
    if (!node_ || !odom_pub_) return;

    auto odom = std::make_unique<nav_msgs::msg::Odometry>();
    
    // 时间戳
    odom->header.stamp = rclcpp::Time(static_cast<uint64_t>(current_time_sec * 1e9));
    odom->header.frame_id = frame_;
    odom->child_frame_id = child_frame_;

    // 位置
    odom->pose.pose.position.x = pose_x_;
    odom->pose.pose.position.y = pose_y_;
    odom->pose.pose.position.z = 0.0;

    // 方向（四元数）
    tf2::Quaternion q;
    q.setRPY(0, 0, pose_yaw_);
    odom->pose.pose.orientation.x = q.x();
    odom->pose.pose.orientation.y = q.y();
    odom->pose.pose.orientation.z = q.z();
    odom->pose.pose.orientation.w = q.w();

    // 速度（车体系）
    odom->twist.twist.linear.x = vx;
    odom->twist.twist.linear.y = vy;
    odom->twist.twist.angular.z = omega;

    // 简单协方差
    odom->pose.covariance[0] = 0.01;   // x
    odom->pose.covariance[7] = 0.01;   // y
    odom->pose.covariance[35] = 0.02;  // yaw
    odom->twist.covariance[0] = 0.01;  // vx
    odom->twist.covariance[7] = 0.01;  // vy
    odom->twist.covariance[35] = 0.02; // omega

    odom_pub_->publish(std::move(odom));

    // 发布 TF
    if (tf_broadcaster_) {
        geometry_msgs::msg::TransformStamped transform;
        transform.header.stamp = odom->header.stamp;
        transform.header.frame_id = frame_;
        transform.child_frame_id = child_frame_;
        transform.transform.translation.x = pose_x_;
        transform.transform.translation.y = pose_y_;
        transform.transform.translation.z = 0.0;
        transform.transform.rotation.x = q.x();
        transform.transform.rotation.y = q.y();
        transform.transform.rotation.z = q.z();
        transform.transform.rotation.w = q.w();
        tf_broadcaster_->sendTransform(transform);
    }

#else
    // 非 ROS 2：可选打印或留空
    // std::cout << "[ODOM] x=" << pose_x_ << ", y=" << pose_y_ 
    //           << ", yaw=" << pose_yaw_ << std::endl;
#endif
}

#ifdef USE_ROS2
void OdometryPublisher::attachNode(std::shared_ptr<rclcpp::Node> node) {
    std::lock_guard<std::mutex> lock(mutex_);
    node_ = node;
    odom_pub_ = node_->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(node_);
}
#endif