#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

using namespace std::chrono_literals;

class DSWChair : public rclcpp::Node
{
public:
  DSWChair()
  : Node("odom_publisher")
  {
    // 初始化变量
    x_ = 0.0;
    y_ = 0.0;
    theta_ = 0.0;
    last_time_ = this->now();

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
    current_vel_.linear.x = msg->linear.x;
    current_vel_.angular.z = msg->angular.z;
  }

  void publishOdometry()
  {
    auto current_time = this->now();
    double dt = (current_time - last_time_).seconds();

    // 积分计算位置（简单里程计模型）
    double dx = current_vel_.linear.x * cos(theta_) * dt;
    double dy = current_vel_.linear.x * sin(theta_) * dt;
    double dtheta = current_vel_.angular.z * dt;

    x_ += dx;
    y_ += dy;
    theta_ += dtheta;

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

    // 速度（在 base_link 坐标系中）
    odom.twist.twist.linear.x = current_vel_.linear.x;
    odom.twist.twist.angular.z = current_vel_.angular.z;

    // 发布
    odom_pub_->publish(odom);

    // 广播 TF（可选但推荐）
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

    last_time_ = current_time;
  }

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  geometry_msgs::msg::Twist current_vel_;
  double x_, y_, theta_;
  rclcpp::Time last_time_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DSWChair>());
  rclcpp::shutdown();
  return 0;
}