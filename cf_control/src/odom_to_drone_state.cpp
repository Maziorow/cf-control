#include "cf_control/odom_to_drone_state.hpp"

namespace evs
{
namespace cf
{

OdomToDroneState::OdomToDroneState(const rclcpp::NodeOptions & options)
: Node("odom_to_drone_state", options)
{
  state_publisher_ = create_publisher<cf_control_msgs::msg::DroneState>("drone_state", 10);

  odom_subscriber_ = create_subscription<nav_msgs::msg::Odometry>(
    "/crazyflie/odom", 10,
    [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
      odom_callback(msg);
    });

  RCLCPP_INFO(this->get_logger(), "OdomToDroneState node has been started.");
}

void OdomToDroneState::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  auto state_msg = cf_control_msgs::msg::DroneState();
  state_msg.timestamp = this->now().nanoseconds();
  state_msg.position.x = msg->pose.pose.position.x;
  state_msg.position.y = msg->pose.pose.position.y;
  state_msg.position.z = msg->pose.pose.position.z;

  // Gazebo odometry twist is in body frame — rotate to world frame
  const auto & q = msg->pose.pose.orientation;
  double qw = q.w, qx = q.x, qy = q.y, qz = q.z;
  double vb_x = msg->twist.twist.linear.x;
  double vb_y = msg->twist.twist.linear.y;
  double vb_z = msg->twist.twist.linear.z;

  // Rotate body-frame velocity to world frame using quaternion rotation: v_w = R * v_b
  state_msg.velocity.x = (1.0 - 2.0 * (qy * qy + qz * qz)) * vb_x +
                          2.0 * (qx * qy - qw * qz) * vb_y +
                          2.0 * (qx * qz + qw * qy) * vb_z;
  state_msg.velocity.y = 2.0 * (qx * qy + qw * qz) * vb_x +
                          (1.0 - 2.0 * (qx * qx + qz * qz)) * vb_y +
                          2.0 * (qy * qz - qw * qx) * vb_z;
  state_msg.velocity.z = 2.0 * (qx * qz - qw * qy) * vb_x +
                          2.0 * (qy * qz + qw * qx) * vb_y +
                          (1.0 - 2.0 * (qx * qx + qy * qy)) * vb_z;

  state_msg.orientation = msg->pose.pose.orientation;
  state_msg.angular_velocity = msg->twist.twist.angular;

  state_publisher_->publish(state_msg);
}

}  // namespace cf
}  // namespace evs

RCLCPP_COMPONENTS_REGISTER_NODE(evs::cf::OdomToDroneState)
