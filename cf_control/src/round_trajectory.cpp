#include "cf_control/round_trajectory.hpp"

#include <chrono>
#include <cmath>

namespace evs
{
namespace cf
{

RoundTrajectory::RoundTrajectory(const rclcpp::NodeOptions & options)
: Node("round_trajectory", options),
  started_(false)
{
  declare_parameter("radius", 2.0);
  declare_parameter("altitude", 1.5);
  declare_parameter("angular_speed", 0.15);
  declare_parameter("center_x", 0.0);
  declare_parameter("center_y", 0.0);
  declare_parameter("yaw", 0.0);
  declare_parameter("publish_rate", 50.0);

  radius_ = get_parameter("radius").as_double();
  altitude_ = get_parameter("altitude").as_double();
  angular_speed_ = get_parameter("angular_speed").as_double();
  center_x_ = get_parameter("center_x").as_double();
  center_y_ = get_parameter("center_y").as_double();
  yaw_ = get_parameter("yaw").as_double();
  const double publish_rate = get_parameter("publish_rate").as_double();

  setpoint_publisher_ =
    create_publisher<cf_control_msgs::msg::TrajectorySetpoint>("trajectory_setpoint", 10);

  process_timer_ = create_wall_timer(
    std::chrono::duration<double>(1.0 / publish_rate),
    [this]() { process(); });

  RCLCPP_INFO(this->get_logger(), "RoundTrajectory node has been started.");
}

void RoundTrajectory::process()
{
  if (!started_) {
    start_time_ = this->now();
    started_ = true;
  }

  const double t = (this->now() - start_time_).seconds();
  const double phase = angular_speed_ * t;

  auto msg = cf_control_msgs::msg::TrajectorySetpoint();
  msg.timestamp = this->now().nanoseconds();
  msg.position.x = center_x_ + radius_ * std::cos(phase);
  msg.position.y = center_y_ + radius_ * std::sin(phase);
  msg.position.z = get_parameter("altitude").as_double();
  msg.velocity.x = -radius_ * angular_speed_ * std::sin(phase);
  msg.velocity.y = radius_ * angular_speed_ * std::cos(phase);
  msg.velocity.z = 0.0;
  msg.yaw = yaw_;

  setpoint_publisher_->publish(msg);
}

}  // namespace cf
}  // namespace evs

RCLCPP_COMPONENTS_REGISTER_NODE(evs::cf::RoundTrajectory)
