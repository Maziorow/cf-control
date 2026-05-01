#ifndef CF_CONTROL_ROUND_TRAJECTORY_HPP_
#define CF_CONTROL_ROUND_TRAJECTORY_HPP_

#include <cf_control_msgs/msg/trajectory_setpoint.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

namespace evs
{
namespace cf
{

class RoundTrajectory : public rclcpp::Node
{
public:
  explicit RoundTrajectory(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void process();

  rclcpp::TimerBase::SharedPtr process_timer_;
  rclcpp::Publisher<cf_control_msgs::msg::TrajectorySetpoint>::SharedPtr setpoint_publisher_;

  double radius_;
  double altitude_;
  double angular_speed_;
  double center_x_;
  double center_y_;
  double yaw_;
  double takeoff_time_;

  rclcpp::Time start_time_;
  bool started_;
};

}  // namespace cf
}  // namespace evs

#endif  // CF_CONTROL_ROUND_TRAJECTORY_HPP_
