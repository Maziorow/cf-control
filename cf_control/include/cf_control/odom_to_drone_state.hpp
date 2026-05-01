#ifndef CF_CONTROL_ODOM_TO_DRONE_STATE_HPP_
#define CF_CONTROL_ODOM_TO_DRONE_STATE_HPP_

#include <cf_control_msgs/msg/drone_state.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

namespace evs
{
namespace cf
{

class OdomToDroneState : public rclcpp::Node
{
public:
  explicit OdomToDroneState(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);

  rclcpp::Publisher<cf_control_msgs::msg::DroneState>::SharedPtr state_publisher_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;
};

}  // namespace cf
}  // namespace evs

#endif  // CF_CONTROL_ODOM_TO_DRONE_STATE_HPP_
