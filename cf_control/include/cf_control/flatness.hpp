#ifndef CF_CONTROL_FLATNESS_HPP_
#define CF_CONTROL_FLATNESS_HPP_

#include <cf_control_msgs/msg/drone_state.hpp>
#include <cf_control_msgs/msg/flat_input.hpp>
#include <cf_control_msgs/msg/thrust_and_torque.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

namespace evs
{
namespace cf
{

class FlatInputProcessor : public rclcpp::Node
{
public:
  explicit FlatInputProcessor(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void input_callback(const cf_control_msgs::msg::FlatInput::SharedPtr msg);

  rclcpp::Publisher<cf_control_msgs::msg::DroneState>::SharedPtr state_publisher_;
  rclcpp::Publisher<cf_control_msgs::msg::ThrustAndTorque>::SharedPtr thrust_torque_publisher_;
  rclcpp::Subscription<cf_control_msgs::msg::FlatInput>::SharedPtr input_subscriber_;

  double mass_, ixx_, iyy_, izz_, gravity_;
};

}  // namespace cf
}  // namespace evs

#endif  // CF_CONTROL_FLATNESS_HPP_
