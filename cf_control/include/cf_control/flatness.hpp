#ifndef CF_CONTROL_FLATNESS_HPP_
#define CF_CONTROL_FLATNESS_HPP_

#include <cf_control_msgs/msg/drone_state.hpp>
#include <cf_control_msgs/msg/flat_input.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

namespace evs
{
namespace cf
{

class Flatness : public rclcpp::Node
{
public:
  explicit Flatness(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void input_callback(const cf_control_msgs::msg::FlatInput::SharedPtr msg);

  rclcpp::Publisher<cf_control_msgs::msg::DroneState>::SharedPtr state_publisher_;
  rclcpp::Subscription<cf_control_msgs::msg::FlatInput>::SharedPtr input_subscriber_;
};

}  // namespace cf
}  // namespace evs

#endif  // CF_CONTROL_FLAT_INPUT_PROCESSOR_HPP_