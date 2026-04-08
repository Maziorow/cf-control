#include "cf_control/flatness.hpp"

#include <rclcpp_components/register_node_macro.hpp>

#include <cmath>

namespace evs
{
namespace cf
{

FlatInputProcessor::FlatInputProcessor(const rclcpp::NodeOptions & options)
: Node("flat_input_processor", options)
{
  // Initialize the publisher
  state_publisher_ = create_publisher<cf_control_msgs::msg::DroneState>("drone_state", 10);

  // Initialize the subscriber
  input_subscriber_ = create_subscription<cf_control_msgs::msg::FlatInput>(
    "flat_input", 10,
    [this](const cf_control_msgs::msg::FlatInput::SharedPtr msg) {
      input_callback(msg);
    });
}

void FlatInputProcessor::input_callback(const cf_control_msgs::msg::FlatInput::SharedPtr msg)
{
  // Create the output message
  auto state_msg = cf_control_msgs::msg::DroneState();

  // Pass through the timestamp (or generate a new one via this->now().nanoseconds())
  state_msg.timestamp = msg->timestamp;

  state_msg.position = msg->position;
  state_msg.velocity = msg->velocity;

  double acc_x = msg.acceleration.x;
  double acc_y = msg.acceleration.y;
  double acc_z = msg.acceleration.z;
  
  double acc_norm = sqrt(acc_x * acc_x + acc_y * acc_y + acc_z * acc_z);

  // Publish the computed state
  state_publisher_->publish(state_msg);
}

}  // namespace cf
}  // namespace evs

RCLCPP_COMPONENTS_REGISTER_NODE(evs::cf::FlatInputProcessor)