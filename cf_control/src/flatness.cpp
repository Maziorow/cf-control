#include "cf_control/flatness.hpp"
#include "cf_control/flatness_math.hpp"

#include <rclcpp_components/register_node_macro.hpp>

namespace evs
{
namespace cf
{

FlatInputProcessor::FlatInputProcessor(const rclcpp::NodeOptions & options)
: Node("flat_input_processor", options)
{
  declare_parameter("mass", 0.027);
  declare_parameter("ixx", 1.4e-5);
  declare_parameter("iyy", 1.4e-5);
  declare_parameter("izz", 2.17e-5);
  declare_parameter("gravity", 9.81);

  mass_    = get_parameter("mass").as_double();
  ixx_     = get_parameter("ixx").as_double();
  iyy_     = get_parameter("iyy").as_double();
  izz_     = get_parameter("izz").as_double();
  gravity_ = get_parameter("gravity").as_double();

  state_publisher_ = create_publisher<cf_control_msgs::msg::DroneState>("drone_state", 10);
  thrust_torque_publisher_ =
    create_publisher<cf_control_msgs::msg::ThrustAndTorque>("control_command", 10);

  input_subscriber_ = create_subscription<cf_control_msgs::msg::FlatInput>(
    "flat_input", 10,
    [this](const cf_control_msgs::msg::FlatInput::SharedPtr msg) {
      input_callback(msg);
    });
}

void FlatInputProcessor::input_callback(const cf_control_msgs::msg::FlatInput::SharedPtr msg)
{
  namespace fi = flat_in;
  namespace fo = flat_out;

  // ── Unpack FlatInput message into the input vector ─────────────────────
  FlatInputVec u{};
  u[fi::POS_X] = msg->position.x;  u[fi::POS_Y] = msg->position.y;  u[fi::POS_Z] = msg->position.z;
  u[fi::VEL_X] = msg->velocity.x;  u[fi::VEL_Y] = msg->velocity.y;  u[fi::VEL_Z] = msg->velocity.z;
  u[fi::ACC_X] = msg->acceleration.x;
  u[fi::ACC_Y] = msg->acceleration.y;
  u[fi::ACC_Z] = msg->acceleration.z;
  u[fi::JERK_X] = msg->jerk.x;  u[fi::JERK_Y] = msg->jerk.y;  u[fi::JERK_Z] = msg->jerk.z;
  u[fi::SNAP_X] = msg->snap.x;  u[fi::SNAP_Y] = msg->snap.y;  u[fi::SNAP_Z] = msg->snap.z;
  u[fi::YAW]     = msg->yaw;
  u[fi::YAW_VEL] = msg->yaw_velocity;
  u[fi::YAW_ACC] = msg->yaw_acceleration;
  u[fi::MASS]    = mass_;
  u[fi::GRAVITY] = gravity_;
  u[fi::IXX] = ixx_;  u[fi::IYY] = iyy_;  u[fi::IZZ] = izz_;

  // ── Core computation ───────────────────────────────────────────────────
  const FlatOutputVec y = compute_flatness(u);

  // ── Pack output vector into DroneState message ─────────────────────────
  auto state_msg = cf_control_msgs::msg::DroneState();
  state_msg.timestamp          = msg->timestamp;
  state_msg.position.x         = y[fo::POS_X];
  state_msg.position.y         = y[fo::POS_Y];
  state_msg.position.z         = y[fo::POS_Z];
  state_msg.velocity.x         = y[fo::VEL_X];
  state_msg.velocity.y         = y[fo::VEL_Y];
  state_msg.velocity.z         = y[fo::VEL_Z];
  state_msg.orientation.w      = y[fo::QW];
  state_msg.orientation.x      = y[fo::QX];
  state_msg.orientation.y      = y[fo::QY];
  state_msg.orientation.z      = y[fo::QZ];
  state_msg.angular_velocity.x = y[fo::OMEGA_X];
  state_msg.angular_velocity.y = y[fo::OMEGA_Y];
  state_msg.angular_velocity.z = y[fo::OMEGA_Z];
  state_publisher_->publish(state_msg);

  // ── Pack output vector into ThrustAndTorque message ────────────────────
  auto tt_msg = cf_control_msgs::msg::ThrustAndTorque();
  tt_msg.timestamp         = msg->timestamp;
  tt_msg.collective_thrust = y[fo::THRUST];
  tt_msg.torque.x          = y[fo::TORQUE_X];
  tt_msg.torque.y          = y[fo::TORQUE_Y];
  tt_msg.torque.z          = y[fo::TORQUE_Z];
  thrust_torque_publisher_->publish(tt_msg);
}

}  // namespace cf
}  // namespace evs

RCLCPP_COMPONENTS_REGISTER_NODE(evs::cf::FlatInputProcessor)
