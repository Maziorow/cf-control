#include "cf_control/lee_controller.hpp"
#include "cf_control/lee_math.hpp"

#include <rclcpp_components/register_node_macro.hpp>

namespace evs
{
namespace cf
{

LeeController::LeeController(const rclcpp::NodeOptions & options)
: Node("lee_controller", options)
{
  // Physical parameters
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

  // Control gains
  declare_parameter("kp_pos", 6.0);
  declare_parameter("kv_pos", 4.0);
  declare_parameter("kr_att", 8.5e-3);
  declare_parameter("komega_att", 2.5e-3);

  kp_pos_     = get_parameter("kp_pos").as_double();
  kv_pos_     = get_parameter("kv_pos").as_double();
  kr_att_     = get_parameter("kr_att").as_double();
  komega_att_ = get_parameter("komega_att").as_double();

  declare_parameter("process_rate", 200.0);
  const double process_rate = get_parameter("process_rate").as_double();

  // ROS interfaces
  command_publisher_ =
    create_publisher<cf_control_msgs::msg::ThrustAndTorque>("control_command", 10);

  state_subscriber_ = create_subscription<cf_control_msgs::msg::DroneState>(
    "drone_state", 10,
    [this](const cf_control_msgs::msg::DroneState::SharedPtr msg) {
      state_callback(msg);
    });

  setpoint_subscriber_ = create_subscription<cf_control_msgs::msg::TrajectorySetpoint>(
    "trajectory_setpoint", 10,
    [this](const cf_control_msgs::msg::TrajectorySetpoint::SharedPtr msg) {
      setpoint_callback(msg);
    });

  process_timer_ = create_wall_timer(
    std::chrono::duration<double>(1.0 / process_rate),
    [this]() { process(); });

  RCLCPP_INFO(this->get_logger(), "LeeController node has been started.");
}

void LeeController::state_callback(const cf_control_msgs::msg::DroneState::SharedPtr msg)
{
  std::scoped_lock lock(data_mutex_);
  state_.px = msg->position.x;
  state_.py = msg->position.y;
  state_.pz = msg->position.z;
  state_.vx = msg->velocity.x;
  state_.vy = msg->velocity.y;
  state_.vz = msg->velocity.z;
  state_.qw = msg->orientation.w;
  state_.qx = msg->orientation.x;
  state_.qy = msg->orientation.y;
  state_.qz = msg->orientation.z;
  state_.wx = msg->angular_velocity.x;
  state_.wy = msg->angular_velocity.y;
  state_.wz = msg->angular_velocity.z;
  state_received_ = true;
}

void LeeController::setpoint_callback(const cf_control_msgs::msg::TrajectorySetpoint::SharedPtr msg)
{
  std::scoped_lock lock(data_mutex_);
  setpoint_.px  = msg->position.x;
  setpoint_.py  = msg->position.y;
  setpoint_.pz  = msg->position.z;
  setpoint_.vx  = msg->velocity.x;
  setpoint_.vy  = msg->velocity.y;
  setpoint_.vz  = msg->velocity.z;
  setpoint_.yaw = msg->yaw;
}

void LeeController::process()
{
  State state;
  Setpoint setpoint;
  {
    std::scoped_lock lock(data_mutex_);
    if (!state_received_) { return; }
    state    = state_;
    setpoint = setpoint_;
  }

  namespace li = lee_in;
  namespace lo = lee_out;

  LeeInputVec u{};
  u[li::POS_X] = state.px;  u[li::POS_Y] = state.py;  u[li::POS_Z] = state.pz;
  u[li::VEL_X] = state.vx;  u[li::VEL_Y] = state.vy;  u[li::VEL_Z] = state.vz;
  u[li::QW]    = state.qw;  u[li::QX]    = state.qx;
  u[li::QY]    = state.qy;  u[li::QZ]    = state.qz;
  u[li::OMEGA_X] = state.wx;  u[li::OMEGA_Y] = state.wy;  u[li::OMEGA_Z] = state.wz;

  u[li::DES_POS_X] = setpoint.px;  u[li::DES_POS_Y] = setpoint.py;  u[li::DES_POS_Z] = setpoint.pz;
  u[li::DES_VEL_X] = setpoint.vx;  u[li::DES_VEL_Y] = setpoint.vy;  u[li::DES_VEL_Z] = setpoint.vz;
  u[li::DES_YAW]   = setpoint.yaw;

  u[li::KP_POS]   = kp_pos_;
  u[li::KP_VEL]   = kv_pos_;
  u[li::KP_ATT]   = kr_att_;
  u[li::KP_OMEGA] = komega_att_;

  u[li::MASS]    = mass_;
  u[li::GRAVITY] = gravity_;
  u[li::IXX]     = ixx_;
  u[li::IYY]     = iyy_;
  u[li::IZZ]     = izz_;

  const LeeOutputVec y = compute_lee(u);

  auto cmd = cf_control_msgs::msg::ThrustAndTorque();
  cmd.timestamp        = this->now().nanoseconds();
  cmd.collective_thrust = y[lo::THRUST];
  cmd.torque.x          = y[lo::TORQUE_X];
  cmd.torque.y          = y[lo::TORQUE_Y];
  cmd.torque.z          = y[lo::TORQUE_Z];
  command_publisher_->publish(cmd);
}

}  // namespace cf
}  // namespace evs

RCLCPP_COMPONENTS_REGISTER_NODE(evs::cf::LeeController)
