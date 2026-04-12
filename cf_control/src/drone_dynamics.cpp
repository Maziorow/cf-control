#include "cf_control/drone_dynamics.hpp"

#include <cmath>
#include <rclcpp_components/register_node_macro.hpp>

namespace evs
{
namespace cf
{

DroneDynamics::DroneDynamics(const rclcpp::NodeOptions & options)
: Node("drone_dynamics", options),
  state_{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
  thrust_(0.0),
  tau_x_(0.0),
  tau_y_(0.0),
  tau_z_(0.0)
{
  declare_parameter("mass", 0.027);
  declare_parameter("ixx", 1.4e-5);
  declare_parameter("iyy", 1.4e-5);
  declare_parameter("izz", 2.17e-5);
  declare_parameter("gravity", 9.81);
  declare_parameter("simulation_rate", 1000.0);
  declare_parameter("initial_position.x", 0.0);
  declare_parameter("initial_position.y", 0.0);
  declare_parameter("initial_position.z", 0.0);

  mass_ = get_parameter("mass").as_double();
  ixx_ = get_parameter("ixx").as_double();
  iyy_ = get_parameter("iyy").as_double();
  izz_ = get_parameter("izz").as_double();
  gravity_ = get_parameter("gravity").as_double();
  dt_ = 1.0 / get_parameter("simulation_rate").as_double();

  state_[0] = get_parameter("initial_position.x").as_double();
  state_[1] = get_parameter("initial_position.y").as_double();
  state_[2] = get_parameter("initial_position.z").as_double();

  state_publisher_ = create_publisher<cf_control_msgs::msg::DroneState>("drone_state", 10);
  control_subscriber_ = create_subscription<cf_control_msgs::msg::ThrustAndTorque>(
    "control_command", 10,
    [this](const cf_control_msgs::msg::ThrustAndTorque::SharedPtr msg) {
      control_callback(msg);
    });

  process_timer_ = create_wall_timer(
    std::chrono::duration<double>(dt_),
    [this]() { process(); });
}

void DroneDynamics::control_callback(const cf_control_msgs::msg::ThrustAndTorque::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(input_mutex_);
  thrust_ = msg->collective_thrust;
  tau_x_ = msg->torque.x;
  tau_y_ = msg->torque.y;
  tau_z_ = msg->torque.z;
}

std::array<double, 13> DroneDynamics::compute_derivatives(
  const std::array<double, 13> & s,
  double T, double tau_x, double tau_y, double tau_z)
{
  std::array<double, 13> ds{};

  const double qw = s[6], qx = s[7], qy = s[8], qz = s[9];
  const double wx = s[10], wy = s[11], wz = s[12];

  ds[0] = s[3];
  ds[1] = s[4];
  ds[2] = s[5];

  ds[3] = (T / mass_) * 2.0 * (qx * qz + qw * qy);
  ds[4] = (T / mass_) * 2.0 * (qy * qz - qw * qx);
  ds[5] = -gravity_ + (T / mass_) * (1.0 - 2.0 * (qx * qx + qy * qy));

  ds[6] = -0.5 * (qx * wx + qy * wy + qz * wz);
  ds[7] =  0.5 * (qw * wx + qy * wz - qz * wy);
  ds[8] =  0.5 * (qw * wy - qx * wz + qz * wx);
  ds[9] =  0.5 * (qw * wz + qx * wy - qy * wx);

  ds[10] = (tau_x - (izz_ - iyy_) * wy * wz) / ixx_;
  ds[11] = (tau_y - (ixx_ - izz_) * wx * wz) / iyy_;
  ds[12] = (tau_z - (iyy_ - ixx_) * wx * wy) / izz_;

  return ds;
}

std::array<double, 13> DroneDynamics::rk4_step(
  const std::array<double, 13> & state,
  double T, double tau_x, double tau_y, double tau_z,
  double dt)
{
  // Standard RK4: k1..k4 are derivative evaluations at state, mid-points, and end.
  // Control inputs T, tau_* are held constant over the step (zero-order hold).
  auto add = [](const std::array<double, 13> & a,
                double h,
                const std::array<double, 13> & b) {
    std::array<double, 13> out{};
    for (size_t i = 0; i < 13; ++i) out[i] = a[i] + h * b[i];
    return out;
  };

  const auto k1 = compute_derivatives(state,              T, tau_x, tau_y, tau_z);
  const auto k2 = compute_derivatives(add(state, dt/2, k1), T, tau_x, tau_y, tau_z);
  const auto k3 = compute_derivatives(add(state, dt/2, k2), T, tau_x, tau_y, tau_z);
  const auto k4 = compute_derivatives(add(state, dt,   k3), T, tau_x, tau_y, tau_z);

  std::array<double, 13> next{};
  for (size_t i = 0; i < 13; ++i) {
    next[i] = state[i] + (dt / 6.0) * (k1[i] + 2*k2[i] + 2*k3[i] + k4[i]);
  }

  // Re-normalise quaternion (indices 6-9) to prevent drift.
  const double q_norm = std::sqrt(
    next[6]*next[6] + next[7]*next[7] +
    next[8]*next[8] + next[9]*next[9]);
  next[6] /= q_norm;
  next[7] /= q_norm;
  next[8] /= q_norm;
  next[9] /= q_norm;

  return next;
}

void DroneDynamics::process()
{
  double T, tau_x, tau_y, tau_z;
  {
    std::lock_guard<std::mutex> lock(input_mutex_);
    T = thrust_;
    tau_x = tau_x_;
    tau_y = tau_y_;
    tau_z = tau_z_;
  }

  state_ = rk4_step(state_, T, tau_x, tau_y, tau_z, dt_);

  auto msg = cf_control_msgs::msg::DroneState();
  msg.timestamp = this->now().nanoseconds();
  msg.position.x = state_[0];
  msg.position.y = state_[1];
  msg.position.z = state_[2];
  msg.velocity.x = state_[3];
  msg.velocity.y = state_[4];
  msg.velocity.z = state_[5];
  msg.orientation.w = state_[6];
  msg.orientation.x = state_[7];
  msg.orientation.y = state_[8];
  msg.orientation.z = state_[9];
  msg.angular_velocity.x = state_[10];
  msg.angular_velocity.y = state_[11];
  msg.angular_velocity.z = state_[12];

  state_publisher_->publish(msg);
}

}  // namespace cf
}  // namespace evs

RCLCPP_COMPONENTS_REGISTER_NODE(evs::cf::DroneDynamics)
