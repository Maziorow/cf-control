#ifndef CF_CONTROL_DRONE_DYNAMICS_HPP_
#define CF_CONTROL_DRONE_DYNAMICS_HPP_

#include <array>
#include <cf_control_msgs/msg/drone_state.hpp>
#include <cf_control_msgs/msg/thrust_and_torque.hpp>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

namespace evs
{
namespace cf
{

class DroneDynamics : public rclcpp::Node
{
public:
  explicit DroneDynamics(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void process();
  void control_callback(const cf_control_msgs::msg::ThrustAndTorque::SharedPtr msg);

  std::array<double, 13> compute_derivatives(
    const std::array<double, 13> & state,
    double T, double tau_x, double tau_y, double tau_z);

  std::array<double, 13> euler_step(
    const std::array<double, 13> & state,
    double T, double tau_x, double tau_y, double tau_z,
    double dt);

  rclcpp::TimerBase::SharedPtr process_timer_;
  rclcpp::Publisher<cf_control_msgs::msg::DroneState>::SharedPtr state_publisher_;
  rclcpp::Subscription<cf_control_msgs::msg::ThrustAndTorque>::SharedPtr control_subscriber_;

  double mass_;
  double ixx_;
  double iyy_;
  double izz_;
  double gravity_;
  double dt_;

  std::array<double, 13> state_;

  double thrust_;
  double tau_x_;
  double tau_y_;
  double tau_z_;

  std::mutex input_mutex_;
};

}  // namespace cf
}  // namespace evs

#endif  // CF_CONTROL_DRONE_DYNAMICS_HPP_
