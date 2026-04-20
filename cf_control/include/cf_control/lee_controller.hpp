#ifndef CF_CONTROL_LEE_CONTROLLER_HPP_
#define CF_CONTROL_LEE_CONTROLLER_HPP_

#include <cf_control_msgs/msg/drone_state.hpp>
#include <cf_control_msgs/msg/thrust_and_torque.hpp>
#include <cf_control_msgs/msg/trajectory_setpoint.hpp>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

namespace evs
{
namespace cf
{

class LeeController : public rclcpp::Node
{
public:
  explicit LeeController(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void state_callback(const cf_control_msgs::msg::DroneState::SharedPtr msg);
  void setpoint_callback(const cf_control_msgs::msg::TrajectorySetpoint::SharedPtr msg);
  void process();

  rclcpp::TimerBase::SharedPtr process_timer_;

  // ROS interfaces
  rclcpp::Publisher<cf_control_msgs::msg::ThrustAndTorque>::SharedPtr command_publisher_;
  rclcpp::Subscription<cf_control_msgs::msg::DroneState>::SharedPtr state_subscriber_;
  rclcpp::Subscription<cf_control_msgs::msg::TrajectorySetpoint>::SharedPtr setpoint_subscriber_;

  // Control gains
  double kp_pos_, kv_pos_, kr_att_, komega_att_;

  // Physical parameters
  double mass_, ixx_, iyy_, izz_, gravity_;

  // Buffered current state
  struct State
  {
    double px, py, pz;
    double vx, vy, vz;
    double qw, qx, qy, qz;
    double wx, wy, wz;
  };

  // Buffered setpoint
  struct Setpoint
  {
    double px, py, pz;
    double vx, vy, vz;
    double yaw;
  };

  State state_{};
  Setpoint setpoint_{};
  bool state_received_{false};
  std::mutex data_mutex_;
};

}  // namespace cf
}  // namespace evs

#endif  // CF_CONTROL_LEE_CONTROLLER_HPP_
