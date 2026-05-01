cmake_minimum_required(VERSION 3.8)
project(cf_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(actuator_msgs REQUIRED)
find_package(cf_control_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

# libraries
add_library(mixer_lib SHARED src/mixer.cpp)
ament_target_dependencies(mixer_lib rclcpp rclcpp_components actuator_msgs cf_control_msgs)
target_include_directories(mixer_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
rclcpp_components_register_node(
    mixer_lib
    PLUGIN "evs::cf::Mixer"
    EXECUTABLE mixer
)

add_library(drone_dynamics_lib SHARED src/drone_dynamics.cpp)
ament_target_dependencies(drone_dynamics_lib rclcpp rclcpp_components cf_control_msgs)
target_include_directories(drone_dynamics_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
rclcpp_components_register_node(
    drone_dynamics_lib
    PLUGIN "evs::cf::DroneDynamics"
    EXECUTABLE drone_dynamics
)

add_library(flatness_lib SHARED src/flatness.cpp)
ament_target_dependencies(flatness_lib rclcpp rclcpp_components cf_control_msgs)
target_include_directories(flatness_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
rclcpp_components_register_node(
    flatness_lib
    PLUGIN "evs::cf::Flatness"
    EXECUTABLE flatness
)

add_library(lee_controller_lib SHARED src/lee_controller.cpp)
ament_target_dependencies(lee_controller_lib rclcpp rclcpp_components cf_control_msgs)
target_include_directories(lee_controller_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
rclcpp_components_register_node(
    lee_controller_lib
    PLUGIN "evs::cf::LeeController"
    EXECUTABLE lee_controller
)

add_library(round_trajectory_lib SHARED src/round_trajectory.cpp)
ament_target_dependencies(round_trajectory_lib rclcpp rclcpp_components cf_control_msgs)
target_include_directories(round_trajectory_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
rclcpp_components_register_node(
    round_trajectory_lib
    PLUGIN "evs::cf::RoundTrajectory"
    EXECUTABLE round_trajectory
)

add_library(odom_to_drone_state_lib SHARED src/odom_to_drone_state.cpp)
ament_target_dependencies(odom_to_drone_state_lib rclcpp rclcpp_components cf_control_msgs nav_msgs)
target_include_directories(odom_to_drone_state_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
rclcpp_components_register_node(
    odom_to_drone_state_lib
    PLUGIN "evs::cf::OdomToDroneState"
    EXECUTABLE odom_to_drone_state
)

# install shared libraries
install(TARGETS
  mixer_lib
  drone_dynamics_lib
  flatness_lib
  lee_controller_lib
  round_trajectory_lib
  odom_to_drone_state_lib
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
ament_export_targets(export_${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
