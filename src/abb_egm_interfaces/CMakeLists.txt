cmake_minimum_required(VERSION 3.10)
project(abb_egm_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/ExecuteTrajectory.action"
  "msg/EgmRobot.msg"
  "msg/EgmSensor.msg"
  "msg/RobotJointState.msg"
  "msg/RobotPoseState.msg"
  "srv/SetControlMode.srv"
  DEPENDENCIES std_msgs sensor_msgs geometry_msgs trajectory_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()