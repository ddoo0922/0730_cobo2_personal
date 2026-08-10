cmake_minimum_required(VERSION 3.8)
project(vla_interfaces)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/SceneObject.msg"
  "msg/SceneSnapshot.msg"
  "msg/RobotAction.msg"
  "msg/RobotState.msg"
  "msg/AgentReply.msg"
  "msg/TcpPose.msg"
  "msg/GraspRequest.msg"
  "msg/GraspPlan.msg"
  DEPENDENCIES geometry_msgs std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
