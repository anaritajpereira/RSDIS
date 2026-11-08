cmake_minimum_required(VERSION 3.5)
project(missao_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/DetecaoAlvo.msg"
  "msg/EstimativaAlvo.msg"
  "action/Resgatar.action"
  "action/Rebocar.action"
  DEPENDENCIES std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
