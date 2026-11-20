cmake_minimum_required(VERSION 3.5)
project(usv_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Controller.msg"
  "msg/McuAux.msg"
  "msg/McuInput.msg"
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
