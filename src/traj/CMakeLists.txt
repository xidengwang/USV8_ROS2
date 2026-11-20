cmake_minimum_required(VERSION 3.5)
project(traj)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(usv_interfaces REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Expect.msg"
)

ament_export_dependencies(rosidl_default_runtime)

install(PROGRAMS
  scripts/trajgen.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  data
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
