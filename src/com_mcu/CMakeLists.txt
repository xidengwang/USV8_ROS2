cmake_minimum_required(VERSION 3.5)
project(com_mcu)

find_package(ament_cmake REQUIRED)
find_package(traj REQUIRED)
find_package(usv_interfaces REQUIRED)

install(PROGRAMS
  scripts/uart_example_accept.py
  scripts/uart_example_send.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
