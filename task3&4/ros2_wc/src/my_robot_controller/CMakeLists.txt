cmake_minimum_required(VERSION 3.5)
project(my_robot_controller)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(action_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/CountUntil.action"
  DEPENDENCIES action_msgs
)

ament_export_dependencies(rosidl_default_runtime)

install(
  DIRECTORY my_robot_controller
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
