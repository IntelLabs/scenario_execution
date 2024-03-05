cmake_minimum_required(VERSION 2.8.3)
project(scenario_execution_control)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
ament_export_dependencies(rclpy)

# Install launch files.
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)

ament_package()
