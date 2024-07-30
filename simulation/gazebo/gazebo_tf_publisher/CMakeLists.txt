cmake_minimum_required(VERSION 3.8)
project(gazebo_tf_publisher)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)

# gazebo
find_package(ignition-transport11 REQUIRED)

add_executable(gazebo_tf_publisher_node src/gazebo_tf_publisher_node.cpp)

ament_target_dependencies(gazebo_tf_publisher_node
  rclcpp
  geometry_msgs
  tf2_msgs
  ignition-transport11
)

install(DIRECTORY 
  launch
  DESTINATION share/${PROJECT_NAME})

install(TARGETS 
  gazebo_tf_publisher_node
  DESTINATION lib/${PROJECT_NAME}
  )

ament_package()
