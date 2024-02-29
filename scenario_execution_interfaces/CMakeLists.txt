cmake_minimum_required(VERSION 3.8)
project(scenario_execution_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  msg/ScenarioStatus.msg
  msg/Scenario.msg
  msg/ScenarioList.msg
  msg/ScenarioExecutionStatus.msg
  srv/ExecuteScenario.srv
  DEPENDENCIES
  builtin_interfaces
 )

ament_export_dependencies(rosidl_default_runtime)

ament_package()
