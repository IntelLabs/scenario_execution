cmake_minimum_required(VERSION 3.3)
project(scenario_execution_rviz)

set(CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 14)

add_compile_options(-Wall -Wextra -Werror -flto -fvisibility=hidden -z noexecstac -fPIC -D_FORTIFY_SOURCE=2 -Wl,-z,relro,-z,now -fstack-protector-strong)

find_package(ament_cmake REQUIRED)
find_package(rviz_common REQUIRED)
find_package(nav_msgs COMPONENTS)
find_package(std_srvs COMPONENTS)
find_package(scenario_execution_interfaces COMPONENTS)
find_package(py_trees_ros_interfaces COMPONENTS)
find_package(pluginlib REQUIRED)
find_package(rviz_ogre_vendor REQUIRED)

set(CMAKE_AUTOMOC ON)

find_package(Qt5 REQUIRED Core Widgets)
set(QT_LIBRARIES Qt5::Widgets)

set(SRC_FILES src/indicator_widget.cpp
              src/control_panel.cpp)

qt5_add_resources(SRC_FILES scenario_execution_rviz.qrc)

add_library(${PROJECT_NAME} SHARED ${SRC_FILES})

target_link_libraries(${PROJECT_NAME} ${QT_LIBRARIES}
                      rviz_ogre_vendor::OgreMain)

target_compile_features(${PROJECT_NAME} PRIVATE cxx_std_14)

pluginlib_export_plugin_description_file(rviz_common
                                          plugin_description.xml)

ament_target_dependencies(scenario_execution_rviz rclcpp nav_msgs std_srvs
                          scenario_execution_interfaces py_trees_ros_interfaces rviz_common)

ament_export_libraries(${PROJECT_NAME})

install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(FILES plugin_description.xml DESTINATION "share/${PROJECT_NAME}")

install(DIRECTORY icons/ DESTINATION "share/${PROJECT_NAME}")

ament_package()

