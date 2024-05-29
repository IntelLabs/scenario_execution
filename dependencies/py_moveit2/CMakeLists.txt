cmake_minimum_required(VERSION 3.5)
project(py_moveit2)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Install Python package
ament_python_install_package(py_moveit2)

install(DIRECTORY
    DESTINATION lib/${PROJECT_NAME}
)

# Setup the project
ament_package()
