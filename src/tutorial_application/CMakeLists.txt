cmake_minimum_required(VERSION 3.8)
project(tutorial_application)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})


set(NODES
tutorial_application/my_control.py
tutorial_application/camera_view.py
tutorial_application/move_box.py
tutorial_application/bk_subtract.py
tutorial_application/alter_setpoint.py
)

install(PROGRAMS
    ${NODES}
DESTINATION lib/${PROJECT_NAME}
)

ament_package()
