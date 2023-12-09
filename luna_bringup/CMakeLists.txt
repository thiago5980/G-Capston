cmake_minimum_required(VERSION 3.5)
project(luna_bringup)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(luna_msg REQUIRED)
find_package(md REQUIRED)
find_package(ydlidar_ros2_driver REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav_msgs REQUIRED)
# Include Cpp "include" directory
include_directories(include)

# Create Cpp executable
# add_executable(helloworld_publisher src/helloworld_publisher.cpp)
# ament_target_dependencies(helloworld_publisher rclcpp std_msgs)

# install(TARGETS
#   helloworld_publisher
#   DESTINATION lib/${PROJECT_NAME}
# )

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  scripts/luna_odom.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch params config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
