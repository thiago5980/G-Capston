cmake_minimum_required(VERSION 3.5)
project(md)

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
find_package(serial REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclpy REQUIRED)
find_package(luna_msg REQUIRED)


add_library(md_library SHARED
  src/md_node/com.cpp
  src/md_node/robot.cpp
)

ament_target_dependencies(md_library rclcpp std_msgs luna_msg geometry_msgs tf2 nav_msgs sensor_msgs serial)

include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
)


add_executable(md src/md_node/main.cpp)
add_dependencies(md md_library)
target_link_libraries(md md_library ${rclcpp_LIBRARIES} ${serial_LIBRARIES})
ament_target_dependencies(md rclcpp std_msgs luna_msg geometry_msgs tf2 nav_msgs sensor_msgs serial)

# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION $share/${PROJECT_NAME}
#   FILES_MATCHING PATTERN "*.hpp"
# )

install(TARGETS
  md
  DESTINATION lib/${PROJECT_NAME})

install (DIRECTORY
  launch param
  DESTINATION share/${PROJECT_NAME}
)

install (DIRECTORY include/
  DESTINATION include
)

ament_package()