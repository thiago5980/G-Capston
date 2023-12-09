cmake_minimum_required(VERSION 3.5)
project(capston_main)

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
find_package(std_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(capston_msgs REQUIRED)

include_directories(include)

set(ROSDEPENDENCIES
  rclcpp
  std_msgs
  capston_msgs
  geometry_msgs
)

install(
  DIRECTORY launch params
  DESTINATION share/${PROJECT_NAME}
)

add_executable(capston_main_node src/main.cpp)
ament_target_dependencies(capston_main_node ${ROSDEPENDENCIES})
install(TARGETS
  capston_main_node
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(test_service src/test.cpp)
ament_target_dependencies(test_service ${ROSDEPENDENCIES})
install(TARGETS
  test_service
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
