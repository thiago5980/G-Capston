cmake_minimum_required(VERSION 3.5)
project(capston_plan)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
set(CMAKE_BUILD_TYPE Release)


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(Eigen3 REQUIRED)
find_package(OpenCV 4 REQUIRED)
find_package(CGAL QUIET COMPONENTS Core)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(capston_msgs REQUIRED)

include_directories(include)

include_directories(
  ${EIGEN_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}  
  ${CGAL_INCLUDE_DIRS}
)

link_directories(${CGAL_LIBRARY_DIRS})
link_directories(${OpenCV_LIBRARY_DIRS})
link_directories(${EIGEN_INCLUDE_DIRS})

set(DEPENDENCIES
  Eigen3
  OpenCV
  CGAL
)
set(ROSDEPENDENCIES
  rclcpp
  std_msgs
  capston_msgs
  geometry_msgs
)
file(GLOB SOURCES "src/*.cc" "src/*.cpp")

add_executable(planner src/main.cpp ${SOURCES})
ament_target_dependencies(planner ${ROSDEPENDENCIES} ${DEPENDENCIES})
target_link_libraries(planner 
  ${OpenCV_LIBRARIES}
  CGAL::CGAL
  CGAL::CGAL_Core 
)
install(TARGETS
  planner
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  data
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
