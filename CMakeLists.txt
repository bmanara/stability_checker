cmake_minimum_required(VERSION 3.8)
project(stability_checker)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pinocchio REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
  include
  ${pinocchio_INCLUDE_DIRS}
)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_executable(stability_checker src/stability_checker.cpp)

ament_target_dependencies(stability_checker
  rclcpp
  sensor_msgs
  geometry_msgs
  Eigen3
  pinocchio
)

install(
  TARGETS stability_checker
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
