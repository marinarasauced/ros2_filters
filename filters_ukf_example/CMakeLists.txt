cmake_minimum_required(VERSION 3.8)
project(filters_ukf_example)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(filters_msgs REQUIRED)
find_package(filters_ukf_core REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

add_executable(example_ukf src/node.cpp)

ament_target_dependencies(
  example_ukf
  "rclcpp"
  "filters_msgs"
  "filters_ukf_core"
  "sensor_msgs"
  "Eigen3"
)

install(
  TARGETS example_ukf
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/filters_ukf_example/
  DESTINATION include/filters_ukf_example
)

ament_package()
