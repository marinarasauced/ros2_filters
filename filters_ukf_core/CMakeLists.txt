cmake_minimum_required(VERSION 3.8)
project(filters_ukf_core)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

install(
  DIRECTORY include/filters_ukf_core/
  DESTINATION include/filters_ukf_core
)

ament_export_include_directories(include)
ament_package()
