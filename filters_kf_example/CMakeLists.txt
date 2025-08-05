cmake_minimum_required(VERSION 3.8)
project(filters_kf_example)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(filters_base REQUIRED)
find_package(filters_kf_core REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rosidl_default_generators REQUIRED)

include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

rosidl_generate_interfaces(
  ${PROJECT_NAME}
  "msg/FakeX.msg"
  "msg/FakeZ.msg"
  DEPENDENCIES std_msgs
)

rosidl_get_typesupport_target(
  cpp_typesupport_target
  ${PROJECT_NAME}
  "rosidl_typesupport_cpp"
)

add_executable(
  node
  src/node.cpp
)

ament_target_dependencies(
  node
  "rclcpp"
  "std_msgs"
  "filters_base"
  "filters_kf_core"
  "Eigen3"
)

target_link_libraries(
  node
  ${cpp_typesupport_target}
)

add_executable(
  fake
  src/fake.cpp
)

ament_target_dependencies(
  fake
  "rclcpp"
  "std_msgs"
  "filters_base"
  "filters_kf_core"
  "Eigen3"
)

target_link_libraries(
  fake
  ${cpp_typesupport_target}
)

install(
  TARGETS node fake
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)

ament_package()
