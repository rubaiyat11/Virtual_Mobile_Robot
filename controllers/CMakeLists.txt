cmake_minimum_required(VERSION 3.8)
project(controllers)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(mission_interface REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(mpc_controller_node src/mpc_controller_node.cpp)

ament_target_dependencies(
  mpc_controller_node
  rclcpp
  mission_interface
)

target_include_directories(mpc_controller_node PUBLIC /usr/include/eigen3)

install(TARGETS
  mpc_controller_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
