cmake_minimum_required(VERSION 3.8)
project(communication)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(ros2_interface REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(action_msgs REQUIRED)
find_package(ros2_utils REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)



find_package(cpr REQUIRED)
find_package(nlohmann_json REQUIRED)

include_directories(
  include
  )
  
  
add_executable(io_reeman_node
  src/io_reeman.cpp
  include/ReemanClient.hpp   # header-only, still include to install properly
)

target_link_libraries(io_reeman_node
  cpr::cpr
)

ament_target_dependencies(io_reeman_node
  rclcpp
  ros2_interface
  geometry_msgs
  std_msgs
  nav_msgs
  sensor_msgs
  cpr
  nlohmann_json
  rclcpp_action
  action_msgs
  ros2_utils
  pcl_conversions
  pcl_ros

)

install(TARGETS 
  io_reeman_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
