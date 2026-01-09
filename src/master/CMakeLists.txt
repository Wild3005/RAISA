cmake_minimum_required(VERSION 3.8)
project(master)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ros2_utils REQUIRED)
find_package(ros2_interface REQUIRED)
find_package(nlohmann_json REQUIRED)

# Master node executable
add_executable(master src/master.cpp)

add_executable(convert_csv src/convert_csv.cpp)

ament_target_dependencies(master
  rclcpp
  geometry_msgs
  std_msgs
  ros2_utils
  ros2_interface
)

ament_target_dependencies(convert_csv
  rclcpp
  geometry_msgs
  ros2_interface
)

target_link_libraries(master nlohmann_json::nlohmann_json)

install(TARGETS
  master
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  convert_csv 
  DESTINATION
  lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
