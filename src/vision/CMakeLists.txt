cmake_minimum_required(VERSION 3.8)
project(vision)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(ros2_utils REQUIRED)


# find dependencies
set(var_dependencies
    ament_cmake
    rclcpp
    std_msgs
    ros2_utils
    ros2_interface
    sensor_msgs
    OpenCV
    cv_bridge
    image_transport

)

foreach(var_dependency ${var_dependencies})
  find_package(${var_dependency} REQUIRED)
endforeach()

include_directories(include)

add_executable(capture src/capture.cpp)
ament_target_dependencies(capture ${var_dependencies})
target_link_libraries(capture ${OpenCV_LIBS})

install(TARGETS
  capture
  DESTINATION lib/${PROJECT_NAME}
)


# Python scripts
install(
  PROGRAMS
    script/detection.py 
    script/hand_track.py
    script/face_detection.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
