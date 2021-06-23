cmake_minimum_required(VERSION 3.5.0)
project(ros2_avt_cameras)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(camera_info_manager REQUIRED)
find_package(image_transport REQUIRED)
find_package(camera_calibration_parsers REQUIRED)
find_package(vimba_sdk_vendor REQUIRED)

###########
## Build ##
###########
add_executable(mono_camera_node
  src/nodes/mono_camera_node.cpp
  src/mono_camera.cpp
  src/avt_vimba_camera.cpp
  src/frame_observer.cpp
)

target_include_directories(mono_camera_node
  PRIVATE include)

ament_target_dependencies(mono_camera_node
  "rclcpp"
  "rclcpp_components"
  "sensor_msgs"
  "std_msgs"
  "camera_info_manager"
  "image_transport"
  "camera_calibration_parsers"
  "vimba_sdk_vendor")
  
target_link_libraries(mono_camera_node VimbaSDK::VimbaCPP)

install(TARGETS mono_camera_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_package()
