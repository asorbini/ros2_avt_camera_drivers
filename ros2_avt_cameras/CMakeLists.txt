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
add_library(${PROJECT_NAME} SHARED
  src/mono_camera.cpp
  src/avt_vimba_camera.cpp
  src/frame_observer.cpp
)
target_include_directories(${PROJECT_NAME}
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")
target_link_libraries(${PROJECT_NAME} VimbaSDK::VimbaCPP)
ament_export_libraries(${PROJECT_NAME})
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
set(dependencies
  "rclcpp"
  "rclcpp_components"
  "sensor_msgs"
  "std_msgs"
  "camera_info_manager"
  "image_transport"
  "camera_calibration_parsers"
  "vimba_sdk_vendor"
)
ament_target_dependencies(${PROJECT_NAME} ${dependencies})
ament_export_dependencies(${dependencies})
install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

add_executable(mono_camera_node
  src/nodes/mono_camera_node.cpp
) 
target_link_libraries(mono_camera_node ${PROJECT_NAME})
install(TARGETS mono_camera_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_package()
