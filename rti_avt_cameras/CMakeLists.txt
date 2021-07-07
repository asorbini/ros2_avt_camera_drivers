cmake_minimum_required(VERSION 3.12)
project(rti_avt_cameras)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ros2_avt_cameras REQUIRED)
find_package(connext_msgs REQUIRED)
find_package(ros2dds REQUIRED)

set(${PROJECT_NAME}_DDS_TYPES
  sensor_msgs/msg/Image
  sensor_msgs/msg/CameraInfo
  sensor_msgs/msg/RegionOfInterest
  std_msgs/msg/Header
  builtin_interfaces/msg/Header
  builtin_interfaces/msg/Time
)
list(JOIN ${PROJECT_NAME}_DDS_TYPES "|" ${PROJECT_NAME}_INCLUDE_REGEX)
connext_msgs_generate_library(${PROJECT_NAME}_ddstypes
  VARIANTS basic flat_zc
  INCLUDE_REGEX "${${PROJECT_NAME}_INCLUDE_REGEX}"
  INSTALL_IDLS
)

add_executable(mono_camera_node
  src/nodes/mono_camera_node.cpp
  src/mono_camera.cpp
  src/rti_image_helpers.cpp
) 
target_include_directories(mono_camera_node
  PRIVATE
    include
)
target_link_libraries(mono_camera_node ${PROJECT_NAME}_ddstypes)
ament_target_dependencies(mono_camera_node
  ros2_avt_cameras
  ros2dds
)
install(TARGETS mono_camera_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_package()

