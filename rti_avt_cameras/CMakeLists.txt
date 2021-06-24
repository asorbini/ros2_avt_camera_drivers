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

add_executable(mono_camera_node
  src/nodes/mono_camera_node.cpp
  src/mono_camera.cpp
) 
target_link_libraries(mono_camera_node ${PROJECT_NAME})
install(TARGETS mono_camera_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_package()

