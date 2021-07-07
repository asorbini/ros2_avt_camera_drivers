// Copyright 2021 Real-Time Innovations, Inc. (RTI)
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef rti_image_helpers_hpp
#define rti_image_helpers_hpp

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>

#include <VimbaCPP/Include/VimbaCPP.h>
#include "sensor_msgs/image_encodings.hpp"

#include <dds/dds.hpp>
#include <rti/core/QosProviderParams.hpp>
#include "rti/topic/flat/FlatData.hpp"
#include "ros2/flat_zc/sensor_msgs/msg/Image.hpp"
#include "ros2/flat_zc/sensor_msgs/msg/CameraInfo.hpp"

namespace ros2 { namespace flat_zc { namespace sensor_msgs {
  bool fillImage(
      rti::flat::flat_type_traits<msg::Image>::builder & img_builder,
      const std::string& encoding_arg,
      uint32_t rows_arg,
      uint32_t cols_arg,
      uint32_t step_arg,
      const void* data_arg);

  std::string errorCodeToMessage(VmbErrorType error);

  bool frameToImage(
      rclcpp::Node * const node,
      const AVT::VmbAPI::FramePtr vimba_frame_ptr,
      rti::flat::flat_type_traits<msg::Image>::builder& img_builder);

  void buildCameraInfo(
    rti::flat::flat_type_traits<msg::CameraInfo>::builder & ci_builder,
    ::sensor_msgs::msg::CameraInfo & ros_msg);

}  // namespace sensor_msgs
}  // namespace flat_zc
}  // namespace ros2

#endif  // rti_image_helpers_hpp
