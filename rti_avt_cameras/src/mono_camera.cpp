/// Copyright (c) 2014,
/// Systems, Robotics and Vision Group
/// University of the Balearic Islands
/// All rights reserved.
///
/// Redistribution and use in source and binary forms, with or without
/// modification, are permitted provided that the following conditions are met:
///     * Redistributions of source code must retain the above copyright
///       notice, this list of conditions and the following disclaimer.
///     * Redistributions in binary form must reproduce the above copyright
///       notice, this list of conditions and the following disclaimer in the
///       documentation and/or other materials provided with the distribution.
///     * All advertising materials mentioning features or use of this software
///       must display the following acknowledgement:
///       This product includes software developed by
///       Systems, Robotics and Vision Group, Univ. of the Balearic Islands
///     * Neither the name of Systems, Robotics and Vision Group, University of
///       the Balearic Islands nor the names of its contributors may be used
///       to endorse or promote products derived from this software without
///       specific prior written permission.
///
/// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
/// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
/// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
/// ARE DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
/// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
/// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
/// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
/// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
/// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
/// THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#include "rti_avt_cameras/mono_camera.h"
#include "rclcpp/expand_topic_or_service_name.hpp"
#include "rcl/node.h"
#include "ros2dds/ros2dds.hpp"

namespace rti_avt_cameras {

void
MonoCamera::createPublisher() {
  using ros2::flat_zc::sensor_msgs::msg::Image;
  using ros2::flat_zc::sensor_msgs::msg::CameraInfo;

  writer_image_ = ros2dds::create_datawriter<Image>(*this, "image");

  std::string topic_name_info = getCameraInfoTopic(writer_image_.topic().name().c_str());
  writer_info_ = ros2dds::create_datawriter<CameraInfo>(*this, topic_name_info.c_str());
}

static std::vector<std::string> split(std::string input,
                               const std::string & delim)
{
  size_t pos = 0;
  std::vector<std::string> out;

  while ((pos = input.find(delim)) != std::string::npos) {
    auto token = input.substr(0, pos);
    if (token.size() > 0) {
      out.push_back(token);
    }
    input.erase(0, pos + delim.length());
  }
  out.push_back(input);

  return out;
}

std::string
MonoCamera::getCameraInfoTopic(const std::string & base_topic)
{
  std::string info_topic;
  auto tokens = split(base_topic, "/");

  if (tokens.size() > 0) {
    for(size_t ii = 0; ii < tokens.size() - 1; ++ii) {
      info_topic.append("/");
      info_topic.append(tokens[ii]);
    }
  }
  info_topic += "/camera_info";

  return info_topic;
}

void
MonoCamera::frameCallback(const FramePtr& vimba_frame_ptr) {
  rclcpp::Time ros_time = ros_clock_.now();
  if (dds::pub::matched_subscriptions(writer_image_).size() > 0) {
    auto img_builder = rti::flat::build_data(writer_image_);
    if (ros2::flat_zc::sensor_msgs::frameToImage(this, vimba_frame_ptr, img_builder)) {
      ::sensor_msgs::msg::CameraInfo ros_ci = info_man_->getCameraInfo();
      ros_ci.header.stamp = ros_time;
      auto hdr_builder = img_builder.build_header();
      auto stamp_builder = hdr_builder.build_stamp();
      stamp_builder.add_sec(ros_time.seconds());
      stamp_builder.add_nanosec(ros_time.nanoseconds());
      stamp_builder.finish();
      hdr_builder.build_frame_id().set_string(ros_ci.header.frame_id.c_str());
      writer_image_->write(*img_builder.finish_sample());

      auto ci_builder = rti::flat::build_data(writer_info_);
      ros2::flat_zc::sensor_msgs::buildCameraInfo(ci_builder, ros_ci);
      writer_info_->write(*ci_builder.finish_sample());
    } else {
      RCLCPP_WARN(this->get_logger(), "Function frameToImage returned 0. No image published.");
    }
  }
}

}  // namespace rti_avt_cameras
