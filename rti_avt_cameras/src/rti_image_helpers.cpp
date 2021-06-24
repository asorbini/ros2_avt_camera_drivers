// (c) 2021 Copyright, Real-Time Innovations, Inc. (RTI)
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

#include "rti_avt_cameras/rti_image_helpers.hpp"

namespace ros2 { namespace flat_zc { namespace sensor_msgs {
  bool fillImage(
      rti::flat::flat_type_traits<msg::Image>::builder & img_builder,
      const std::string& encoding_arg,
      uint32_t rows_arg,
      uint32_t cols_arg,
      uint32_t step_arg,
      const void* data_arg)
  {
    img_builder.build_encoding().set_string(encoding_arg.c_str());
    img_builder.add_height(rows_arg);
    img_builder.add_width(cols_arg);
    img_builder.add_step(step_arg);
    img_builder.add_is_bigendian(0);
    size_t st0 = (step_arg * rows_arg);
    auto data_builder = img_builder.build_data();
    data_builder.add_n(static_cast<const uint8_t*>(data_arg), st0);
    data_builder.finish();
    return true;
  }

  /** Translates Vimba error codes to readable error messages
    *
    * @param error Vimba error tyme
    * @return readable string error
    *
    **/
  std::string errorCodeToMessage(VmbErrorType error) {
    std::map<VmbErrorType, std::string> error_msg;
    error_msg[VmbErrorSuccess]        = "Success.";
    error_msg[VmbErrorApiNotStarted]  = "API not started.";
    error_msg[VmbErrorNotFound]       = "Not found.";
    error_msg[VmbErrorBadHandle]      = "Invalid handle ";
    error_msg[VmbErrorDeviceNotOpen]  = "Device not open.";
    error_msg[VmbErrorInvalidAccess]  = "Invalid access.";
    error_msg[VmbErrorBadParameter]   = "Bad parameter.";
    error_msg[VmbErrorStructSize]     = "Wrong DLL version.";
    error_msg[VmbErrorWrongType]      = "Wrong type.";
    error_msg[VmbErrorInvalidValue]   = "Invalid value.";
    error_msg[VmbErrorTimeout]        = "Timeout.";
    error_msg[VmbErrorOther]          = "TL error.";
    error_msg[VmbErrorInvalidCall]    = "Invalid call.";
    error_msg[VmbErrorNoTL]           = "TL not loaded.";
    error_msg[VmbErrorNotImplemented] = "Not implemented.";
    error_msg[VmbErrorNotSupported]   = "Not supported.";
    error_msg[VmbErrorResources]      = "Resource not available.";
    error_msg[VmbErrorInternalFault]  = "Unexpected fault in VmbApi or driver.";
    error_msg[VmbErrorMoreData]       = "More data returned than memory provided.";

    std::map<VmbErrorType, std::string>::const_iterator iter =
      error_msg.find(error);
    if ( error_msg.end() != iter ) {
      return iter->second;
    }
    return "Unsupported error code passed.";
  }


  bool frameToImage(
      rclcpp::Node * const node,
      const AVT::VmbAPI::FramePtr vimba_frame_ptr,
      rti::flat::flat_type_traits<msg::Image>::builder& img_builder)
  {

    VmbPixelFormatType pixel_format;
    VmbUint32_t width, height, nSize;

    vimba_frame_ptr->GetWidth(width);
    vimba_frame_ptr->GetHeight(height);
    vimba_frame_ptr->GetPixelFormat(pixel_format);
    vimba_frame_ptr->GetImageSize(nSize);

    VmbUint32_t step = nSize / height;

    // NOTE: YUV and ARGB formats not supported
    std::string encoding;
    if      (pixel_format == VmbPixelFormatMono8          ) encoding = ::sensor_msgs::image_encodings::MONO8;
    else if (pixel_format == VmbPixelFormatMono10         ) encoding = ::sensor_msgs::image_encodings::MONO16;
    else if (pixel_format == VmbPixelFormatMono12         ) encoding = ::sensor_msgs::image_encodings::MONO16;
    else if (pixel_format == VmbPixelFormatMono12Packed   ) encoding = ::sensor_msgs::image_encodings::MONO16;
    else if (pixel_format == VmbPixelFormatMono14         ) encoding = ::sensor_msgs::image_encodings::MONO16;
    else if (pixel_format == VmbPixelFormatMono16         ) encoding = ::sensor_msgs::image_encodings::MONO16;
    else if (pixel_format == VmbPixelFormatBayerGR8       ) encoding = ::sensor_msgs::image_encodings::BAYER_GRBG8;
    else if (pixel_format == VmbPixelFormatBayerRG8       ) encoding = ::sensor_msgs::image_encodings::BAYER_RGGB8;
    else if (pixel_format == VmbPixelFormatBayerGB8       ) encoding = ::sensor_msgs::image_encodings::BAYER_GBRG8;
    else if (pixel_format == VmbPixelFormatBayerBG8       ) encoding = ::sensor_msgs::image_encodings::BAYER_BGGR8;
    else if (pixel_format == VmbPixelFormatBayerGR10      ) encoding = ::sensor_msgs::image_encodings::TYPE_16SC1;
    else if (pixel_format == VmbPixelFormatBayerRG10      ) encoding = ::sensor_msgs::image_encodings::TYPE_16SC1;
    else if (pixel_format == VmbPixelFormatBayerGB10      ) encoding = ::sensor_msgs::image_encodings::TYPE_16SC1;
    else if (pixel_format == VmbPixelFormatBayerBG10      ) encoding = ::sensor_msgs::image_encodings::TYPE_16SC1;
    else if (pixel_format == VmbPixelFormatBayerGR12      ) encoding = ::sensor_msgs::image_encodings::TYPE_16SC1;
    else if (pixel_format == VmbPixelFormatBayerRG12      ) encoding = ::sensor_msgs::image_encodings::TYPE_16SC1;
    else if (pixel_format == VmbPixelFormatBayerGB12      ) encoding = ::sensor_msgs::image_encodings::TYPE_16SC1;
    else if (pixel_format == VmbPixelFormatBayerBG12      ) encoding = ::sensor_msgs::image_encodings::TYPE_16SC1;
    else if (pixel_format == VmbPixelFormatBayerGR12Packed) encoding = ::sensor_msgs::image_encodings::TYPE_32SC4;
    else if (pixel_format == VmbPixelFormatBayerRG12Packed) encoding = ::sensor_msgs::image_encodings::TYPE_32SC4;
    else if (pixel_format == VmbPixelFormatBayerGB12Packed) encoding = ::sensor_msgs::image_encodings::TYPE_32SC4;
    else if (pixel_format == VmbPixelFormatBayerBG12Packed) encoding = ::sensor_msgs::image_encodings::TYPE_32SC4;
    else if (pixel_format == VmbPixelFormatBayerGR16      ) encoding = ::sensor_msgs::image_encodings::TYPE_16SC1;
    else if (pixel_format == VmbPixelFormatBayerRG16      ) encoding = ::sensor_msgs::image_encodings::TYPE_16SC1;
    else if (pixel_format == VmbPixelFormatBayerGB16      ) encoding = ::sensor_msgs::image_encodings::TYPE_16SC1;
    else if (pixel_format == VmbPixelFormatBayerBG16      ) encoding = ::sensor_msgs::image_encodings::TYPE_16SC1;
    else if (pixel_format == VmbPixelFormatRgb8           ) encoding = ::sensor_msgs::image_encodings::RGB8;
    else if (pixel_format == VmbPixelFormatBgr8           ) encoding = ::sensor_msgs::image_encodings::BGR8;
    else if (pixel_format == VmbPixelFormatRgba8          ) encoding = ::sensor_msgs::image_encodings::RGBA8;
    else if (pixel_format == VmbPixelFormatBgra8          ) encoding = ::sensor_msgs::image_encodings::BGRA8;
    else if (pixel_format == VmbPixelFormatRgb12          ) encoding = ::sensor_msgs::image_encodings::TYPE_16UC3;
    else if (pixel_format == VmbPixelFormatRgb16          ) encoding = ::sensor_msgs::image_encodings::TYPE_16UC3;
    else {
      RCLCPP_WARN_ONCE(node->get_logger(), "Received frame with unsupported pixel format %d", pixel_format);
    }
    if (encoding == "") return false;

    VmbUchar_t *buffer_ptr;
    VmbErrorType err = vimba_frame_ptr->GetImage(buffer_ptr);
    bool res = false;
    if ( VmbErrorSuccess == err ) {
      res = fillImage(img_builder,
                      encoding,
                      height,
                      width,
                      step,
                      buffer_ptr);
    } else {
      RCLCPP_ERROR_STREAM(node->get_logger(), "[" << "rclcpp::Node::get_name()"
        << "]: Could not GetImage. "
        << "\n Error: " << errorCodeToMessage(err));
    }
    return res;
  }


  void buildCameraInfo(
    rti::flat::flat_type_traits<msg::CameraInfo>::builder & ci_builder,
    ::sensor_msgs::msg::CameraInfo & ros_msg)
  {
    auto header_builder = ci_builder.build_header();
    auto time_builder = header_builder.build_stamp();
    time_builder.add_sec(ros_msg.header.stamp.sec);
    time_builder.add_nanosec(ros_msg.header.stamp.nanosec);
    time_builder.finish();
    header_builder.build_frame_id().set_string(ros_msg.header.frame_id.c_str());
    header_builder.finish();
    ci_builder.add_height(ros_msg.height);
    ci_builder.add_width(ros_msg.width);
    ci_builder.build_distortion_model().set_string(ros_msg.distortion_model.c_str());
    if (ros_msg.d.size() > 0) {
      auto d_builder = ci_builder.build_d();
      d_builder.add_n(ros_msg.d.data(), ros_msg.d.size());
      d_builder.finish();
    }
    memcpy(ci_builder.add_k().get_buffer(), ros_msg.k.data(), sizeof(ros_msg.k));
    memcpy(ci_builder.add_r().get_buffer(), ros_msg.r.data(), sizeof(ros_msg.r));
    memcpy(ci_builder.add_p().get_buffer(), ros_msg.p.data(), sizeof(ros_msg.p));

    ci_builder.add_binning_x(ros_msg.binning_x);
    ci_builder.add_binning_y(ros_msg.binning_y);

    auto roi_builder = ci_builder.build_roi();
    roi_builder.add_x_offset(ros_msg.roi.x_offset);
    roi_builder.add_y_offset(ros_msg.roi.y_offset);
    roi_builder.add_height(ros_msg.roi.height);
    roi_builder.add_width(ros_msg.roi.width);
    roi_builder.add_do_rectify(ros_msg.roi.do_rectify);
    roi_builder.finish();
  }

}  // namespace sensor_msgs
}  // namespace flat_zc
}  // namespace ros2
