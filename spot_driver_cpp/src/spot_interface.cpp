// Copyright (c) 2023 Boston Dynamics AI Institute LLC. All rights reserved.

#include "spot_driver_cpp/spot_image_sources.hpp"
#include <spot_driver_cpp/spot_interface.hpp>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/hal/interface.h>
#include <opencv2/imgcodecs.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>
#include <tl_expected/expected.hpp>

#include <iostream>
#include <stdexcept>
#include <utility>

namespace
{
tl::expected<int, std::string> getCvPixelFormat(const bosdyn::api::Image_PixelFormat& format)
{
  switch(format)
  {
    case bosdyn::api::Image_PixelFormat_PIXEL_FORMAT_RGB_U8:
    {
      return CV_8UC3;
    }
    case bosdyn::api::Image_PixelFormat_PIXEL_FORMAT_RGBA_U8:
    {
      return CV_8UC4;
    }
    case bosdyn::api::Image_PixelFormat_PIXEL_FORMAT_GREYSCALE_U8:
    {
      return CV_8UC1;
    }
    case bosdyn::api::Image_PixelFormat_PIXEL_FORMAT_GREYSCALE_U16:
    {
      return CV_16UC1;
    }
    case bosdyn::api::Image_PixelFormat_PIXEL_FORMAT_DEPTH_U16:
    {
      return CV_16UC1;
    }
    default:
    {
      return tl::make_unexpected("Unknown pixel format.");
    }
  }
}

tl::expected<sensor_msgs::msg::Image, std::string> toImageMsg(const bosdyn::api::ImageCapture& image_capture)
{
      const auto& image = image_capture.image();
      auto data = image.data();
      const auto& timestamp = image_capture.acquisition_time();

      std_msgs::msg::Header header;
      header.frame_id = image_capture.frame_name_image_sensor();
      header.stamp.sec = timestamp.seconds();
      header.stamp.nanosec = timestamp.nanos();

      const auto pixel_format_cv = getCvPixelFormat(image.pixel_format());
      if (!pixel_format_cv)
      {
        return tl::make_unexpected("Failed to convert image to message: " + pixel_format_cv.error());
      }

      if (image.format() == bosdyn::api::Image_Format_FORMAT_JPEG)
      {
        // When the image is JPEG-compressed, it is represented as a 1 x (width * height) row of bytes.
        // First we create a cv::Mat which contains the compressed image data...
        const cv::Mat img_compressed{ 1, image.rows() * image.cols(), CV_8UC1, &data.front()};
        // Then we decode it to extract the raw image into a new cv::Mat.
        // Note: this assumes that if an image is provided as JPEG-compressed data, then it is an RGB image.
        const cv::Mat img_bgr = cv::imdecode(img_compressed, cv::IMREAD_COLOR);
        if (!img_bgr.data)
        {
          return tl::make_unexpected("Failed to decode JPEG-compressed image.");
        }
        const auto image = cv_bridge::CvImage{header, "bgr8", img_bgr}.toImageMsg();
        return *image;
      }
      else if (image.format() == bosdyn::api::Image_Format_FORMAT_RAW)
      {
        // Note: as currently implemented, this assumes that the only images which will be provided as raw data will be 16UC1 depth images.
        // TODO: handle converting raw RGB and grayscale images as well.
        const cv::Mat img = cv::Mat(image.rows(), image.cols(), pixel_format_cv.value(), &data.front());
        const auto image = cv_bridge::CvImage{header, "mono16", img}.toImageMsg();
        return *image;
      }
      else if (image.format() == bosdyn::api::Image_Format_FORMAT_RLE)
      {
        return tl::make_unexpected("Conversion from FORMAT_RLE is not yet implemented.");
      }
      else
      {
        return tl::make_unexpected("Unknown image format.");
      }
}
}

namespace spot_ros2
{
SpotInterface::SpotInterface()
: client_sdk_{ ::bosdyn::client::CreateStandardSDK("get_image") }
{
}

bool SpotInterface::createRobot(const std::string& ip_address)
{
  auto create_robot_result = client_sdk_->CreateRobot(ip_address);
  if(!create_robot_result.status)
  {
    return false;
  }

  robot_ = std::move(create_robot_result.response);

  return true;
}

bool SpotInterface::authenticate(const std::string& username, const std::string& password)
{
  if (!robot_)
  {
    return false;
  }

  const auto authenticate_result = robot_->Authenticate(username, password);
  if(!authenticate_result)
  {
    return false;
  }

  const auto image_client_result =
        robot_->EnsureServiceClient<::bosdyn::client::ImageClient>(
            ::bosdyn::client::ImageClient::GetDefaultServiceName());
  if (!image_client_result.status)
  {
    return false;
  }

  image_client_.reset(std::move(image_client_result.response));

  return true;
}

bool SpotInterface::hasArm() const
{
  // TODO: programmatically determine if Spot has an arm attached, like the existing Python driver does
  return true;
}

tl::expected<GetImagesResult, std::string> SpotInterface::getImages(::bosdyn::api::GetImageRequest request)
{
  std::shared_future<::bosdyn::client::GetImageResultType> get_image_result_future = image_client_->GetImageAsync(request);

  ::bosdyn::client::GetImageResultType get_image_result = get_image_result_future.get();
  if (!get_image_result.status) {
      return tl::make_unexpected("Failed to get images: " + get_image_result.status.DebugString());
  }

  GetImagesResult out;
  for (const auto& image_response : get_image_result.response.image_responses())
  {      
      const auto& image = image_response.shot().image();
      auto data = image.data();

      if (const auto image_msg = toImageMsg(image_response.shot()); image_msg.has_value())
      {
        const auto& camera_name = image_response.source().name();
        if(const auto result = fromSpotImageSourceName(camera_name); result.has_value())
        {
          out.try_emplace(result.value(), image_msg.value());
        }
        else
        {
          std::cerr << "Failed to convert API image source name to ImageSource." << std::endl;
        }
      }
      else
      {
        // If an image cannot be converted, log an error message but continue processing the remaining images.
        std::cerr << image_msg.error() << std::endl;
      }
  }

  return out;
}

}
