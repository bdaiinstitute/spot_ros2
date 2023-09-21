// Copyright (c) 2023 Boston Dynamics AI Institute LLC. All rights reserved.

#include <spot_driver_cpp/spot_image_sources.hpp>
#include <spot_driver_cpp/types.hpp>
#include <tl_expected/expected.hpp>

namespace
{
const std::vector<std::string> kImageSourceNames = {
    "back",
    "frontleft",
    "frontright",
    "left",
    "right",
};

constexpr auto kImageSourceHand = "hand";

constexpr auto kSuffixBodyColor = "_fisheye_image";
constexpr auto kSuffixHandColor = "_color_image";

constexpr auto kSuffixDepth = "_depth";
constexpr auto kSuffixDepthRegistered = "_depth_registered";

tl::expected<std::string, std::string> getBeforeSuffix(const std::string& string, const std::string& suffix)
{
    const auto index = string.find(suffix);
    if (index != std::string::npos)
    {
      return string.substr(0, index);
    }

    return tl::make_unexpected("Did not find substring `" + suffix + "` within string `" + string + "`.");
}
}

namespace spot_ros2
{
std::string toRosTopic(const ImageSource& image_source)
{
    if (image_source.type == SpotImageType::RGB)
    {
        return std::string("camera").append("/").append(image_source.name);
    }
    else if (image_source.type == SpotImageType::DEPTH)
    {
        return std::string("depth").append("/").append(image_source.name);
    }
    else  // SpotImageType::DEPTH_REGISTERED
    {
        return std::string("depth_registered").append("/").append(image_source.name);
    }
}

std::string toSpotImageSourceName(const ImageSource& image_source)
{
    if (image_source.type == SpotImageType::RGB)
    {
      if (image_source.name == kImageSourceHand)
      {
        return std::string(image_source.name).append(kSuffixHandColor);
      }
      else
      {
        return std::string(image_source.name).append(kSuffixBodyColor);
      }
    }
    else if (image_source.type == SpotImageType::DEPTH)
    {
        return std::string(image_source.name).append(kSuffixDepth);
    }
    else  // SpotImageType::DEPTH_REGISTERED
    {
        return std::string(image_source.name).append(kSuffixDepth);
    }
}

tl::expected<ImageSource, std::string> fromSpotImageSourceName(const std::string& source_name)
{
  if(const auto result = getBeforeSuffix(source_name, kSuffixBodyColor); result.has_value())
  {
    return ImageSource{result.value(), SpotImageType::RGB};
  }
  else if(const auto result = getBeforeSuffix(source_name, kSuffixHandColor); result.has_value())
  {
    return ImageSource{result.value(), SpotImageType::RGB};
  }
  else if(const auto result = getBeforeSuffix(source_name, kSuffixDepth); result.has_value())
  {
    return ImageSource{result.value(), SpotImageType::DEPTH};
  }
  else if(const auto result = getBeforeSuffix(source_name, kSuffixDepthRegistered); result.has_value())
  {
    return ImageSource{result.value(), SpotImageType::DEPTH_REGISTERED};
  }
  else
  {
    return tl::make_unexpected("Could not convert source name `" + source_name + "` to ImageSource.");
  }
}

std::vector<ImageSource> createImageSourcesList(const bool get_rgb_images, const bool get_depth_images, const bool get_depth_registered_images, const bool has_hand_camera)
{
    std::vector<ImageSource> sources;
    if (get_rgb_images)
    {
        for (const auto& name : kImageSourceNames)
        {
            sources.push_back(ImageSource{name, SpotImageType::RGB});
        }
        if (has_hand_camera)
        {
            sources.push_back(ImageSource{kImageSourceHand, SpotImageType::RGB});
        }
    }
    if (get_depth_images)
    {
        for (const auto& name : kImageSourceNames)
        {
            sources.push_back(ImageSource{name, SpotImageType::DEPTH});
        }
        if (has_hand_camera)
        {
            sources.push_back(ImageSource{kImageSourceHand, SpotImageType::DEPTH});
        }
    }
    if (get_depth_registered_images)
    {
        for (const auto& name : kImageSourceNames)
        {
            sources.push_back(ImageSource{name, SpotImageType::DEPTH_REGISTERED});
        }
        if (has_hand_camera)
        {
            sources.push_back(ImageSource{kImageSourceHand, SpotImageType::DEPTH_REGISTERED});
        }
    }

    return sources;
}
}
