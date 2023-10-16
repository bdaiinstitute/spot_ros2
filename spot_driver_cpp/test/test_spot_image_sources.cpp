// Copyright (c) 2023 Boston Dynamics AI Institute LLC. All rights reserved.

#include <gmock/gmock.h>

#include <spot_driver_cpp/spot_image_sources.hpp>
#include <spot_driver_cpp/types.hpp>

namespace
{
  using ::testing::IsEmpty;
  using ::testing::StrEq;
  using ::testing::UnorderedElementsAre;
}

namespace spot_ros2
{
TEST(SpotImageSources, toRosTopic)
{
  // Check that all combinations of SpotCamera and SpotImageType are converted to the correct topic name

  EXPECT_THAT(toRosTopic(ImageSource{SpotCamera::FRONTLEFT, SpotImageType::RGB}), StrEq("camera/frontleft"));
  EXPECT_THAT(toRosTopic(ImageSource{SpotCamera::FRONTRIGHT, SpotImageType::RGB}), StrEq("camera/frontright"));
  EXPECT_THAT(toRosTopic(ImageSource{SpotCamera::BACK, SpotImageType::RGB}), StrEq("camera/back"));
  EXPECT_THAT(toRosTopic(ImageSource{SpotCamera::HAND, SpotImageType::RGB}), StrEq("camera/hand"));
  EXPECT_THAT(toRosTopic(ImageSource{SpotCamera::LEFT, SpotImageType::RGB}), StrEq("camera/left"));
  EXPECT_THAT(toRosTopic(ImageSource{SpotCamera::RIGHT, SpotImageType::RGB}), StrEq("camera/right"));

  EXPECT_THAT(toRosTopic(ImageSource{SpotCamera::FRONTLEFT, SpotImageType::DEPTH}), StrEq("depth/frontleft"));
  EXPECT_THAT(toRosTopic(ImageSource{SpotCamera::FRONTRIGHT, SpotImageType::DEPTH}), StrEq("depth/frontright"));
  EXPECT_THAT(toRosTopic(ImageSource{SpotCamera::BACK, SpotImageType::DEPTH}), StrEq("depth/back"));
  EXPECT_THAT(toRosTopic(ImageSource{SpotCamera::HAND, SpotImageType::DEPTH}), StrEq("depth/hand"));
  EXPECT_THAT(toRosTopic(ImageSource{SpotCamera::LEFT, SpotImageType::DEPTH}), StrEq("depth/left"));
  EXPECT_THAT(toRosTopic(ImageSource{SpotCamera::RIGHT, SpotImageType::DEPTH}), StrEq("depth/right"));

  EXPECT_THAT(toRosTopic(ImageSource{SpotCamera::FRONTLEFT, SpotImageType::DEPTH_REGISTERED}), StrEq("depth_registered/frontleft"));
  EXPECT_THAT(toRosTopic(ImageSource{SpotCamera::FRONTRIGHT, SpotImageType::DEPTH_REGISTERED}), StrEq("depth_registered/frontright"));
  EXPECT_THAT(toRosTopic(ImageSource{SpotCamera::BACK, SpotImageType::DEPTH_REGISTERED}), StrEq("depth_registered/back"));
  EXPECT_THAT(toRosTopic(ImageSource{SpotCamera::HAND, SpotImageType::DEPTH_REGISTERED}), StrEq("depth_registered/hand"));
  EXPECT_THAT(toRosTopic(ImageSource{SpotCamera::LEFT, SpotImageType::DEPTH_REGISTERED}), StrEq("depth_registered/left"));
  EXPECT_THAT(toRosTopic(ImageSource{SpotCamera::RIGHT, SpotImageType::DEPTH_REGISTERED}), StrEq("depth_registered/right"));
}

TEST(SpotImageSources, toSpotImageSourceName)
{
  // Check that all combinations of SpotCamera and SpotImageType are converted to the correct Spot API source name

  EXPECT_THAT(toSpotImageSourceName(ImageSource{SpotCamera::FRONTLEFT, SpotImageType::RGB}), StrEq("frontleft_fisheye_image"));
  EXPECT_THAT(toSpotImageSourceName(ImageSource{SpotCamera::FRONTRIGHT, SpotImageType::RGB}), StrEq("frontright_fisheye_image"));
  EXPECT_THAT(toSpotImageSourceName(ImageSource{SpotCamera::BACK, SpotImageType::RGB}), StrEq("back_fisheye_image"));
  EXPECT_THAT(toSpotImageSourceName(ImageSource{SpotCamera::LEFT, SpotImageType::RGB}), StrEq("left_fisheye_image"));
  EXPECT_THAT(toSpotImageSourceName(ImageSource{SpotCamera::RIGHT, SpotImageType::RGB}), StrEq("right_fisheye_image"));

  EXPECT_THAT(toSpotImageSourceName(ImageSource{SpotCamera::FRONTLEFT, SpotImageType::DEPTH}), StrEq("frontleft_depth"));
  EXPECT_THAT(toSpotImageSourceName(ImageSource{SpotCamera::FRONTRIGHT, SpotImageType::DEPTH}), StrEq("frontright_depth"));
  EXPECT_THAT(toSpotImageSourceName(ImageSource{SpotCamera::BACK, SpotImageType::DEPTH}), StrEq("back_depth"));
  EXPECT_THAT(toSpotImageSourceName(ImageSource{SpotCamera::LEFT, SpotImageType::DEPTH}), StrEq("left_depth"));
  EXPECT_THAT(toSpotImageSourceName(ImageSource{SpotCamera::RIGHT, SpotImageType::DEPTH}), StrEq("right_depth"));

  EXPECT_THAT(toSpotImageSourceName(ImageSource{SpotCamera::FRONTLEFT, SpotImageType::DEPTH_REGISTERED}), StrEq("frontleft_depth_in_visual_frame"));
  EXPECT_THAT(toSpotImageSourceName(ImageSource{SpotCamera::FRONTRIGHT, SpotImageType::DEPTH_REGISTERED}), StrEq("frontright_depth_in_visual_frame"));
  EXPECT_THAT(toSpotImageSourceName(ImageSource{SpotCamera::BACK, SpotImageType::DEPTH_REGISTERED}), StrEq("back_depth_in_visual_frame"));
  EXPECT_THAT(toSpotImageSourceName(ImageSource{SpotCamera::LEFT, SpotImageType::DEPTH_REGISTERED}), StrEq("left_depth_in_visual_frame"));
  EXPECT_THAT(toSpotImageSourceName(ImageSource{SpotCamera::RIGHT, SpotImageType::DEPTH_REGISTERED}), StrEq("right_depth_in_visual_frame"));

  // Note that the Spot API uses a different naming convention for the hand images than for the body images.
  EXPECT_THAT(toSpotImageSourceName(ImageSource{SpotCamera::HAND, SpotImageType::RGB}), StrEq("hand_color_image"));
  EXPECT_THAT(toSpotImageSourceName(ImageSource{SpotCamera::HAND, SpotImageType::DEPTH}), StrEq("hand_depth"));
  EXPECT_THAT(toSpotImageSourceName(ImageSource{SpotCamera::HAND, SpotImageType::DEPTH_REGISTERED}), StrEq("hand_depth_in_hand_color_frame"));
}

TEST(SpotImageSources, createImageSourcesList)
{
  // WHEN no image types are requested, regardless of whether or not the hand camera is requested
  // THEN no ImageSources are returned
  EXPECT_THAT(createImageSourcesList(false, false, false, false), IsEmpty());
  EXPECT_THAT(createImageSourcesList(false, false, false, true), IsEmpty());

  // WHEN RGB images are requested from the body cameras, excluding the hand camera
  // THEN image sources for all RGB body cameras are returned
  EXPECT_THAT(createImageSourcesList(true, false, false, false),
    UnorderedElementsAre(
      ImageSource{SpotCamera::FRONTLEFT, SpotImageType::RGB},
      ImageSource{SpotCamera::FRONTRIGHT, SpotImageType::RGB},
      ImageSource{SpotCamera::BACK, SpotImageType::RGB},
      ImageSource{SpotCamera::LEFT, SpotImageType::RGB},
      ImageSource{SpotCamera::RIGHT, SpotImageType::RGB}
    )
  );

  // WHEN RGB images are requested from the body cameras and the hand camera
  // THEN image sources for all RGB body cameras and the hand camera are returned
  EXPECT_THAT(createImageSourcesList(true, false, false, true),
    UnorderedElementsAre(
      ImageSource{SpotCamera::FRONTLEFT, SpotImageType::RGB},
      ImageSource{SpotCamera::FRONTRIGHT, SpotImageType::RGB},
      ImageSource{SpotCamera::BACK, SpotImageType::RGB},
      ImageSource{SpotCamera::LEFT, SpotImageType::RGB},
      ImageSource{SpotCamera::RIGHT, SpotImageType::RGB},
      ImageSource{SpotCamera::HAND, SpotImageType::RGB}
    )
  );

  // WHEN depth images are requested from the body cameras, excluding the hand camera
  // THEN image sources for all depth body cameras are returned
  EXPECT_THAT(createImageSourcesList(false, true, false, false),
    UnorderedElementsAre(
      ImageSource{SpotCamera::FRONTLEFT, SpotImageType::DEPTH},
      ImageSource{SpotCamera::FRONTRIGHT, SpotImageType::DEPTH},
      ImageSource{SpotCamera::BACK, SpotImageType::DEPTH},
      ImageSource{SpotCamera::LEFT, SpotImageType::DEPTH},
      ImageSource{SpotCamera::RIGHT, SpotImageType::DEPTH}
    )
  );

  // WHEN depth images are requested from the body cameras and the hand camera
  // THEN image sources for all depth body cameras and the hand camera are returned
  EXPECT_THAT(createImageSourcesList(false, true, false, true),
    UnorderedElementsAre(
      ImageSource{SpotCamera::FRONTLEFT, SpotImageType::DEPTH},
      ImageSource{SpotCamera::FRONTRIGHT, SpotImageType::DEPTH},
      ImageSource{SpotCamera::BACK, SpotImageType::DEPTH},
      ImageSource{SpotCamera::LEFT, SpotImageType::DEPTH},
      ImageSource{SpotCamera::RIGHT, SpotImageType::DEPTH},
      ImageSource{SpotCamera::HAND, SpotImageType::DEPTH}
    )
  );

  // WHEN registered depth images are requested from the body cameras, excluding the hand camera
  // THEN image sources for all registered depth body cameras are returned
  EXPECT_THAT(createImageSourcesList(false, false, true, false),
    UnorderedElementsAre(
      ImageSource{SpotCamera::FRONTLEFT, SpotImageType::DEPTH_REGISTERED},
      ImageSource{SpotCamera::FRONTRIGHT, SpotImageType::DEPTH_REGISTERED},
      ImageSource{SpotCamera::BACK, SpotImageType::DEPTH_REGISTERED},
      ImageSource{SpotCamera::LEFT, SpotImageType::DEPTH_REGISTERED},
      ImageSource{SpotCamera::RIGHT, SpotImageType::DEPTH_REGISTERED}
    )
  );

  // WHEN registered depth images are requested from the body cameras and the hand camera
  // THEN image sources for all registered depth body cameras and the hand camera are returned
  EXPECT_THAT(createImageSourcesList(false, false, true, true),
    UnorderedElementsAre(
      ImageSource{SpotCamera::FRONTLEFT, SpotImageType::DEPTH_REGISTERED},
      ImageSource{SpotCamera::FRONTRIGHT, SpotImageType::DEPTH_REGISTERED},
      ImageSource{SpotCamera::BACK, SpotImageType::DEPTH_REGISTERED},
      ImageSource{SpotCamera::LEFT, SpotImageType::DEPTH_REGISTERED},
      ImageSource{SpotCamera::RIGHT, SpotImageType::DEPTH_REGISTERED},
      ImageSource{SpotCamera::HAND, SpotImageType::DEPTH_REGISTERED}
    )
  );

  // WHEN RGB, depth, and registered images are requested from the body cameras, excluding the hand camera
  // THEN image sources for all camera types and all body cameras are returned
  EXPECT_THAT(createImageSourcesList(true, true, true, false),
    UnorderedElementsAre(
      ImageSource{SpotCamera::FRONTLEFT, SpotImageType::RGB},
      ImageSource{SpotCamera::FRONTRIGHT, SpotImageType::RGB},
      ImageSource{SpotCamera::BACK, SpotImageType::RGB},
      ImageSource{SpotCamera::LEFT, SpotImageType::RGB},
      ImageSource{SpotCamera::RIGHT, SpotImageType::RGB},
      ImageSource{SpotCamera::FRONTLEFT, SpotImageType::DEPTH},
      ImageSource{SpotCamera::FRONTRIGHT, SpotImageType::DEPTH},
      ImageSource{SpotCamera::BACK, SpotImageType::DEPTH},
      ImageSource{SpotCamera::LEFT, SpotImageType::DEPTH},
      ImageSource{SpotCamera::RIGHT, SpotImageType::DEPTH},
      ImageSource{SpotCamera::FRONTLEFT, SpotImageType::DEPTH_REGISTERED},
      ImageSource{SpotCamera::FRONTRIGHT, SpotImageType::DEPTH_REGISTERED},
      ImageSource{SpotCamera::BACK, SpotImageType::DEPTH_REGISTERED},
      ImageSource{SpotCamera::LEFT, SpotImageType::DEPTH_REGISTERED},
      ImageSource{SpotCamera::RIGHT, SpotImageType::DEPTH_REGISTERED}
    )
  );

  // WHEN RGB, depth, and registered images are requested from the body cameras and the hand camera
  // THEN image sources for all camera types and all body cameras (plus the hand camera) are returned
  EXPECT_THAT(createImageSourcesList(true, true, true, true),
    UnorderedElementsAre(
      ImageSource{SpotCamera::FRONTLEFT, SpotImageType::RGB},
      ImageSource{SpotCamera::FRONTRIGHT, SpotImageType::RGB},
      ImageSource{SpotCamera::BACK, SpotImageType::RGB},
      ImageSource{SpotCamera::LEFT, SpotImageType::RGB},
      ImageSource{SpotCamera::RIGHT, SpotImageType::RGB},
      ImageSource{SpotCamera::HAND, SpotImageType::RGB},
      ImageSource{SpotCamera::FRONTLEFT, SpotImageType::DEPTH},
      ImageSource{SpotCamera::FRONTRIGHT, SpotImageType::DEPTH},
      ImageSource{SpotCamera::BACK, SpotImageType::DEPTH},
      ImageSource{SpotCamera::LEFT, SpotImageType::DEPTH},
      ImageSource{SpotCamera::RIGHT, SpotImageType::DEPTH},
      ImageSource{SpotCamera::HAND, SpotImageType::DEPTH},
      ImageSource{SpotCamera::FRONTLEFT, SpotImageType::DEPTH_REGISTERED},
      ImageSource{SpotCamera::FRONTRIGHT, SpotImageType::DEPTH_REGISTERED},
      ImageSource{SpotCamera::BACK, SpotImageType::DEPTH_REGISTERED},
      ImageSource{SpotCamera::LEFT, SpotImageType::DEPTH_REGISTERED},
      ImageSource{SpotCamera::RIGHT, SpotImageType::DEPTH_REGISTERED},
      ImageSource{SpotCamera::HAND, SpotImageType::DEPTH_REGISTERED}
    )
  );
}
}
