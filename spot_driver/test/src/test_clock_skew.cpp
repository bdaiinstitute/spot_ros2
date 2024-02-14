// Copyright (c) 2024 Boston Dynamics AI Institute LLC. All rights reserved.

#include <gmock/gmock.h>
#include <google/protobuf/timestamp.pb.h>

#include <spot_driver/api/time_sync_api.hpp>

namespace spot_ros2::test {
TEST(TestClockSkew, ZeroSkew) {
  // GIVEN a positive timestamp and an all-zero clock skew
  google::protobuf::Timestamp timestamp;
  timestamp.set_seconds(1000);
  timestamp.set_nanos(500);
  google::protobuf::Duration clock_skew;
  clock_skew.set_seconds(0);
  clock_skew.set_nanos(0);

  // WHEN we apply the clock skew to the timestamp
  auto out = applyClockSkew(timestamp, clock_skew);

  // THEN the output timestamp is identical to the input timestamp
  EXPECT_THAT(out.sec, testing::Eq(1000));
  EXPECT_THAT(out.nanosec, testing::Eq(500));
}

TEST(TestClockSkew, PositiveSkew) {
  // GIVEN a postiive timestamp and a clock skew with positive seconds and nanoseconds fields
  google::protobuf::Timestamp timestamp;
  timestamp.set_seconds(1000);
  timestamp.set_nanos(500);
  google::protobuf::Duration clock_skew;
  clock_skew.set_seconds(100);
  clock_skew.set_nanos(10);

  // WHEN we apply the clock skew to the timestamp
  auto out = applyClockSkew(timestamp, clock_skew);

  // THEN the output timestamp is decreased by the values in the clock skew
  EXPECT_THAT(out.sec, testing::Eq(900));
  EXPECT_THAT(out.nanosec, testing::Eq(490));
}

TEST(TestClockSkew, NegativeSkew) {
  // GIVEN a postiive timestamp and a clock skew with negative seconds and nanoseconds fields
  google::protobuf::Timestamp timestamp;
  timestamp.set_seconds(1000);
  timestamp.set_nanos(500);
  google::protobuf::Duration clock_skew;
  clock_skew.set_seconds(-100);
  clock_skew.set_nanos(-10);

  // WHEN we apply the clock skew to the timestamp
  auto out = applyClockSkew(timestamp, clock_skew);

  // THEN the output timestamp is increased by the values in the clock skew
  EXPECT_THAT(out.sec, testing::Eq(1100));
  EXPECT_THAT(out.nanosec, testing::Eq(510));
}

TEST(TestClockSkew, PositiveSkewNanosecsCarry) {
  // GIVEN a positive timestamp and a clock skew where the number of seconds is zero and the number of nanoseconds in
  // the clock skew is greater than the number of nanoseconds in the timestamp
  google::protobuf::Timestamp timestamp;
  timestamp.set_seconds(1000);
  timestamp.set_nanos(100000000);
  google::protobuf::Duration clock_skew;
  clock_skew.set_seconds(0);
  clock_skew.set_nanos(200000000);

  // WHEN we apply the clock skew to the timestamp
  auto out = applyClockSkew(timestamp, clock_skew);

  // THEN a second is carried over into the nanoseconds field
  EXPECT_THAT(out.sec, testing::Eq(999));
  EXPECT_THAT(out.nanosec, testing::Eq(900000000));
}

TEST(TestClockSkew, NegativeSkewNanosecsCarry) {
  // GIVEN a positive timestamp and a clock skew where the number of seconds is zero and the number of nanoseconds in
  // the clock skew is negative and of sufficient magnitude to add to greater than one full second when summed with the
  // timetamp nanoseconds
  google::protobuf::Timestamp timestamp;
  timestamp.set_seconds(1000);
  timestamp.set_nanos(900000000);
  google::protobuf::Duration clock_skew;
  clock_skew.set_seconds(0);
  clock_skew.set_nanos(-200000000);

  // WHEN we apply the clock skew to the timestamp
  auto out = applyClockSkew(timestamp, clock_skew);

  // THEN a full second and the remainder of nanoseconds are added to the timestamp
  EXPECT_THAT(out.sec, testing::Eq(1001));
  EXPECT_THAT(out.nanosec, testing::Eq(100000000));
}

TEST(TestClockSkew, HandleNegativeInputTimestamp) {
  // GIVEN a timestamp with a negative number of seconds and an all-zero clock skew
  google::protobuf::Timestamp timestamp;
  timestamp.set_seconds(-1);
  timestamp.set_nanos(0);
  google::protobuf::Duration clock_skew;
  clock_skew.set_seconds(0);
  clock_skew.set_nanos(0);

  // WHEN we apply the clock skew to the timestamp
  auto out = applyClockSkew(timestamp, clock_skew);

  // THEN an all-zero timestamp is output
  EXPECT_THAT(out.sec, testing::Eq(0));
  EXPECT_THAT(out.nanosec, testing::Eq(0));
}

TEST(TestClockSkew, HandleNegativeUnskewedTimestamp) {
  // GIVEN a timestamp with a positive number of seconds and a positive clock skew with a greater number of seconds than
  // the timestamp
  google::protobuf::Timestamp timestamp;
  timestamp.set_seconds(1000);
  timestamp.set_nanos(0);
  google::protobuf::Duration clock_skew;
  clock_skew.set_seconds(1001);
  clock_skew.set_nanos(0);

  // WHEN we apply the clock skew to the timestamp
  auto out = applyClockSkew(timestamp, clock_skew);

  // THEN an all-zero timestamp is output
  EXPECT_THAT(out.sec, testing::Eq(0));
  EXPECT_THAT(out.nanosec, testing::Eq(0));
}

TEST(TestClockSkew, HandleNegativeUnskewedTimestampFromNanoseconds) {
  // GIVEN an all-zero timestamp and a positive clock skew with a non-zero number of nanoseconds
  google::protobuf::Timestamp timestamp;
  timestamp.set_seconds(0);
  timestamp.set_nanos(0);
  google::protobuf::Duration clock_skew;
  clock_skew.set_seconds(0);
  clock_skew.set_nanos(1);

  // WHEN we apply the clock skew to the timestamp
  auto out = applyClockSkew(timestamp, clock_skew);

  // THEN an all-zero timestamp is output
  EXPECT_THAT(out.sec, testing::Eq(0));
  EXPECT_THAT(out.nanosec, testing::Eq(0));
}
}  // namespace spot_ros2::test
