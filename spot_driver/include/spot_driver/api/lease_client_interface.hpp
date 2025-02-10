// Copyright (c) 2025 Boston Dynamics AI Institute LLC. All rights reserved.

#pragma once

#include <functional>
#include <memory>
#include <string>

#include <tl_expected/expected.hpp>

#include <bosdyn/api/lease.pb.h>
#include <bosdyn/client/lease/lease.h>
#include <bosdyn/client/lease/lease_wallet.h>

namespace spot_ros2 {
class LeaseClientInterface {
 public:
  using LeaseUseCallback = std::function<void(const bosdyn::api::LeaseUseResult&)>;

  // LeaseClientInterface is move-only
  LeaseClientInterface() = default;
  LeaseClientInterface(LeaseClientInterface&& other) = default;
  LeaseClientInterface(const LeaseClientInterface&) = delete;
  LeaseClientInterface& operator=(LeaseClientInterface&& other) = default;
  LeaseClientInterface& operator=(const LeaseClientInterface&) = delete;

  virtual ~LeaseClientInterface() = default;

  virtual tl::expected<::bosdyn::client::Lease, std::string> acquireLease(
      const std::string& resource_name, LeaseUseCallback retention_failure_callback) = 0;

  virtual tl::expected<::bosdyn::client::Lease, std::string> takeLease(const std::string& resource_name,
                                                                       LeaseUseCallback retention_failure_callback) = 0;

  virtual tl::expected<bool, std::string> returnLease(const ::bosdyn::client::Lease& lease) = 0;

  virtual std::shared_ptr<::bosdyn::client::LeaseWallet> getLeaseWallet() const = 0;

  virtual const ::bosdyn::client::ResourceHierarchy& getResourceHierarchy() const = 0;
};
}  // namespace spot_ros2
