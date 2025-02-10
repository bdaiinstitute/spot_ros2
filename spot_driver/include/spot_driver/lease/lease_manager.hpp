// Copyright (c) 2025 Boston Dynamics AI Institute LLC. All rights reserved.

#pragma once

#include <spot_driver/api/lease_client_interface.hpp>
#include <spot_driver/interfaces/logger_interface_base.hpp>

#include <spot_msgs/srv/acquire_lease.hpp>
#include <spot_msgs/srv/return_lease.hpp>

#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>

namespace spot_ros2::lease {

using spot_msgs::srv::AcquireLease;
using spot_msgs::srv::ReturnLease;

class LeaseManager {
 public:
  /**
   * This middleware handle is used to register a service and assign to it a
   * callback. In testing, it can be mocked to avoid using the ROS
   * infrastructure.
   */
  class MiddlewareHandle {
   public:
    class Bond {
     public:
      virtual ~Bond() = default;
    };

    virtual void createAcquireLeaseService(
        const std::string& service_name,
        std::function<void(const std::shared_ptr<AcquireLease::Request>, std::shared_ptr<AcquireLease::Response>)>
            callback) = 0;

    virtual void createReturnLeaseService(
        const std::string& service_name,
        std::function<void(const std::shared_ptr<ReturnLease::Request>, std::shared_ptr<ReturnLease::Response>)>
            callback) = 0;

    virtual std::shared_ptr<Bond> createBond(const std::string& node_name, std::function<void()> break_callback) = 0;

    virtual ~MiddlewareHandle() = default;
  };

  /**
   * Create the logic for the GetInverseKinematicSolutions service.
   * @param node The ROS node.
   * @param kinematic_api The Api to interact with the Spot SDK.
   * @param logger Logging interface.
   */
  explicit LeaseManager(std::shared_ptr<LeaseClientInterface> lease_client, std::shared_ptr<LoggerInterfaceBase> logger,
                        std::unique_ptr<MiddlewareHandle> middleware_handle);

  /** Initialize the service. */
  void initialize();

  /**
   * Invoke the Spot SDK to get IK solutions.
   * @param request The ROS request.
   * @param response A ROS response to be filled.
   */
  void acquireLease(const std::shared_ptr<AcquireLease::Request> request,
                    std::shared_ptr<AcquireLease::Response> response);

  void returnLease(const std::shared_ptr<ReturnLease::Request> request,
                   std::shared_ptr<ReturnLease::Response> response);

 private:
  void onLeaseRetentionFailure(const bosdyn::api::LeaseUseResult& result);

  // The API to interact with Spot SDK.
  std::shared_ptr<LeaseClientInterface> lease_client_;

  // Logger.
  std::shared_ptr<LoggerInterfaceBase> logger_;

  // The service provider.
  std::unique_ptr<MiddlewareHandle> middleware_handle_;

  struct ManagedSublease {
    bosdyn::client::Lease lease;
    std::shared_ptr<MiddlewareHandle::Bond> bond;
  };
  std::unordered_map<std::string, ManagedSublease> subleases_;
  std::recursive_mutex subleases_mutex_;
};
}  // namespace spot_ros2::lease
