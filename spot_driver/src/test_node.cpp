//Credit goes to Katie for the code below! 

#include "bosdyn/client/sdk/client_sdk.h"


int main(int argc, char** argv) {
    const std::string ip_address = "127.0.0.1";
    const std::string sdk_client_name = "foo";
    // std::unique_ptr<::bosdyn::client::ClientSdk> client_sdk_ = std::make_unique<::bosdyn::client::ClientSdk>();
    std::unique_ptr<::bosdyn::client::ClientSdk> client_sdk_ = ::bosdyn::client::CreateStandardSDK(sdk_client_name);
    auto create_robot_result = client_sdk_->CreateRobot(ip_address, ::bosdyn::client::USE_PROXY);
    if (!create_robot_result.status) {
        std::cout << create_robot_result.status.DebugString() << std::endl;
    } else {
        std::cout << "success" << std::endl;
    }
}
