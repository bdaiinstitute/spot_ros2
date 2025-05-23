#include <string>

#pragma once

namespace spot_controllers {

/// @brief Get the frame prefix from the namespace of the node.
/// If the namespace is "/", will return the empty string. Else, for "/<ns>" will return "<ns>".
/// @param node_namespace Namespace of the ROS node. Should be obtained via a `get_namespace()` call.
/// @return Appropriate frame prefix with the leading "/" trimmed off.
std::string get_prefix_from_namespace(const std::string& node_namespace);

}  // namespace spot_controllers
