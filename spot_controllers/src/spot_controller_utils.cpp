#include "spot_controllers/spot_controller_utils.hpp"

namespace spot_controllers {

std::string get_prefix_from_namespace(const std::string& node_namespace) {
  const std::string trimmed_namespace = node_namespace.substr(1);
  if (trimmed_namespace.empty()) {
    return "";
  } else {
    return trimmed_namespace + "/";
  }
}

}  // namespace spot_controllers
