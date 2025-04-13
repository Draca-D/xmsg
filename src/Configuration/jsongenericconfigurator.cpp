#include "xmsg/Configuration/jsongenericconfigurator.h"

#include <stdexcept>

JSONGenericConfigurator::JSONGenericConfigurator(
    const std::string &configuration_file) {
  std::ifstream ifs(configuration_file);
  if (!ifs.good()) {
    throw std::runtime_error("Could not read from file at " +
                             configuration_file);
  }
  json_ = nlohmann::json::parse(ifs);
}

std::string
JSONGenericConfigurator::get_mapped_name(const std::string &identifier,
                                         const std::string &type,
                                         const std::string &name) {
  if (json_.contains(identifier)) {
    if (json_[identifier].contains(type)) {
      if (json_[identifier][type].contains(name)) {
        if (json_[identifier][type][name].contains("transforms_to")) {
          return json_[identifier][type][name]["transforms_to"];
        }
      }
    }
  }

  return name;
}

std::string
JSONGenericConfigurator::get_transformation(const std::string &identifier,
                                            const std::string &type,
                                            const std::string &name) {
  if (json_.contains(identifier)) {
    if (json_[identifier].contains(type)) {
      if (json_[identifier][type].contains(name)) {
        if (json_[identifier][type][name].contains("process")) {
          return json_[identifier][type][name]["process"];
        }
      }
    }
  }

  return std::string();
}

std::string
JSONGenericConfigurator::check_if_child(const std::string &identifier,
                                        const std::string &type,
                                        const std::string &name) {
  if (json_.contains(identifier)) {
    if (json_[identifier].contains(type)) {
      if (json_[identifier][type].contains(name)) {
        if (json_[identifier][type][name].contains("set_child_instead")) {
          return json_[identifier][type][name]["set_child_instead"];
        }
      }
    }
  }

  return std::string();
}
