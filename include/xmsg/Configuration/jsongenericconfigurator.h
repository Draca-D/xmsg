#ifndef JSONGENERICCONFIGURATOR_H
#define JSONGENERICCONFIGURATOR_H

#include <fstream>
#include <nlohmann/json.hpp>
#include <xmsg/Configuration/genericmessageconfigurator.h>

class JSONGenericConfigurator : public GenericMessageConfigurator {
public:
  JSONGenericConfigurator(const std::string &configuration_file);
  std::string get_mapped_name(const std::string &identifier,
                              const std::string &type,
                              const std::string &name) override;
  std::string get_transformation(const std::string &identifier,
                                 const std::string &type,
                                 const std::string &name) override;
  std::string check_if_child(const std::string &identifier,
                             const std::string &type,
                             const std::string &name) override;

private:
  nlohmann::json json_;
};

#endif // JSONGENERICCONFIGURATOR_H
