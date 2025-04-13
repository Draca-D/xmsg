#ifndef GENERICMESSAGECONFIGURATOR_H
#define GENERICMESSAGECONFIGURATOR_H

#include <string>

class GenericMessageConfigurator {
public:
  virtual ~GenericMessageConfigurator() {}
  virtual std::string get_mapped_name(const std::string &identifier,
                                      const std::string &type,
                                      const std::string &name) = 0;
  virtual std::string get_transformation(const std::string &identifier,
                                         const std::string &type,
                                         const std::string &name) = 0;
  virtual std::string check_if_child(const std::string &identifier,
                                     const std::string &type,
                                     const std::string &name) = 0;

private:
protected:
  GenericMessageConfigurator();
};

#endif // GENERICMESSAGECONFIGURATOR_H
