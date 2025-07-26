#ifndef GENERICMESSAGE_H
#define GENERICMESSAGE_H

#include <regex>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <xmsg/Configuration/genericmessageconfigurator.h>
#include <xmsg/rawmemberwrapper.h>

namespace XMSG {
class GenericMessage;
class ProtoGenericMessage;
class JSONGenericMessage;
class BrokenMessage;
class ROSGenericMessage;

enum LogLevel { DEBUG, INFO, WARN, ERROR, FATAL };

enum OnceType { SHOWALL, ROS, UNIQUEONLY };

struct member_meta {
  std::string member_name;
  XMSG::Type type;
  std::string type_as_str;
  std::string type_identifier;
};

} // namespace XMSG

class XMSG::BrokenMessage : public std::runtime_error {
public:
  BrokenMessage(std::string message) : std::runtime_error(message) {}
  virtual ~BrokenMessage();
};

class XMSG::GenericMessage {
public:
  virtual ~GenericMessage() {}
  GenericMessage(const GenericMessage &msg);

  virtual const XMSG::RawMemberWrapper
  get_member(const std::string &member_name,
             const Type expected_type) const = 0;

  virtual void set_member(const std::string &member_name,
                          const XMSG::RawMemberWrapper &data) = 0;

  virtual GenericMessage &operator=(XMSG::GenericMessage &message);
  virtual void copy_from(const GenericMessage *const message);

  void set_strict(bool strict) noexcept {
    strict_ = strict;
  } // if strict compare ill require that any matches are exact. Non strict
    // ignores Capitals and '_', i.e., eveything goes to lowercase and
    // underscores are removed(on a local copy) prior to checking
  void log_members();

  std::string get_type() const noexcept { return type_; }
  std::string get_identifier() const noexcept { return identifier_; }

  std::vector<XMSG::member_meta> get_members() const noexcept {
    return members_;
  }

  bool is_formless() const noexcept { return formless_; }

  virtual GenericMessage *New(const std::string &type) const = 0;
  virtual GenericMessage *NewFormless() const { return nullptr; }

private:
  bool strict_ = false;
  bool formless_ = false; // useful for 'formatless' messages like json

  std::string type_;
  std::string identifier_;
  std::string helpful_name_;

  std::vector<XMSG::member_meta> members_;

  inline static std::vector<std::string> logged_messages_;

  inline static std::shared_ptr<GenericMessageConfigurator> configuration_;
  inline static bool auto_extract_child_ =
      false; // if member is base type, and new_member is generic type with
             // single child, automatically use that child
  inline static bool auto_set_child_ =
      false; // if member is generic type with single child member, and new
             // member is not of the same type, then use the child

private: // methods
  bool strict_compare(const std::string &str1,
                      const std::string &str2) const noexcept {
    return (str1.compare(str2) == 0);
  }
  bool lax_compare(const std::string &str1,
                   const std::string &str2) const noexcept;
  bool is_transformable(const XMSG::RawMemberWrapper &member) const noexcept;
  bool has_member(const std::string &member_name) const noexcept;
  bool is_simple() {
    return get_member_count() == 1;
  } // a simple generic type is one that only contains a single child member

  std::string
  all_members_to_str(int depth); // iterate through all members and return an
                                 // indented string outline names and types
  std::string get_mapped_name(const std::string &member_name);
  std::string get_transformation(const std::string &member_name);
  std::string access_requested_child_member(const std::string &member_name);

  XMSG::RawMemberWrapper transform_member(XMSG::RawMemberWrapper &member,
                                          const std::string &transformation);

  GenericMessage *create_single_child_populated_msg(
      const std::string &type_id, const std::string &name,
      const std::string &child, XMSG::RawMemberWrapper &new_member);
  std::vector<GenericMessage *> *create_single_child_populated_msg_array(
      const std::string &type_id, const std::string &name,
      const std::string &child, XMSG::RawMemberWrapper &new_member);

  XMSG::Type get_member_type(const std::string &name);

  size_t get_member_count();

protected:
  GenericMessage() {}

  std::string to_lower(std::string str) const noexcept;
  std::string to_upper(std::string str) const noexcept;
  std::string remove_character(std::string str, char c) const noexcept;

  void set_type(const std::string &type) { type_ = type; }
  void set_identifier(const std::string &id) { identifier_ = id; }
  void set_helpful_name(const std::string &name) { helpful_name_ = name; }

  void log(const std::string &message, XMSG::LogLevel level,
           OnceType once = XMSG::OnceType::SHOWALL);
  void add_member(const std::string member, XMSG::Type type,
                  const std::string &type_identifier);

  void set_formless(bool formless) { formless_ = formless; }

  bool compare(const std::string &str1, const std::string &str2) const noexcept;

  std::string type_to_str(XMSG::Type type);

  RawMemberWrapper build_wrapper(void *data, const Type &type) const noexcept;

public: // statics
  static void set_configuration_engine(
      std::shared_ptr<GenericMessageConfigurator> configuration) {
    configuration_ = configuration;
  }
  static void set_auto_extract_child(bool extract) {
    auto_extract_child_ = extract;
  }
  static void set_auto_set_child(bool set) { auto_set_child_ = set; }
  static void auto_perform_simple_translation() {
    auto_extract_child_ = true;
    auto_set_child_ = true;
  }
};

#endif // GENERICMESSAGE_H
