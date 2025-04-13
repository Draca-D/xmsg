#ifndef GENERICMESSAGE_H
#define GENERICMESSAGE_H

#include <regex>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <xmsg/Configuration/genericmessageconfigurator.h>

namespace XMSG {
class GenericMessage;
class ProtoGenericMessage;
class JSONGenericMessage;
class BrokenMessage;
class ROSGenericMessage;

enum Type {
  NONE = 0,
  ARRAY = 1,
  CHAR = 2,
  UINT = 4,
  INT = 8,
  DOUBLE = 16,
  STRING = 32,
  BOOL = 64,
  GENERIC = 128,
  NOTSET = 256
};

enum LogLevel { DEBUG, INFO, WARN, ERROR, FATAL };

enum OnceType { SHOWALL, ROS, UNIQUEONLY };

struct member {
  void *data;
  XMSG::Type type;
};

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

  virtual const XMSG::member get_member(const std::string &member_name,
                                        const Type expected_type) = 0;
  virtual void set_member(const std::string &member_name,
                          XMSG::member data) = 0;

  // If the member does not exist, or if its an invalid type, an empty optional
  // will be returned.
  // Note: This will attempt type coercion where it makes sense, i.e., if you
  // request a float, and the underlying type is an int, it will coerce the int
  // to a float. If it isnt possible, an empty optional will be returned.
  // Coercion will be done as a static cast, so check docs for any pitfalls
  template <typename RET_TYPE>
  std::optional<RET_TYPE> get(const std::string &member_name)
      const; // ideally std::optional will be replaced with std::expected, but
             // ros2 humble is at c++17 so want to keep that as a minimum
             // requirement

  virtual void operator=(XMSG::GenericMessage &message);
  virtual void copy_from(XMSG::GenericMessage *message);

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
  bool is_transformable(const XMSG::member &member) const noexcept;
  bool has_member(const std::string &member_name) const noexcept;
  bool is_simple() {
    return get_member_count() == 1;
  } // a simple generic type is one that only contains a single child member

  void delete_array(void *data, XMSG::Type type);
  void delete_member(void *data, XMSG::Type type);

  std::string
  all_members_to_str(int depth); // iterate through all members and return an
                                 // indented string outline names and types
  std::string get_mapped_name(const std::string &member_name);
  std::string get_transformation(const std::string &member_name);
  std::string access_requested_child_member(const std::string &member_name);

  XMSG::member transform_member(XMSG::member &member,
                                const std::string &transformation);

  GenericMessage *create_single_child_populated_msg(const std::string &type_id,
                                                    const std::string &name,
                                                    const std::string &child,
                                                    member &new_member);
  std::vector<GenericMessage *> *create_single_child_populated_msg_array(
      const std::string &type_id, const std::string &name,
      const std::string &child, member &new_member);

  XMSG::Type get_member_type(const std::string &name);

  size_t get_member_count();

protected:
  GenericMessage() {}

  std::string to_lower(std::string str) const noexcept;
  std::string to_upper(std::string str) const noexcept;
  std::string remove_character(std::string str, char c) const noexcept;

  bool is_integer_type(XMSG::Type type) const noexcept; // int uint
  bool is_numerical_type(
      XMSG::Type type) const noexcept; // int uint double bool char

  void set_type(const std::string &type) { type_ = type; }
  void set_identifier(const std::string &id) { identifier_ = id; }
  void set_helpful_name(const std::string &name) { helpful_name_ = name; }
  void delete_member(XMSG::member &member);

  void log(const std::string &message, XMSG::LogLevel level,
           OnceType once = XMSG::OnceType::SHOWALL);
  void add_member(const std::string member, XMSG::Type type,
                  const std::string &type_identifier);

  void modify_type(XMSG::member &member, XMSG::Type new_type);

  void set_formless(bool formless) { formless_ = formless; }

  template <typename undertype, typename basetype>
  basetype extract_and_free(void *data) {
    basetype x = static_cast<basetype>(*static_cast<undertype *>(data));
    free(data);

    return x;
  }

  template <typename undertype, typename basetype>
  std::vector<basetype> extract_and_free_array(void *data) {
    std::vector<basetype> x = static_cast<std::vector<basetype>>(
        *static_cast<std::vector<undertype> *>(data));
    delete static_cast<std::vector<undertype> *>(data);
    return x;
  }

  template <typename undertype, typename basetype>
  void assign_and_set(void *&data, basetype new_data) {
    data = calloc(1, sizeof(undertype));
    static_cast<undertype *>(data)[0] = static_cast<undertype>(new_data);
  }

  template <typename undertype, typename basetype>
  void assign_and_set_array(void *&data, std::vector<basetype> &new_data) {
    data = new std::vector<undertype>;

    for (basetype nd : new_data) {
      static_cast<std::vector<undertype> *>(data)->push_back(
          static_cast<undertype>(nd));
    }
  }

  bool compare(const std::string &str1, const std::string &str2) const noexcept;

  virtual GenericMessage *New(const std::string &type) = 0;
  virtual GenericMessage *NewFormless() { return nullptr; }

  std::string type_to_str(XMSG::Type type);

protected: // in header methods // may be long
  template <typename ty>
  void set_new_type(XMSG::member &member, XMSG::Type new_type, ty x) {
    auto type = new_type;

    if (type & XMSG::Type::CHAR) {
      assign_and_set<int64_t, ty>(member.data, x);
    } else if (type & XMSG::Type::INT) {
      assign_and_set<int64_t, ty>(member.data, x);
    } else if (type & XMSG::Type::UINT) {
      assign_and_set<uint64_t, ty>(member.data, x);
    } else if (type & XMSG::Type::DOUBLE) {
      assign_and_set<double, ty>(member.data, x);
    } else if (type & XMSG::Type::BOOL) {
      assign_and_set<bool, ty>(member.data, x);
    } else {
      throw std::invalid_argument("not a settable type");
    }
  }

  template <typename ty>
  void set_new_array(XMSG::member &member, XMSG::Type new_type,
                     std::vector<ty> x) {
    auto type = new_type;

    if (type & XMSG::Type::CHAR) {
      assign_and_set_array<int64_t, ty>(member.data, x);
    } else if (type & XMSG::Type::INT) {
      assign_and_set_array<int64_t, ty>(member.data, x);
    } else if (type & XMSG::Type::UINT) {
      assign_and_set_array<uint64_t, ty>(member.data, x);
    } else if (type & XMSG::Type::DOUBLE) {
      assign_and_set_array<double, ty>(member.data, x);
    } else if (type & XMSG::Type::BOOL) {
      assign_and_set_array<bool, ty>(member.data, x);
    } else {
      throw std::invalid_argument("not a settable type");
    }
  }

  // transformation

  template <typename undertype>
  static void transformation_handle_addition(member &member,
                                             const std::string &term,
                                             const std::string &decimal) {
    if (decimal.empty()) {
      *static_cast<undertype *>(member.data) =
          *static_cast<undertype *>(member.data) +
          static_cast<undertype>(std::stoi(term));
    } else {
      *static_cast<undertype *>(member.data) =
          *static_cast<undertype *>(member.data) +
          static_cast<undertype>(std::stod(term));
    }
  }

  template <typename undertype>
  static void transformation_handle_subtraction(member &member,
                                                const std::string &term,
                                                const std::string &decimal) {
    if (decimal.empty()) {
      *static_cast<undertype *>(member.data) =
          *static_cast<undertype *>(member.data) -
          static_cast<undertype>(std::stoi(term));
    } else {
      *static_cast<undertype *>(member.data) =
          *static_cast<undertype *>(member.data) -
          static_cast<undertype>(std::stod(term));
    }
  }

  template <typename undertype>
  static void transformation_handle_multiplication(member &member,
                                                   const std::string &term,
                                                   const std::string &decimal) {
    if (decimal.empty()) {
      *static_cast<undertype *>(member.data) =
          *static_cast<undertype *>(member.data) *
          static_cast<undertype>(std::stoi(term));
    } else {
      *static_cast<undertype *>(member.data) =
          *static_cast<undertype *>(member.data) *
          static_cast<undertype>(std::stod(term));
    }
  }

  template <typename undertype>
  static void transformation_handle_division(member &member,
                                             const std::string &term,
                                             const std::string &decimal) {
    if (decimal.empty()) {
      *static_cast<undertype *>(member.data) =
          *static_cast<undertype *>(member.data) /
          static_cast<undertype>(std::stoi(term));
    } else {
      *static_cast<undertype *>(member.data) =
          *static_cast<undertype *>(member.data) /
          static_cast<undertype>(std::stod(term));
    }
  }

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
