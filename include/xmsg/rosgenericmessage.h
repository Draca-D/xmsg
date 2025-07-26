#ifndef ROSGENERICMESSAGE_H
#define ROSGENERICMESSAGE_H

#include <xmsg/genericmessage.h>

// ROS type introspection and serialisation
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rosidl_typesupport_introspection_cpp/field_types.hpp>
#include <rosidl_typesupport_introspection_cpp/identifier.hpp>
#include <rosidl_typesupport_introspection_cpp/message_introspection.hpp>
#include <rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp>

// Basic Types, all ROS messages are built from these basic types
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/char.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/int16.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/int64.hpp>
#include <std_msgs/msg/int8.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/u_int16.hpp>
#include <std_msgs/msg/u_int32.hpp>
#include <std_msgs/msg/u_int64.hpp>
#include <std_msgs/msg/u_int8.hpp>

class XMSG::ROSGenericMessage : public XMSG::GenericMessage {
public:
  ROSGenericMessage(const std::string &type);
  ROSGenericMessage(const std::string &type,
                    const std::shared_ptr<rclcpp::SerializedMessage> message);
  ROSGenericMessage(const std::string &type, const void *message);
  virtual ~ROSGenericMessage() override;

  const XMSG::RawMemberWrapper get_member(const std::string &member_name,
                                const Type expected_type) const override;
  void set_member(const std::string &member_name,
                  const XMSG::RawMemberWrapper &data) override;

  void deep_memcpy_to(void *data);
  void deep_memcpy_from(const void *data);

  const std::pair<void *, size_t> get_message_reference() {
    return {message_, sz_};
  }

  std::shared_ptr<rclcpp::SerializedMessage> get_serialised();

private: // members
  void *message_;
  size_t sz_;
  std::string type_support_name_;
  std::string serialiser_support_name_;

  std::shared_ptr<rcpputils::SharedLibrary> library_;

  bool initialised_;

  const rosidl_message_type_support_t *type_handle_;
  const rosidl_typesupport_introspection_cpp::MessageMembers *intro_msg_;

private: // methods
  GenericMessage *New(const std::string &type) const override {
    return new ROSGenericMessage(type);
  }

  void enumerate_members();
  void *
  extract(const rosidl_typesupport_introspection_cpp::MessageMember &member,
          const void *sub_msg) const;
  void *extract_array(
      const rosidl_typesupport_introspection_cpp::MessageMember &member,
      const void *sub_msg) const;

  void set(const rosidl_typesupport_introspection_cpp::MessageMember &member,
           void *ros_member_loc, void *data);
  void set_repeated(
      const rosidl_typesupport_introspection_cpp::MessageMember &member,
      void *ros_member_loc, void *data);

  XMSG::Type member_to_type(uint8_t type) const noexcept;

  int find_member_pos(const std::string &member_name) const;

  void set_member_at(const std::string &member_name,
                     const XMSG::RawMemberWrapper &data, void *at);
  const XMSG::RawMemberWrapper get_member_at(const std::string &member_name,
                                   const Type expected_type,
                                   const void *at) const;
  void cleanup(void *ros_msg,
               const rosidl_typesupport_introspection_cpp::MessageMembers
                   *introspected_msg);

public
    : /*HACK METHODS TO SUPPORT ROS GALACTIC LIMITATION IN DYNAMIC ALLOCATION OF
 ARRAY OF COMPLEX TYPES, WHEN MOVING TO HUMBLE, DELETE ALL FOLLOWING FUNCTIONS
 AND UPDATE FUNCTION 'set_repeated' WHERE 'type == ROS_TYPE_MESSAGE'
 // AND USE HUMBLE'S SUPPORT FOR SETTING AN ARRAY
 //SEE:
 https://github.com/ros2/rosidl/blob/humble/rosidl_typesupport_introspection_cpp/include/rosidl_typesupport_introspection_cpp/message_introspection.hpp
 SPECIFICALLY: void (* assign_function)(void *, size_t index, const void *);
 AND: void (* init_function)(void *, rosidl_runtime_cpp::MessageInitialization);

 Its up to the package including this library to provide the specific templated
 types necessary for array of complex types to work The package needs to call
 the following:

 XMSG::ROSGenericMessage::register_nested_handler("ros-package-name/msg/msgname",
 [](void *ros_member_loc, void
 *data){handle_repeated_nested_msg<ros-package-name::msg::msgname>(ros_member_loc,
 data);});

 for each type that we want array support for

 EVERYTHING BELOW HERE IS A HACK
 CPP HACKS WILL BE IDENTIFIED
 */
  static void
  register_nested_handler(const std::string &type_name,
                          std::function<void(void *, void *)> handler) {
    repeated_handlers_.insert({type_name, handler});
  }

  template <typename tp>
  static void handle_repeated_nested_msg(void *ros_member_loc, void *data) {
    auto submsg = new (ros_member_loc) std::vector<tp>;
    auto array = static_cast<std::vector<GenericMessage *> *>(data);

    for (auto &generic : *array) {
      tp msg;

      auto ros_msg = dynamic_cast<ROSGenericMessage *>(generic);

      ros_msg->deep_memcpy_to(&msg);
      submsg->push_back(msg);
    }
  }

private:
  inline static std::map<std::string, std::function<void(void *, void *)>>
      repeated_handlers_;
};

#endif // ROSGENERICMESSAGE_H
