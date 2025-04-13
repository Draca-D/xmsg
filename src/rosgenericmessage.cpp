#include "xmsg/rosgenericmessage.h"

XMSG::ROSGenericMessage::ROSGenericMessage(const std::string &type) {
  set_identifier("ROSMessage");
  set_type(type);

  type_support_name_ = "rosidl_typesupport_introspection_cpp";
  serialiser_support_name_ = "rosidl_typesupport_cpp";

  library_ = rclcpp::get_typesupport_library(get_type(), type_support_name_);
  type_handle_ = rclcpp::get_typesupport_handle(get_type(), type_support_name_,
                                                library_.operator*());
  intro_msg_ =
      static_cast<const rosidl_typesupport_introspection_cpp::MessageMembers *>(
          type_handle_->data); // this returns a fun struct, that has
                               // information about members and types and allows
                               // for dynamic packing of an unknown message
  sz_ = intro_msg_->size_of_;
  message_ = calloc(1, sz_);
  initialised_ = false;

  enumerate_members();
}

XMSG::ROSGenericMessage::ROSGenericMessage(
    const std::string &type,
    const std::shared_ptr<rclcpp::SerializedMessage> message)
    : ROSGenericMessage(type) {
  auto library =
      rclcpp::get_typesupport_library(get_type(), serialiser_support_name_);
  auto type_handle = rclcpp::get_typesupport_handle(
      get_type(), serialiser_support_name_, library.operator*());

  rclcpp::SerializationBase serialiser(
      type_handle); // while the ros message is in serialised form, its not
                    // useful to us,
  // we need to deserialise it into something

  serialiser.deserialize_message(message.get(),
                                 message_); // deserialise ros message
  initialised_ = true;
}

XMSG::ROSGenericMessage::ROSGenericMessage(const std::string &type,
                                           const void *message)
    : ROSGenericMessage(type) {
  deep_memcpy_from(message);
  initialised_ = true;
}

XMSG::ROSGenericMessage::~ROSGenericMessage() {
  if (initialised_) {
    cleanup(message_, intro_msg_);
  }

  free(message_);
}

const XMSG::member
XMSG::ROSGenericMessage::get_member(const std::string &member_name,
                                    const Type expected_type) {
  return get_member_at(member_name, expected_type, message_);
}

void XMSG::ROSGenericMessage::set_member(const std::string &member_name,
                                         XMSG::member data) {
  set_member_at(member_name, data, message_);
}

void XMSG::ROSGenericMessage::deep_memcpy_to(void *data) {
  for (auto &memb : get_members()) {
    auto member = get_member(memb.member_name, memb.type);

    set_member_at(memb.member_name, member, data);

    delete_member(member);
  }
}

void XMSG::ROSGenericMessage::deep_memcpy_from(const void *data) {
  for (auto &memb : get_members()) {
    auto member = get_member_at(memb.member_name, memb.type, data);

    set_member(memb.member_name, member);

    delete_member(member);
  }
}

std::shared_ptr<rclcpp::SerializedMessage>
XMSG::ROSGenericMessage::get_serialised() {
  if (!initialised_) {
    throw XMSG::BrokenMessage("ROS Message has not been initialised with any "
                              "data, unable to serialise");
  }

  auto library =
      rclcpp::get_typesupport_library(get_type(), serialiser_support_name_);
  auto type_handle = rclcpp::get_typesupport_handle(
      get_type(), serialiser_support_name_, library.operator*());

  auto serialized_msg = std::make_shared<rclcpp::SerializedMessage>();
  auto message_header_length = 8u;
  auto message_payload_length = sz_;
  serialized_msg->reserve(message_header_length + message_payload_length);

  rclcpp::SerializationBase serialiser(type_handle);
  serialiser.serialize_message(message_, serialized_msg.get());

  return serialized_msg;
}

void XMSG::ROSGenericMessage::set_member_at(const std::string &member_name,
                                            XMSG::member data, void *at) {
  initialised_ = true;

  auto pos = find_member_pos(member_name);

  if (pos == -1) {
    throw std::invalid_argument("No member " + member_name + " in message " +
                                get_type());
  }

  auto member = intro_msg_->members_[pos];
  auto sub = static_cast<char *>(at) + member.offset_;

  if (member.is_array_) {
    set_repeated(member, sub, data.data);
  } else {
    set(member, sub, data.data);
  }
}

const XMSG::member XMSG::ROSGenericMessage::get_member_at(
    const std::string &member_name, const Type expected_type, const void *at) {
  (void)expected_type; // unused for now, may be used for type safety

  auto pos = find_member_pos(member_name);

  if (pos == -1) {
    return {nullptr, XMSG::Type::NOTSET};
  }

  auto member = intro_msg_->members_[pos];
  auto type = member_to_type(member.type_id_);
  auto sub = static_cast<const char *>(at) + member.offset_;

  void *data;

  if (member.is_array_) {
    data = extract_array(member, sub);
    type = static_cast<XMSG::Type>(static_cast<int>(type) |
                                   static_cast<int>(XMSG::Type::ARRAY));
  } else {
    data = extract(member, sub);
  }

  return {data, type};
}

void XMSG::ROSGenericMessage::cleanup(
    void *ros_msg, const rosidl_typesupport_introspection_cpp::MessageMembers
                       *introspected_msg) {
  for (uint32_t x = 0; x < introspected_msg->member_count_; x++) {
    auto member = introspected_msg->members_[x];

    void *msg = static_cast<char *>(ros_msg) + member.offset_;

    if (member.members_) { // nested message
      auto introspected_msg2 = static_cast<
          const rosidl_typesupport_introspection_cpp::MessageMembers *>(
          member.members_->data); // get introspection for sub message

      cleanup(msg,
              introspected_msg2); // recursive call, this will allow multiple
                                  // depths of nested messages to work
      continue;
    }

    if (member.is_array_) {
      using namespace rosidl_typesupport_introspection_cpp;
      using namespace std_msgs::msg;

      if (member.members_) { // nested message
        auto introspected_msg2 = static_cast<
            const rosidl_typesupport_introspection_cpp::MessageMembers *>(
            member.members_->data); // get introspection for sub message
        auto sz_func = member.size_function;
        auto gt_func = member.get_function;

        auto sz = sz_func(msg);

        std::string ros_namespace = introspected_msg2->message_namespace_;
        std::string ros_type = introspected_msg2->message_name_;

        ros_namespace.replace(ros_namespace.find("::"), 2, "/");

        auto type_name = ros_namespace + "/" + ros_type;
        for (size_t x = 0; x < sz;
             x++) { // first cleanup the messages inside the vector, make sure
                    // all their memory is gone
          auto memb = gt_func(msg, x);
          cleanup(memb, introspected_msg2);
        }

        static_cast<std::vector<void *> *>(msg)
            ->~vector(); // then, make sure the vector itself is cleaned up

        continue;
      }

      // Determines ros base type and calls its destructor, make sure any memory
      // creaed by it has been cleaned out.
      if (member.type_id_ == ROS_TYPE_STRING) {
        static_cast<std::vector<String> *>(msg)->~vector();
      } else if (member.type_id_ == ROS_TYPE_FLOAT) {
        static_cast<std::vector<Float32> *>(msg)->~vector();
      } else if (member.type_id_ == ROS_TYPE_DOUBLE) {
        static_cast<std::vector<Float64> *>(msg)->~vector();
      }
      //  else if(member.type_id_ ==
      //  ROS_TYPE_LONG_DOUBLE){static_cast<Float32*>(msg)->~;} //not handled on
      //  this architecture
      else if (member.type_id_ == ROS_TYPE_CHAR) {
        static_cast<std::vector<Char> *>(msg)->~vector();
      } else if (member.type_id_ == ROS_TYPE_WCHAR) {
        static_cast<std::vector<Int16> *>(msg)->~vector();
      } else if (member.type_id_ == ROS_TYPE_BOOLEAN) {
        static_cast<std::vector<Bool> *>(msg)->~vector();
      }
      //  else if(member.type_id_ ==
      //  ROS_TYPE_OCTET){static_cast<Float32*>(msg)->~;} //not handled on this
      //  architecture
      else if (member.type_id_ == ROS_TYPE_UINT8) {
        static_cast<std::vector<UInt8> *>(msg)->~vector();
      } else if (member.type_id_ == ROS_TYPE_INT8) {
        static_cast<std::vector<Int8> *>(msg)->~vector();
      } else if (member.type_id_ == ROS_TYPE_UINT16) {
        static_cast<std::vector<UInt16> *>(msg)->~vector();
      } else if (member.type_id_ == ROS_TYPE_INT16) {
        static_cast<std::vector<Int16> *>(msg)->~vector();
      } else if (member.type_id_ == ROS_TYPE_UINT32) {
        static_cast<std::vector<UInt32> *>(msg)->~vector();
      } else if (member.type_id_ == ROS_TYPE_INT32) {
        static_cast<std::vector<Int32> *>(msg)->~vector();
      } else if (member.type_id_ == ROS_TYPE_UINT64) {
        static_cast<std::vector<UInt64> *>(msg)->~vector();
      } else if (member.type_id_ == ROS_TYPE_INT64) {
        static_cast<std::vector<Int64> *>(msg)->~vector();
      } else if (member.type_id_ == ROS_TYPE_WSTRING) {
        static_cast<std::vector<String> *>(msg)->~vector();
      }
    } else {
      using namespace rosidl_typesupport_introspection_cpp;
      using namespace std_msgs::msg;

      if (member.members_) { // nested message
        auto introspected_msg2 = static_cast<
            const rosidl_typesupport_introspection_cpp::MessageMembers *>(
            member.members_->data); // get introspection for sub message
        cleanup(msg,
                introspected_msg2); // recursive call, this will allow multiple
                                    // depths of nested messages to work
        continue;
      }

      // Determines ros base type and calls its destructor, make sure any memory
      // created by it has been cleaned out.
      if (member.type_id_ == ROS_TYPE_STRING) {
        static_cast<String *>(msg)->~String_();
      } else if (member.type_id_ == ROS_TYPE_FLOAT) {
        static_cast<Float32 *>(msg)->~Float32_();
      } else if (member.type_id_ == ROS_TYPE_DOUBLE) {
        static_cast<Float64 *>(msg)->~Float64_();
      }
      //  else if(member.type_id_ ==
      //  ROS_TYPE_LONG_DOUBLE){static_cast<Float32*>(msg)->~;} //not handled on
      //  this architecture
      else if (member.type_id_ == ROS_TYPE_CHAR) {
        static_cast<Char *>(msg)->~Char_();
      } else if (member.type_id_ == ROS_TYPE_WCHAR) {
        static_cast<Int16 *>(msg)->~Int16_();
      } else if (member.type_id_ == ROS_TYPE_BOOLEAN) {
        static_cast<Bool *>(msg)->~Bool();
      }
      //  else if(member.type_id_ ==
      //  ROS_TYPE_OCTET){static_cast<Float32*>(msg)->~;} //not handled on this
      //  architecture
      else if (member.type_id_ == ROS_TYPE_UINT8) {
        static_cast<UInt8 *>(msg)->~UInt8_();
      } else if (member.type_id_ == ROS_TYPE_INT8) {
        static_cast<Int8 *>(msg)->~Int8_();
      } else if (member.type_id_ == ROS_TYPE_UINT16) {
        static_cast<UInt16 *>(msg)->~UInt16_();
      } else if (member.type_id_ == ROS_TYPE_INT16) {
        static_cast<Int16 *>(msg)->~Int16_();
      } else if (member.type_id_ == ROS_TYPE_UINT32) {
        static_cast<UInt32 *>(msg)->~UInt32_();
      } else if (member.type_id_ == ROS_TYPE_INT32) {
        static_cast<Int32 *>(msg)->~Int32_();
      } else if (member.type_id_ == ROS_TYPE_UINT64) {
        static_cast<UInt64 *>(msg)->~UInt64_();
      } else if (member.type_id_ == ROS_TYPE_INT64) {
        static_cast<Int64 *>(msg)->~Int64_();
      } else if (member.type_id_ == ROS_TYPE_WSTRING) {
        static_cast<String *>(msg)->~String_();
      }
    }
  }
}

void XMSG::ROSGenericMessage::enumerate_members() {
  for (uint32_t x = 0; x < intro_msg_->member_count_;
       x++) { // iterate through all ros message members
    auto member = intro_msg_->members_[x];
    auto type = member_to_type(member.type_id_);
    auto type_name = std::string("basic type");
    auto name = std::string(member.name_);

    if (type == XMSG::Type::GENERIC) {
      auto sub_message_intro = static_cast<
          const rosidl_typesupport_introspection_cpp::MessageMembers *>(
          member.members_->data);
      std::string ros_namespace = sub_message_intro->message_namespace_;
      std::string ros_type = sub_message_intro->message_name_;

      ros_namespace.replace(ros_namespace.find("::"), 2, "/");

      type_name = ros_namespace + "/" + ros_type;
    }

    if (member.is_array_) {
      type = static_cast<XMSG::Type>(static_cast<int>(type) |
                                     static_cast<int>(XMSG::Type::ARRAY));
    }

    add_member(name, type, type_name);
  }
}

void *XMSG::ROSGenericMessage::extract(
    const rosidl_typesupport_introspection_cpp::MessageMember &member,
    const void *sub_msg) {
  auto type = member.type_id_;
  void *data;

  using namespace rosidl_typesupport_introspection_cpp;

  if (type == ROS_TYPE_STRING) {
    data = new std::string(
        static_cast<const std_msgs::msg::String *>(sub_msg)->data);
  } else if (type == ROS_TYPE_FLOAT) {
    data = calloc(1, sizeof(double));
    static_cast<double *>(data)[0] = static_cast<double>(
        static_cast<const std_msgs::msg::Float32 *>(sub_msg)->data);
  } else if (type == ROS_TYPE_DOUBLE) {
    data = calloc(1, sizeof(double));
    static_cast<double *>(data)[0] = static_cast<double>(
        static_cast<const std_msgs::msg::Float64 *>(sub_msg)->data);
  } else if (type == ROS_TYPE_CHAR) {
    data = calloc(1, sizeof(int64_t));
    static_cast<int64_t *>(data)[0] = static_cast<int64_t>(
        static_cast<const std_msgs::msg::Char *>(sub_msg)->data);
  } else if (type == ROS_TYPE_WCHAR) {
    data = calloc(1, sizeof(int64_t));
    static_cast<int64_t *>(data)[0] = static_cast<int64_t>(
        static_cast<const std_msgs::msg::Char *>(sub_msg)->data);
  } else if (type == ROS_TYPE_BOOLEAN) {
    data = calloc(1, sizeof(bool));
    static_cast<bool *>(data)[0] = static_cast<bool>(
        static_cast<const std_msgs::msg::Bool *>(sub_msg)->data);
  } else if (type == ROS_TYPE_UINT8) {
    data = calloc(1, sizeof(uint64_t));
    static_cast<uint64_t *>(data)[0] = static_cast<uint64_t>(
        static_cast<const std_msgs::msg::UInt8 *>(sub_msg)->data);
  } else if (type == ROS_TYPE_INT8) {
    data = calloc(1, sizeof(int64_t));
    static_cast<int64_t *>(data)[0] = static_cast<int64_t>(
        static_cast<const std_msgs::msg::Int8 *>(sub_msg)->data);
  } else if (type == ROS_TYPE_UINT16) {
    data = calloc(1, sizeof(uint64_t));
    static_cast<uint64_t *>(data)[0] = static_cast<uint64_t>(
        static_cast<const std_msgs::msg::UInt16 *>(sub_msg)->data);
  } else if (type == ROS_TYPE_INT16) {
    data = calloc(1, sizeof(int64_t));
    static_cast<int64_t *>(data)[0] = static_cast<int64_t>(
        static_cast<const std_msgs::msg::Int16 *>(sub_msg)->data);
  } else if (type == ROS_TYPE_UINT32) {
    data = calloc(1, sizeof(uint64_t));
    static_cast<uint64_t *>(data)[0] = static_cast<uint64_t>(
        static_cast<const std_msgs::msg::UInt32 *>(sub_msg)->data);
  } else if (type == ROS_TYPE_INT32) {
    data = calloc(1, sizeof(int64_t));
    static_cast<int64_t *>(data)[0] = static_cast<int64_t>(
        static_cast<const std_msgs::msg::Int32 *>(sub_msg)->data);
  } else if (type == ROS_TYPE_UINT64) {
    data = calloc(1, sizeof(uint64_t));
    static_cast<uint64_t *>(data)[0] = static_cast<uint64_t>(
        static_cast<const std_msgs::msg::UInt64 *>(sub_msg)->data);
  } else if (type == ROS_TYPE_INT64) {
    data = calloc(1, sizeof(int64_t));
    static_cast<int64_t *>(data)[0] = static_cast<int64_t>(
        static_cast<const std_msgs::msg::Int64 *>(sub_msg)->data);
  } else if (type == ROS_TYPE_WSTRING) {
    data = new std::string(
        static_cast<const std_msgs::msg::String *>(sub_msg)->data);
  } else if (type == ROS_TYPE_MESSAGE) {
    auto sub_message_intro = static_cast<
        const rosidl_typesupport_introspection_cpp::MessageMembers *>(
        member.members_->data);
    std::string ros_namespace = sub_message_intro->message_namespace_;
    std::string ros_type = sub_message_intro->message_name_;

    ros_namespace.replace(ros_namespace.find("::"), 2, "/");

    auto type_name = ros_namespace + "/" + ros_type;

    GenericMessage *msg = new ROSGenericMessage(type_name, sub_msg);
    data = msg;
  } else {
    data = nullptr;
  }

  return data;
}

void *XMSG::ROSGenericMessage::extract_array(
    const rosidl_typesupport_introspection_cpp::MessageMember &member,
    const void *sub_msg) {
  auto sz_func = member.size_function;
  auto gt_func = member.get_const_function; // array getter function
  auto type = member.type_id_;

  auto sz = sz_func(
      sub_msg); // size of member array //member.array_size_ does not work

  void *data;

  using namespace rosidl_typesupport_introspection_cpp;

  if (type == ROS_TYPE_STRING) {
    data = new std::vector<std::string>;
    for (size_t x = 0; x < sz; x++) {
      static_cast<std::vector<std::string> *>(data)->push_back(
          static_cast<const std_msgs::msg::String *>(gt_func(sub_msg, x))
              ->data);
    }
  } else if (type == ROS_TYPE_FLOAT) {
    data = new std::vector<double>;
    for (size_t x = 0; x < sz; x++) {
      static_cast<std::vector<double> *>(data)->push_back(static_cast<double>(
          static_cast<const std_msgs::msg::Float32 *>(gt_func(sub_msg, x))
              ->data));
    }
  } else if (type == ROS_TYPE_DOUBLE) {
    data = new std::vector<double>;
    for (size_t x = 0; x < sz; x++) {
      static_cast<std::vector<double> *>(data)->push_back(
          static_cast<const std_msgs::msg::Float64 *>(gt_func(sub_msg, x))
              ->data);
    }
  } else if (type == ROS_TYPE_CHAR) {
    data = new std::vector<int64_t>;
    for (size_t x = 0; x < sz; x++) {
      static_cast<std::vector<int64_t> *>(data)->push_back(
          static_cast<const std_msgs::msg::Char *>(gt_func(sub_msg, x))->data);
    }
  } else if (type == ROS_TYPE_WCHAR) {
    data = new std::vector<int64_t>;
    for (size_t x = 0; x < sz; x++) {
      static_cast<std::vector<int64_t> *>(data)->push_back(
          static_cast<const std_msgs::msg::Char *>(gt_func(sub_msg, x))->data);
    }
  } else if (type == ROS_TYPE_BOOLEAN) {
    data = new std::vector<bool>;
    for (size_t x = 0; x < sz; x++) {
      static_cast<std::vector<bool> *>(data)->push_back(
          static_cast<const std_msgs::msg::Bool *>(gt_func(sub_msg, x))->data);
    }
  } else if (type == ROS_TYPE_UINT8) {
    data = new std::vector<uint64_t>;
    for (size_t x = 0; x < sz; x++) {
      static_cast<std::vector<uint64_t> *>(data)->push_back(
          static_cast<const std_msgs::msg::UInt8 *>(gt_func(sub_msg, x))->data);
    }
  } else if (type == ROS_TYPE_INT8) {
    data = new std::vector<int64_t>;
    for (size_t x = 0; x < sz; x++) {
      static_cast<std::vector<int64_t> *>(data)->push_back(
          static_cast<const std_msgs::msg::Int8 *>(gt_func(sub_msg, x))->data);
    }
  } else if (type == ROS_TYPE_UINT16) {
    data = new std::vector<uint64_t>;
    for (size_t x = 0; x < sz; x++) {
      static_cast<std::vector<uint64_t> *>(data)->push_back(
          static_cast<const std_msgs::msg::UInt16 *>(gt_func(sub_msg, x))
              ->data);
    }
  } else if (type == ROS_TYPE_INT16) {
    data = new std::vector<int64_t>;
    for (size_t x = 0; x < sz; x++) {
      static_cast<std::vector<int64_t> *>(data)->push_back(
          static_cast<const std_msgs::msg::Int16 *>(gt_func(sub_msg, x))->data);
    }
  } else if (type == ROS_TYPE_UINT32) {
    data = new std::vector<uint64_t>;
    for (size_t x = 0; x < sz; x++) {
      static_cast<std::vector<uint64_t> *>(data)->push_back(
          static_cast<const std_msgs::msg::UInt32 *>(gt_func(sub_msg, x))
              ->data);
    }
  } else if (type == ROS_TYPE_INT32) {
    data = new std::vector<int64_t>;
    for (size_t x = 0; x < sz; x++) {
      static_cast<std::vector<int64_t> *>(data)->push_back(
          static_cast<const std_msgs::msg::Int32 *>(gt_func(sub_msg, x))->data);
    }
  } else if (type == ROS_TYPE_UINT64) {
    data = new std::vector<uint64_t>;
    for (size_t x = 0; x < sz; x++) {
      static_cast<std::vector<uint64_t> *>(data)->push_back(
          static_cast<const std_msgs::msg::UInt64 *>(gt_func(sub_msg, x))
              ->data);
    }
  } else if (type == ROS_TYPE_INT64) {
    data = new std::vector<int64_t>;
    for (size_t x = 0; x < sz; x++) {
      static_cast<std::vector<int64_t> *>(data)->push_back(
          static_cast<const std_msgs::msg::Int64 *>(gt_func(sub_msg, x))->data);
    }
  } else if (type == ROS_TYPE_WSTRING) {
    data = new std::vector<std::string>;
    for (size_t x = 0; x < sz; x++) {
      static_cast<std::vector<std::string> *>(data)->push_back(
          static_cast<const std_msgs::msg::String *>(gt_func(sub_msg, x))
              ->data);
    }
  } else if (type == ROS_TYPE_MESSAGE) {
    data = new std::vector<GenericMessage *>;
    auto sub_message_intro = static_cast<
        const rosidl_typesupport_introspection_cpp::MessageMembers *>(
        member.members_->data);
    std::string ros_namespace = sub_message_intro->message_namespace_;
    std::string ros_type = sub_message_intro->message_name_;

    ros_namespace.replace(ros_namespace.find("::"), 2, "/");

    auto type_name = ros_namespace + "/" + ros_type;
    for (size_t x = 0; x < sz; x++) {
      GenericMessage *msg =
          new ROSGenericMessage(type_name, gt_func(sub_msg, x));
      static_cast<std::vector<GenericMessage *> *>(data)->push_back(msg);
    }
  } else {
    data = nullptr;
  }

  return data;
}

void XMSG::ROSGenericMessage::set(
    const rosidl_typesupport_introspection_cpp::MessageMember &member,
    void *ros_member_loc, void *data) {
  using namespace rosidl_typesupport_introspection_cpp;

  auto type = member.type_id_;

  if (type == ROS_TYPE_STRING) {
    auto submsg = new (ros_member_loc) std_msgs::msg::String;
    submsg->data = *static_cast<const std::string *>(data);
  } else if (type == ROS_TYPE_FLOAT) {
    auto submsg = new (ros_member_loc) std_msgs::msg::Float32;
    submsg->data = static_cast<float>(*static_cast<const double *>(data));
  } else if (type == ROS_TYPE_DOUBLE) {
    auto submsg = new (ros_member_loc) std_msgs::msg::Float64;
    submsg->data = *static_cast<const double *>(data);
  } else if (type == ROS_TYPE_CHAR) {
    auto submsg = new (ros_member_loc) std_msgs::msg::UInt8;
    submsg->data =
        static_cast<unsigned char>(*static_cast<const int64_t *>(data));
  } else if (type == ROS_TYPE_WCHAR) {
    auto submsg = new (ros_member_loc) std_msgs::msg::Int16;
    submsg->data = static_cast<int16_t>(*static_cast<const int64_t *>(data));
  } else if (type == ROS_TYPE_BOOLEAN) {
    auto submsg = new (ros_member_loc) std_msgs::msg::Bool;
    submsg->data = static_cast<bool>(*static_cast<const bool *>(data));
  } else if (type == ROS_TYPE_UINT8) {
    auto submsg = new (ros_member_loc) std_msgs::msg::UInt8;
    submsg->data = static_cast<uint8_t>(*static_cast<const uint64_t *>(data));
  } else if (type == ROS_TYPE_INT8) {
    auto submsg = new (ros_member_loc) std_msgs::msg::Int8;
    submsg->data = static_cast<int8_t>(*static_cast<const int64_t *>(data));
  } else if (type == ROS_TYPE_UINT16) {
    auto submsg = new (ros_member_loc) std_msgs::msg::UInt16;
    submsg->data = static_cast<uint16_t>(*static_cast<const uint64_t *>(data));
  } else if (type == ROS_TYPE_INT16) {
    auto submsg = new (ros_member_loc) std_msgs::msg::Int16;
    submsg->data = static_cast<int16_t>(*static_cast<const int64_t *>(data));
  } else if (type == ROS_TYPE_UINT32) {
    auto submsg = new (ros_member_loc) std_msgs::msg::UInt32;
    submsg->data = static_cast<uint32_t>(*static_cast<const uint64_t *>(data));
  } else if (type == ROS_TYPE_INT32) {
    auto submsg = new (ros_member_loc) std_msgs::msg::Int32;
    submsg->data = static_cast<int32_t>(*static_cast<const int64_t *>(data));
  } else if (type == ROS_TYPE_UINT64) {
    auto submsg = new (ros_member_loc) std_msgs::msg::UInt64;
    submsg->data = static_cast<uint64_t>(*static_cast<const uint64_t *>(data));
  } else if (type == ROS_TYPE_INT64) {
    auto submsg = new (ros_member_loc) std_msgs::msg::Int64;
    submsg->data = static_cast<int64_t>(*static_cast<const int64_t *>(data));
  } else if (type == ROS_TYPE_WSTRING) {
    auto submsg = new (ros_member_loc) std_msgs::msg::String;
    submsg->data = *static_cast<const std::string *>(data);
  } else if (type == ROS_TYPE_MESSAGE) {
    auto generic = static_cast<GenericMessage *>(data);
    auto ros_msg = dynamic_cast<ROSGenericMessage *>(generic);
    ros_msg->deep_memcpy_to(ros_member_loc);
  }
}

void XMSG::ROSGenericMessage::set_repeated(
    const rosidl_typesupport_introspection_cpp::MessageMember &member,
    void *ros_member_loc, void *data) {
  using namespace rosidl_typesupport_introspection_cpp;

  auto type = member.type_id_;

  if (type == ROS_TYPE_STRING) {
    auto submsg = new (ros_member_loc) std::vector<std_msgs::msg::String>;
    auto array = static_cast<std::vector<std::string> *>(data);
    for (auto &item : *array) {
      std_msgs::msg::String base;
      base.data = item;
      submsg->push_back(base);
    }
  } else if (type == ROS_TYPE_FLOAT) {
    auto submsg = new (ros_member_loc) std::vector<std_msgs::msg::Float32>;
    auto array = static_cast<std::vector<double> *>(data);
    for (auto &item : *array) {
      std_msgs::msg::Float32 base;
      base.data = static_cast<float>(item);
      submsg->push_back(base);
    }
  } else if (type == ROS_TYPE_DOUBLE) {
    auto submsg = new (ros_member_loc) std::vector<std_msgs::msg::Float64>;
    auto array = static_cast<std::vector<double> *>(data);
    for (auto &item : *array) {
      std_msgs::msg::Float64 base;
      base.data = static_cast<double>(item);
      submsg->push_back(base);
    }
  } else if (type == ROS_TYPE_CHAR) {
    auto submsg = new (ros_member_loc) std::vector<std_msgs::msg::UInt8>;
    auto array = static_cast<std::vector<int64_t> *>(data);
    for (auto &item : *array) {
      std_msgs::msg::UInt8 base;
      base.data = static_cast<unsigned char>(item);
      submsg->push_back(base);
    }
  } else if (type == ROS_TYPE_WCHAR) {
    auto submsg = new (ros_member_loc) std::vector<std_msgs::msg::Int16>;
    auto array = static_cast<std::vector<int64_t> *>(data);
    for (auto &item : *array) {
      std_msgs::msg::Int16 base;
      base.data = static_cast<int16_t>(item);
      submsg->push_back(base);
    }
  } else if (type == ROS_TYPE_BOOLEAN) {
    auto submsg = new (ros_member_loc) std::vector<std_msgs::msg::Bool>;
    auto array = static_cast<std::vector<bool> *>(data);
    for (size_t x = 0; x < array->size(); x++) {
      std_msgs::msg::Bool base;
      base.data = array->at(x);
      submsg->push_back(base);
    }
  } else if (type == ROS_TYPE_UINT8) {
    auto submsg = new (ros_member_loc) std::vector<std_msgs::msg::UInt8>;
    auto array = static_cast<std::vector<uint64_t> *>(data);
    for (auto &item : *array) {
      std_msgs::msg::UInt8 base;
      base.data = static_cast<uint8_t>(item);
      submsg->push_back(base);
    }
  } else if (type == ROS_TYPE_INT8) {
    auto submsg = new (ros_member_loc) std::vector<std_msgs::msg::Int8>;
    auto array = static_cast<std::vector<int64_t> *>(data);
    for (auto &item : *array) {
      std_msgs::msg::Int8 base;
      base.data = static_cast<int8_t>(item);
      submsg->push_back(base);
    }
  } else if (type == ROS_TYPE_UINT16) {
    auto submsg = new (ros_member_loc) std::vector<std_msgs::msg::UInt16>;
    auto array = static_cast<std::vector<uint64_t> *>(data);
    for (auto &item : *array) {
      std_msgs::msg::UInt16 base;
      base.data = static_cast<uint16_t>(item);
      submsg->push_back(base);
    }
  } else if (type == ROS_TYPE_INT16) {
    auto submsg = new (ros_member_loc) std::vector<std_msgs::msg::Int16>;
    auto array = static_cast<std::vector<int64_t> *>(data);
    for (auto &item : *array) {
      std_msgs::msg::Int16 base;
      base.data = static_cast<int16_t>(item);
      submsg->push_back(base);
    }
  } else if (type == ROS_TYPE_UINT32) {
    auto submsg = new (ros_member_loc) std::vector<std_msgs::msg::UInt32>;
    auto array = static_cast<std::vector<uint64_t> *>(data);
    for (auto &item : *array) {
      std_msgs::msg::UInt32 base;
      base.data = static_cast<uint32_t>(item);
      submsg->push_back(base);
    }
  } else if (type == ROS_TYPE_INT32) {
    auto submsg = new (ros_member_loc) std::vector<std_msgs::msg::Int32>;
    auto array = static_cast<std::vector<int64_t> *>(data);
    for (auto &item : *array) {
      std_msgs::msg::Int32 base;
      base.data = static_cast<int32_t>(item);
      submsg->push_back(base);
    }
  } else if (type == ROS_TYPE_UINT64) {
    auto submsg = new (ros_member_loc) std::vector<std_msgs::msg::UInt64>;
    auto array = static_cast<std::vector<uint64_t> *>(data);
    for (auto &item : *array) {
      std_msgs::msg::UInt64 base;
      base.data = static_cast<uint64_t>(item);
      submsg->push_back(base);
    }
  } else if (type == ROS_TYPE_INT64) {
    auto submsg = new (ros_member_loc) std::vector<std_msgs::msg::Int64>;
    auto array = static_cast<std::vector<int64_t> *>(data);
    for (auto &item : *array) {
      std_msgs::msg::Int64 base;
      base.data = static_cast<int64_t>(item);
      submsg->push_back(base);
    }
  } else if (type == ROS_TYPE_WSTRING) {
    auto submsg = new (ros_member_loc) std::vector<std_msgs::msg::String>;
    auto array = static_cast<std::vector<std::string> *>(data);
    for (auto &item : *array) {
      std_msgs::msg::String base;
      base.data = item;
      submsg->push_back(base);
    }
  } else if (type == ROS_TYPE_MESSAGE) { // HACKS START HERE. SEE HEADER FILE
    auto array = static_cast<std::vector<GenericMessage *> *>(data);

    if (array->empty()) {
      return;
    }

    auto ros_msg = dynamic_cast<ROSGenericMessage *>(array->front());

    if (repeated_handlers_.find(ros_msg->get_type()) ==
        repeated_handlers_.end()) {
      std::stringstream support_types;

      for (auto &[key, func] : repeated_handlers_) {
        support_types << key << ",";
      }

      throw XMSG::BrokenMessage("Array of Complex types in ROS not supported, "
                                "except for the following: " +
                                support_types.str() +
                                " Attempted type was: " + ros_msg->get_type() +
                                ". Hack due to limitation of ROS Galactic. If "
                                "moved on to Humble, please fix this");
    }

    repeated_handlers_[ros_msg->get_type()](ros_member_loc, data);
  } // HACKS END HERE.
}

XMSG::Type XMSG::ROSGenericMessage::member_to_type(uint8_t type) {
  using namespace rosidl_typesupport_introspection_cpp;

  if (type == ROS_TYPE_STRING) {
    return XMSG::Type::STRING;
  } else if (type == ROS_TYPE_FLOAT) {
    return XMSG::Type::DOUBLE;
  } else if (type == ROS_TYPE_DOUBLE) {
    return XMSG::Type::DOUBLE;
  } else if (type == ROS_TYPE_CHAR) {
    return XMSG::Type::CHAR;
  } else if (type == ROS_TYPE_WCHAR) {
    return XMSG::Type::INT;
  } else if (type == ROS_TYPE_BOOLEAN) {
    return XMSG::Type::BOOL;
  } else if (type == ROS_TYPE_UINT8) {
    return XMSG::Type::UINT;
  } else if (type == ROS_TYPE_INT8) {
    return XMSG::Type::INT;
  } else if (type == ROS_TYPE_UINT16) {
    return XMSG::Type::UINT;
  } else if (type == ROS_TYPE_INT16) {
    return XMSG::Type::INT;
  } else if (type == ROS_TYPE_UINT32) {
    return XMSG::Type::UINT;
  } else if (type == ROS_TYPE_INT32) {
    return XMSG::Type::INT;
  } else if (type == ROS_TYPE_UINT64) {
    return XMSG::Type::UINT;
  } else if (type == ROS_TYPE_INT64) {
    return XMSG::Type::INT;
  } else if (type == ROS_TYPE_WSTRING) {
    return XMSG::Type::STRING;
  } else if (type == ROS_TYPE_MESSAGE) {
    return XMSG::Type::GENERIC;
  }

  return XMSG::Type::NOTSET;
}

int XMSG::ROSGenericMessage::find_member_pos(const std::string &member_name) {
  for (uint32_t x = 0; x < intro_msg_->member_count_;
       x++) { // iterate through all ros message members
    auto member = intro_msg_->members_[x];
    auto name = std::string(member.name_);

    if (compare(name, member_name)) {
      return static_cast<int>(x);
    }
  }

  return -1;
}
