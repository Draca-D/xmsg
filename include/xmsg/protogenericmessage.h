#ifndef PROTOGENERICMESSAGE_H
#define PROTOGENERICMESSAGE_H

#include <xmsg/genericmessage.h>

#include <google/protobuf/descriptor.h>
#include <google/protobuf/descriptor_database.h>
#include <google/protobuf/dynamic_message.h>
#include <google/protobuf/io/zero_copy_stream.h>
#include <google/protobuf/io/zero_copy_stream_impl.h>
#include <google/protobuf/reflection.h>
#include <google/protobuf/text_format.h>

// #include <dccl.h> // headers from dccl_vendor are not found by this package,
// but can be found
//  by other packages ive made, only difference is that this a library and the
//  other packages have had executables

class XMSG::ProtoGenericMessage : public XMSG::GenericMessage {
public:
  ProtoGenericMessage(const std::string &type);
  ProtoGenericMessage(const google::protobuf::Message *message);

  ~ProtoGenericMessage() override;

  const XMSG::member get_member(const std::string &member_name,
                                const Type expected_type) override;
  void set_member(const std::string &member_name, XMSG::member data) override;

  google::protobuf::Message *get_message_reference() { return message_; }

  std::string get_short_debug_string() { return message_->ShortDebugString(); }
  std::string get_debug_string() {
    message_->CheckInitialized();
    return message_->DebugString();
  }

private: // members
  bool released_ = false;
  google::protobuf::Message *message_;

private: // methods
  const google::protobuf::FieldDescriptor *find_field(const std::string name);

  void enumerate_members();

  void *extract(const google::protobuf::Reflection *reflection,
                const google::protobuf::FieldDescriptor *field);
  void *extract_repeated(const google::protobuf::Reflection *reflection,
                         const google::protobuf::FieldDescriptor *field);

  void set(void *data, const google::protobuf::Reflection *reflection,
           const google::protobuf::FieldDescriptor *field);
  void set_repeated(void *data, const google::protobuf::Reflection *reflection,
                    const google::protobuf::FieldDescriptor *field);

  XMSG::Type
  protoTypeToType(google::protobuf::FieldDescriptor::CppType protoType);

  GenericMessage *New(const std::string &type) override {
    return new ProtoGenericMessage(type);
  }

  void released() { released_ = true; }
};

#endif // PROTOGENERICMESSAGE_H
