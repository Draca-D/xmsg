#include "xmsg/protogenericmessage.h"

using namespace XMSG;

ProtoGenericMessage::ProtoGenericMessage(const std::string &type) {
  set_identifier("ProtoMessage");
  set_type(type);

  message_ = nullptr;

  auto desc =
      google::protobuf::DescriptorPool::generated_pool()->FindMessageTypeByName(
          type); // get file descriptor from protobuf pool
  if (!desc) {
    log("Unable to find message in DCCL definitions", XMSG::LogLevel::ERROR);
    throw std::invalid_argument("Message not found");
  }

  message_ = google::protobuf::MessageFactory::generated_factory()
                 ->GetPrototype(desc)
                 ->New(); // create protobuf message based on descriptor

  if (!message_) {
    log("Unable to create message but was able to find it",
        XMSG::LogLevel::ERROR);
    throw std::invalid_argument("Unable to create message");
  }

  enumerate_members();
}

ProtoGenericMessage::ProtoGenericMessage(
    const google::protobuf::Message *message) {
  set_identifier("ProtoMessage");
  set_type(message->GetDescriptor()->name());

  message_ = message->New();
  message_->CopyFrom(*message);

  enumerate_members();
}

ProtoGenericMessage::~ProtoGenericMessage() {
  if (message_ && !released_) {
    delete message_;
    message_ = nullptr;
  }
}

const member ProtoGenericMessage::get_member(const std::string &member_name,
                                             const XMSG::Type expected_type) {
  (void)expected_type; // unused for now, may be used for type safety

  auto field = find_field(member_name);

  if (!field) {
    return {nullptr, XMSG::Type::NOTSET};
  }

  auto reflection = message_->GetReflection();
  auto type = protoTypeToType(field->cpp_type());

  if (field->is_optional()) {
    if (!message_->GetReflection()->HasField(*message_, field)) {
      return {nullptr, XMSG::Type::NOTSET};
    }
  }

  void *data;

  if (field->is_repeated()) {
    data = extract_repeated(reflection, field);
    type = static_cast<XMSG::Type>(static_cast<int>(type) |
                                   static_cast<int>(XMSG::Type::ARRAY));
  } else {
    data = extract(reflection, field);
  }

  return {data, type};
}

void ProtoGenericMessage::set_member(const std::string &member_name,
                                     XMSG::member data) {
  auto field = find_field(member_name);

  if (!field) {
    throw std::invalid_argument("No member " + member_name + " in message " +
                                get_type());
  }

  auto reflection = message_->GetReflection();

  if (field->is_repeated()) {
    set_repeated(data.data, reflection, field);
  } else {
    set(data.data, reflection, field);
  }
}

const google::protobuf::FieldDescriptor *
ProtoGenericMessage::find_field(const std::string name) {
  auto desc = message_->GetDescriptor();

  for (int field = 0; field < desc->field_count(); field++) {
    auto field_desc = desc->field(field);
    auto member_name = field_desc->name();

    if (compare(member_name, name)) {
      return field_desc;
    }
  }

  return nullptr;
}

void ProtoGenericMessage::enumerate_members() {
  auto desc = message_->GetDescriptor();

  for (int field = 0; field < desc->field_count(); field++) {
    auto field_desc = desc->field(field);
    auto type = protoTypeToType(field_desc->cpp_type());
    auto name = field_desc->name();
    auto type_name = std::string("basic type");

    if (type == XMSG::Type::GENERIC) {
      type_name = field_desc->message_type()->name();
    }

    if (field_desc->is_repeated()) {
      type = static_cast<XMSG::Type>(static_cast<int>(type) |
                                     static_cast<int>(XMSG::Type::ARRAY));
    }

    add_member(name, type, type_name);
  }
}

void *
ProtoGenericMessage::extract(const google::protobuf::Reflection *reflection,
                             const google::protobuf::FieldDescriptor *field) {
  using namespace google::protobuf;

  void *data;

  int cpp_type = field->cpp_type();

  if (cpp_type == FieldDescriptor::CPPTYPE_INT32) {
    data = calloc(1, sizeof(int64_t));
    static_cast<int64_t *>(data)[0] = reflection->GetInt32(*message_, field);
  } else if (cpp_type == FieldDescriptor::CPPTYPE_UINT32) {
    data = calloc(1, sizeof(uint64_t));
    static_cast<uint64_t *>(data)[0] = reflection->GetUInt32(*message_, field);
  } else if (cpp_type == FieldDescriptor::CPPTYPE_INT64) {
    data = calloc(1, sizeof(int64_t));
    static_cast<int64_t *>(data)[0] = reflection->GetInt64(*message_, field);
  } else if (cpp_type == FieldDescriptor::CPPTYPE_UINT64) {
    data = calloc(1, sizeof(uint64_t));
    static_cast<uint64_t *>(data)[0] = reflection->GetUInt64(*message_, field);
  } else if (cpp_type == FieldDescriptor::CPPTYPE_DOUBLE) {
    data = calloc(1, sizeof(double));
    static_cast<double *>(data)[0] = reflection->GetDouble(*message_, field);
  } else if (cpp_type == FieldDescriptor::CPPTYPE_FLOAT) {
    data = calloc(1, sizeof(double));
    static_cast<double *>(data)[0] =
        static_cast<double>(reflection->GetFloat(*message_, field));
  } else if (cpp_type == FieldDescriptor::CPPTYPE_BOOL) {
    data = calloc(1, sizeof(bool));
    static_cast<bool *>(data)[0] = reflection->GetBool(*message_, field);
  } else if (cpp_type == FieldDescriptor::CPPTYPE_ENUM) {
    data = calloc(1, sizeof(int));
    static_cast<int *>(data)[0] = reflection->GetEnumValue(*message_, field);
    //    auto e = reflection->GetEnum(*message_, field);
    //    printf("Enum stuff, name: %s, fullname %s, debug: %s\n"
    //    ,e->name().c_str(), e->full_name().c_str(), e->DebugString().c_str());
  } else if (cpp_type == FieldDescriptor::CPPTYPE_STRING) {
    data = new std::string(reflection->GetString(*message_, field));
  } else if (cpp_type == FieldDescriptor::CPPTYPE_MESSAGE) {
    auto subMessage = &reflection->GetMessage(*message_, field);
    GenericMessage *msg = new ProtoGenericMessage(subMessage);
    data = msg;
  } else {
    data = nullptr;
  }

  return data;
}

void *ProtoGenericMessage::extract_repeated(
    const google::protobuf::Reflection *reflection,
    const google::protobuf::FieldDescriptor *field) {
  using namespace google::protobuf;

  void *data;

  int cpp_type = field->cpp_type();

  if (cpp_type == FieldDescriptor::CPPTYPE_INT32) {
    auto ref = reflection->GetRepeatedFieldRef<int32>(*message_, field);
    auto sz = ref.size();
    data = new std::vector<int64_t>;

    for (int x = 0; x < sz; x++) {
      static_cast<std::vector<int64_t> *>(data)->push_back(ref.Get(x));
    }
  } else if (cpp_type == FieldDescriptor::CPPTYPE_UINT32) {
    auto ref = reflection->GetRepeatedFieldRef<uint32>(*message_, field);
    auto sz = ref.size();
    data = new std::vector<uint64_t>;

    for (int x = 0; x < sz; x++) {
      static_cast<std::vector<uint64_t> *>(data)->push_back(ref.Get(x));
    }
  } else if (cpp_type == FieldDescriptor::CPPTYPE_INT64) {
    auto ref = reflection->GetRepeatedFieldRef<int64>(*message_, field);
    auto sz = ref.size();
    data = new std::vector<int64_t>;

    for (int x = 0; x < sz; x++) {
      static_cast<std::vector<int64_t> *>(data)->push_back(ref.Get(x));
    }
  } else if (cpp_type == FieldDescriptor::CPPTYPE_UINT64) {
    auto ref = reflection->GetRepeatedFieldRef<uint64>(*message_, field);
    auto sz = ref.size();
    data = new std::vector<uint64_t>;

    for (int x = 0; x < sz; x++) {
      static_cast<std::vector<uint64_t> *>(data)->push_back(ref.Get(x));
    }
  } else if (cpp_type == FieldDescriptor::CPPTYPE_DOUBLE) {
    auto ref = reflection->GetRepeatedFieldRef<double>(*message_, field);
    auto sz = ref.size();
    data = new std::vector<double>;

    for (int x = 0; x < sz; x++) {
      static_cast<std::vector<double> *>(data)->push_back(ref.Get(x));
    }
  } else if (cpp_type == FieldDescriptor::CPPTYPE_FLOAT) {
    auto ref = reflection->GetRepeatedFieldRef<float>(*message_, field);
    auto sz = ref.size();
    data = new std::vector<float>;

    for (int x = 0; x < sz; x++) {
      static_cast<std::vector<double> *>(data)->push_back(
          static_cast<double>(ref.Get(x)));
    }
  } else if (cpp_type == FieldDescriptor::CPPTYPE_BOOL) {
    auto ref = reflection->GetRepeatedFieldRef<bool>(*message_, field);
    auto sz = ref.size();
    data = new std::vector<bool>;

    for (int x = 0; x < sz; x++) {
      static_cast<std::vector<bool> *>(data)->push_back(ref.Get(x));
    }
  } else if (cpp_type == FieldDescriptor::CPPTYPE_ENUM) {
    auto ref = reflection->GetRepeatedFieldRef<int32>(*message_, field);
    auto sz = ref.size();
    data = new std::vector<int32_t>;

    for (int x = 0; x < sz; x++) {
      static_cast<std::vector<int32_t> *>(data)->push_back(ref.Get(x));
    }
  } else if (cpp_type == FieldDescriptor::CPPTYPE_STRING) {
    auto ref = reflection->GetRepeatedFieldRef<std::string>(*message_, field);
    auto sz = ref.size();
    data = new std::vector<std::string>;

    for (int x = 0; x < sz; x++) {
      static_cast<std::vector<std::string> *>(data)->push_back(ref.Get(x));
    }
  } else if (cpp_type == FieldDescriptor::CPPTYPE_MESSAGE) {
    auto ref = reflection->GetRepeatedFieldRef<google::protobuf::Message>(
        *message_, field);
    auto sz = ref.size();
    data = new std::vector<ProtoGenericMessage>;

    for (int x = 0; x < sz; x++) {
      auto subproto = &reflection->GetRepeatedMessage(*message_, field, x);
      auto submsg = new ProtoGenericMessage(subproto);
      static_cast<std::vector<GenericMessage *> *>(data)->push_back(submsg);
    }
  } else {
    data = nullptr;
  }

  return data;
}

void ProtoGenericMessage::set(void *data,
                              const google::protobuf::Reflection *reflection,
                              const google::protobuf::FieldDescriptor *field) {
  using namespace google::protobuf;

  int cpp_type = field->cpp_type();

  if (cpp_type == FieldDescriptor::CPPTYPE_INT32) {
    auto cast_data = static_cast<const int64_t *>(data);
    reflection->SetInt32(message_, field, static_cast<int32_t>(*cast_data));
  } else if (cpp_type == FieldDescriptor::CPPTYPE_UINT32) {
    auto cast_data = static_cast<const uint64_t *>(data);
    reflection->SetUInt32(message_, field, static_cast<uint32_t>(*cast_data));
  } else if (cpp_type == FieldDescriptor::CPPTYPE_INT64) {
    auto cast_data = static_cast<const int64_t *>(data);
    reflection->SetInt64(message_, field, *cast_data);
  } else if (cpp_type == FieldDescriptor::CPPTYPE_UINT64) {
    auto cast_data = static_cast<const uint64_t *>(data);
    reflection->SetUInt64(message_, field, *cast_data);
  } else if (cpp_type == FieldDescriptor::CPPTYPE_DOUBLE) {
    auto cast_data = static_cast<const double *>(data);
    reflection->SetDouble(message_, field, *cast_data);
  } else if (cpp_type == FieldDescriptor::CPPTYPE_FLOAT) {
    auto cast_data = static_cast<const double *>(data);
    reflection->SetFloat(message_, field, static_cast<float>(*cast_data));
  } else if (cpp_type == FieldDescriptor::CPPTYPE_BOOL) {
    auto cast_data = static_cast<const bool *>(data);
    reflection->SetBool(message_, field, *cast_data);
  } else if (cpp_type == FieldDescriptor::CPPTYPE_ENUM) {
    auto cast_data = static_cast<const int *>(data);
    reflection->SetEnumValue(message_, field, *cast_data);
  } else if (cpp_type == FieldDescriptor::CPPTYPE_STRING) {
    auto cast_data = static_cast<const std::string *>(data);
    reflection->SetString(message_, field, *cast_data);
  } else if (cpp_type == FieldDescriptor::CPPTYPE_MESSAGE) {
    auto generic = static_cast<GenericMessage *>(data);
    auto proto_msg = dynamic_cast<ProtoGenericMessage *>(generic);

    auto msg = proto_msg->get_message_reference();

    reflection->SetAllocatedMessage(message_, msg, field);

    proto_msg->released();
  }
}

void ProtoGenericMessage::set_repeated(
    void *data, const google::protobuf::Reflection *reflection,
    const google::protobuf::FieldDescriptor *field) {
  using namespace google::protobuf;

  int cpp_type = field->cpp_type();

  if (cpp_type == FieldDescriptor::CPPTYPE_INT32) {
    auto array = static_cast<std::vector<int64_t> *>(data);
    for (size_t x = 0; x < array->size(); x++) {
      reflection->AddInt32(message_, field, static_cast<int32_t>(array->at(x)));
    }
  } else if (cpp_type == FieldDescriptor::CPPTYPE_UINT32) {
    auto array = static_cast<std::vector<uint64_t> *>(data);

    for (size_t x = 0; x < array->size(); x++) {
      reflection->AddUInt32(message_, field,
                            static_cast<uint32_t>(array->at(x)));
    }
  } else if (cpp_type == FieldDescriptor::CPPTYPE_INT64) {
    auto array = static_cast<std::vector<int64_t> *>(data);

    for (size_t x = 0; x < array->size(); x++) {
      reflection->AddInt64(message_, field, array->at(x));
    }
  } else if (cpp_type == FieldDescriptor::CPPTYPE_UINT64) {
    auto array = static_cast<std::vector<uint64_t> *>(data);

    for (size_t x = 0; x < array->size(); x++) {
      reflection->AddUInt64(message_, field, array->at(x));
    }
  } else if (cpp_type == FieldDescriptor::CPPTYPE_DOUBLE) {
    auto array = static_cast<std::vector<double> *>(data);

    for (size_t x = 0; x < array->size(); x++) {
      reflection->AddDouble(message_, field, array->at(x));
    }
  } else if (cpp_type == FieldDescriptor::CPPTYPE_FLOAT) {
    auto array = static_cast<std::vector<double> *>(data);

    for (size_t x = 0; x < array->size(); x++) {
      reflection->AddFloat(message_, field, static_cast<float>(array->at(x)));
    }
  } else if (cpp_type == FieldDescriptor::CPPTYPE_BOOL) {
    auto array = static_cast<std::vector<bool> *>(data);

    for (size_t x = 0; x < array->size(); x++) {
      reflection->AddBool(message_, field, array->at(x));
    }
  } else if (cpp_type == FieldDescriptor::CPPTYPE_ENUM) {
    auto array = static_cast<std::vector<int32_t> *>(data);

    for (size_t x = 0; x < array->size(); x++) {
      reflection->AddEnumValue(message_, field, array->at(x));
    }
  } else if (cpp_type == FieldDescriptor::CPPTYPE_STRING) {
    auto array = static_cast<std::vector<std::string> *>(data);

    for (size_t x = 0; x < array->size(); x++) {
      reflection->AddString(message_, field, array->at(x));
    }
  } else if (cpp_type == FieldDescriptor::CPPTYPE_MESSAGE) {
    auto array = static_cast<std::vector<GenericMessage *> *>(data);

    for (size_t x = 0; x < array->size(); x++) {
      auto proto_msg = dynamic_cast<ProtoGenericMessage *>(array->at(x));

      auto msg = proto_msg->get_message_reference();

      reflection->AddAllocatedMessage(message_, field, msg);

      proto_msg->released();
    }
  }
}

Type ProtoGenericMessage::protoTypeToType(
    google::protobuf::FieldDescriptor::CppType protoType) {
  using namespace google::protobuf;

  int cpp_type = protoType;

  if (cpp_type == FieldDescriptor::CPPTYPE_INT32) {
    return XMSG::Type::INT;
  } else if (cpp_type == FieldDescriptor::CPPTYPE_UINT32) {
    return XMSG::Type::UINT;
  } else if (cpp_type == FieldDescriptor::CPPTYPE_INT64) {
    return XMSG::Type::INT;
  } else if (cpp_type == FieldDescriptor::CPPTYPE_UINT64) {
    return XMSG::Type::UINT;
  } else if (cpp_type == FieldDescriptor::CPPTYPE_DOUBLE) {
    return XMSG::Type::DOUBLE;
  } else if (cpp_type == FieldDescriptor::CPPTYPE_FLOAT) {
    return XMSG::Type::DOUBLE;
  } else if (cpp_type == FieldDescriptor::CPPTYPE_BOOL) {
    return XMSG::Type::BOOL;
  } else if (cpp_type == FieldDescriptor::CPPTYPE_ENUM) {
    return XMSG::Type::INT;
  } else if (cpp_type == FieldDescriptor::CPPTYPE_STRING) {
    return XMSG::Type::STRING;
  } else if (cpp_type == FieldDescriptor::CPPTYPE_MESSAGE) {
    return XMSG::Type::GENERIC;
  }

  return XMSG::Type::NOTSET;
}
