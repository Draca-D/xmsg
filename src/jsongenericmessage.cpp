#include "xmsg/jsongenericmessage.h"

using namespace XMSG;
using json = nlohmann::json;

JSONGenericMessage::JSONGenericMessage(const std::string &json_data) {
  set_identifier("JSONMessage");

  json_ = json::parse(json_data);
}

JSONGenericMessage::JSONGenericMessage() {
  set_identifier("JSONMessage");
  set_type("Unset");

  json_ = nlohmann::json::object();

  set_formless(true);
}

JSONGenericMessage::~JSONGenericMessage() {}

const XMSG::member
JSONGenericMessage::get_member(const std::string &member_name,
                               const Type expected_type) {
  (void)expected_type; // may find use in the future
  try {
    auto value = find_member(member_name);

    auto type = member_to_type(value);

    if (type & XMSG::ARRAY) {
      auto data = extract_array(value, find_member_key(member_name));
      return {data, type};
    } else {
      auto data = extract(value, find_member_key(member_name));
      return {data, type};
    }
  } catch (XMSG::BrokenMessage &) {
    throw;
  } catch (std::exception &e) {
    log("thrown: " + std::string(e.what()), XMSG::LogLevel::DEBUG);
    return {nullptr, XMSG::NOTSET};
  }
}

void JSONGenericMessage::set_member(const std::string &member_name,
                                    XMSG::member new_data) {
  if (new_data.type & XMSG::Type::ARRAY) {
    json_[member_name] = nlohmann::json::array();

    auto type = new_data.type;
    auto data = new_data.data;

    if (type & XMSG::Type::CHAR) {
      auto cast_data = static_cast<std::vector<int64_t> *>(data);
      for (auto &val : *cast_data) {
        json_[member_name].push_back(val);
      }
    } else if (type & XMSG::Type::INT) {
      auto cast_data = static_cast<std::vector<int64_t> *>(data);
      for (auto &val : *cast_data) {
        json_[member_name].push_back(val);
      }
    } else if (type & XMSG::Type::UINT) {
      auto cast_data = static_cast<std::vector<uint64_t> *>(data);
      for (auto &val : *cast_data) {
        json_[member_name].push_back(val);
      }
    } else if (type & XMSG::Type::DOUBLE) {
      auto cast_data = static_cast<std::vector<double> *>(data);
      for (auto &val : *cast_data) {
        json_[member_name].push_back(val);
      }
    } else if (type & XMSG::Type::STRING) {
      auto cast_data = static_cast<std::vector<std::string> *>(data);
      for (auto &val : *cast_data) {
        json_[member_name].push_back(val);
      }
    }
    //    else if (type & XMSG::Type::BOOL) { // array of booleans doesnt appear
    //    to be a thing in JSON? Code left here as comment for completness
    //      auto cast_data = static_cast<std::vector<bool>*>(data);
    //      for(auto val:*cast_data){
    //        json_[member_name].push_back(val);
    //      }
    //    }
    else if (type & XMSG::Type::GENERIC) {
      auto generic = static_cast<std::vector<GenericMessage *> *>(data);
      for (auto &val : *generic) {
        auto jsonmsg = dynamic_cast<JSONGenericMessage *>(val);
        json_[member_name].push_back(jsonmsg->json_);
      }
    }
  } else {
    json_[member_name] = member_to_json(new_data);
  }
}

void JSONGenericMessage::copy_from(GenericMessage *message) {
  log("Creating new JSON string from '" + message->get_identifier() +
          "' which has type '" + message->get_type() + "'",
      XMSG::LogLevel::INFO, XMSG::OnceType::UNIQUEONLY);

  XMSG::GenericMessage::copy_from(message);

  auto members = this->get_members();

  for (auto &member : members) {
    std::string path = "/" + member.member_name;
    this->set_current_path_to(path);

    if (!member_exists(member.member_name)) {
      continue;
    }

    auto j_temp = json_;
    json_ = json_[member.member_name];

    post_parse();

    j_temp[member.member_name] = json_;
    json_ = j_temp;
  }
}

void JSONGenericMessage::parse() {
  this->pre_parse();

  enumerate_members();
}

void JSONGenericMessage::enumerate_members() {
  for (auto &[key, value] : json_.items()) {
    auto name = key;
    auto type_name = std::string("basic type");
    auto type = member_to_type(value);

    if (type & XMSG::Type::GENERIC) {
      if (type & XMSG::Type::ARRAY) {
        type_name = value.front().dump();
      } else {
        type_name = value.dump();
      }
    }
    add_member(name, type, type_name);
  }
}

Type JSONGenericMessage::member_to_type(const nlohmann::json &member) {
  XMSG::Type type = XMSG::NONE;

  nlohmann::json sub = member;

  if (member.is_array()) {
    type = static_cast<XMSG::Type>(static_cast<int>(type) |
                                   static_cast<int>(XMSG::Type::ARRAY));
    if (sub.empty()) {
      return XMSG::NOTSET;
    }
    sub = member.front();
  }

  if (sub.is_object()) {
    type = static_cast<XMSG::Type>(static_cast<int>(type) |
                                   static_cast<int>(XMSG::Type::GENERIC));
  }

  if (sub.is_number_float()) {
    type = static_cast<XMSG::Type>(static_cast<int>(type) |
                                   static_cast<int>(XMSG::Type::DOUBLE));
  }

  if (sub.is_number_integer()) {
    if (sub.is_number_unsigned()) {
      type = static_cast<XMSG::Type>(static_cast<int>(type) |
                                     static_cast<int>(XMSG::Type::UINT));
    } else {
      type = static_cast<XMSG::Type>(static_cast<int>(type) |
                                     static_cast<int>(XMSG::Type::INT));
    }
  }

  if (sub.is_boolean()) {
    type = static_cast<XMSG::Type>(static_cast<int>(type) |
                                   static_cast<int>(XMSG::Type::BOOL));
  }

  if (sub.is_string()) {
    type = static_cast<XMSG::Type>(static_cast<int>(type) |
                                   static_cast<int>(XMSG::Type::STRING));
  }

  if (sub.is_null()) {
    type = static_cast<XMSG::Type>(static_cast<int>(type) |
                                   static_cast<int>(XMSG::Type::NOTSET));
  }

  return type;
}

void *JSONGenericMessage::extract(const nlohmann::json &member,
                                  const std::string &key) {
  void *data;

  if (member.is_number_integer()) {
    if (member.is_number_unsigned()) {
      data = calloc(1, sizeof(int64_t));
      static_cast<int64_t *>(data)[0] = member.get<int64_t>();
    } else {
      data = calloc(1, sizeof(uint64_t));
      static_cast<uint64_t *>(data)[0] = member.get<uint64_t>();
    }
  } else if (member.is_number_float()) {
    data = calloc(1, sizeof(double));
    static_cast<double *>(data)[0] = member.get<double>();
  } else if (member.is_boolean()) {
    data = calloc(1, sizeof(bool));
    static_cast<bool *>(data)[0] = member.get<bool>();
  } else if (member.is_string()) {
    data = new std::string;
    *static_cast<std::string *>(data) = member.get<std::string>();
  } else if (member.is_object()) {
    XMSG::GenericMessage *msg = NewGenericJson(member.dump(), true);
    auto current_path = get_current_path_to();
    auto new_path = current_path + "/" + key;
    dynamic_cast<JSONGenericMessage *>(msg)->set_current_path_to(new_path);
    dynamic_cast<JSONGenericMessage *>(msg)->parse();
    data = msg;
  } else {
    data = nullptr;
  }

  return data;
}

void *JSONGenericMessage::extract_array(const nlohmann::json &member,
                                        const std::string &key) {
  auto type = peak_at_type(member);
  if (type & XMSG::NOTSET) {
    return nullptr;
  }

  void *data;

  if (type & XMSG::Type::INT) {
    data = new std::vector<int64_t>;
    for (auto &sub : member) {
      static_cast<std::vector<int64_t> *>(data)->push_back(sub.get<int64_t>());
    }
  } else if (type & XMSG::Type::UINT) {
    data = new std::vector<uint64_t>;
    for (auto &sub : member) {
      static_cast<std::vector<uint64_t> *>(data)->push_back(
          sub.get<uint64_t>());
    }
  } else if (type & XMSG::Type::DOUBLE) {
    data = new std::vector<double>;
    for (auto &sub : member) {
      static_cast<std::vector<double> *>(data)->push_back(sub.get<double>());
    }
  } else if (type & XMSG::Type::BOOL) {
    data = new std::vector<bool>;
    for (auto &sub : member) {
      static_cast<std::vector<bool> *>(data)->push_back(sub.get<bool>());
    }
  } else if (type & XMSG::Type::STRING) {
    data = new std::vector<std::string>;
    for (auto &sub : member) {
      static_cast<std::vector<std::string> *>(data)->push_back(
          sub.get<std::string>());
    }
  } else if (type & XMSG::Type::GENERIC) {
    data = new std::vector<XMSG::GenericMessage *>;
    for (auto &sub : member) {
      XMSG::GenericMessage *msg = NewGenericJson(sub.dump(), true);
      auto current_path = get_current_path_to();
      auto new_path = current_path + "/" + key;
      dynamic_cast<JSONGenericMessage *>(msg)->set_current_path_to(new_path);
      dynamic_cast<JSONGenericMessage *>(msg)->parse();
      static_cast<std::vector<XMSG::GenericMessage *> *>(data)->push_back(msg);
    }
  } else {
    data = nullptr;
  }

  return data;
}

Type JSONGenericMessage::peak_at_type(const nlohmann::json &member) {
  if (member.empty()) {
    return XMSG::Type::NOTSET;
  }

  auto type = member_to_type(member.front());

  return type;
}

nlohmann::json JSONGenericMessage::find_member(const std::string &member) {
  for (auto &[key, value] : json_.items()) {
    if (compare(member, key)) {
      return value;
    }
  }

  throw std::invalid_argument("Unable to find member '" + member + "'");
}

nlohmann::json JSONGenericMessage::member_to_json(const member &member) {
  auto type = member.type;
  auto data = member.data;

  auto j = nlohmann::json::object();

  if (type & XMSG::Type::CHAR) {
    auto cast_data = static_cast<int64_t *>(data);
    j = *cast_data;
  } else if (type & XMSG::Type::INT) {
    auto cast_data = static_cast<int64_t *>(data);
    j = *cast_data;
  } else if (type & XMSG::Type::UINT) {
    auto cast_data = static_cast<uint64_t *>(data);
    j = *cast_data;
  } else if (type & XMSG::Type::DOUBLE) {
    auto cast_data = static_cast<double *>(data);
    j = *cast_data;
  } else if (type & XMSG::Type::STRING) {
    auto cast_data = static_cast<std::string *>(data);
    j = *cast_data;
  } else if (type & XMSG::Type::BOOL) {
    auto cast_data = static_cast<bool *>(data);
    j = *cast_data;
  } else if (type & XMSG::Type::GENERIC) {
    auto generic = static_cast<GenericMessage *>(data);
    auto jsonmsg = dynamic_cast<JSONGenericMessage *>(generic);

    j = jsonmsg->json_;
  }

  return j;
}

std::string JSONGenericMessage::find_member_key(const std::string &member) {
  for (auto &[key, value] : json_.items()) {
    if (compare(member, key)) {
      return key;
    }
  }

  throw std::invalid_argument("Unable to find key for member '" + member + "'");
}

bool JSONGenericMessage::member_exists(const std::string &member_name) {
  for (auto &[key, value] : json_.items()) {
    if (compare(key, member_name)) {
      return true;
    }
  }

  return false;
}

std::string JSONGenericMessage::get_parent_name() {
  auto pos = path_to_.rfind("/");
  if (pos != std::string::npos) {
    return path_to_.substr(pos + 1);
  }

  return "/";
}
