#include "xmsg/genericmessage.h"

using namespace XMSG;

GenericMessage::GenericMessage(const GenericMessage &msg) { copy_from(&msg); }

XMSG::GenericMessage &GenericMessage::operator=(GenericMessage &message) {
  copy_from(&message);
  return *this;
}

void GenericMessage::copy_from(const GenericMessage *const message) {
  if (formless_) {
    members_ = message->get_members();
  }

  for (auto &member : members_) {
    auto name = member.member_name;
    auto type = member.type;
    auto type_id = member.type_identifier;

    auto mapped_name = get_mapped_name(name);
    auto transform = get_transformation(name);

    log("Looking for member: " + name + "[mapped to: " + mapped_name +
            "] with type: " + type_to_str(type) + "[" + type_id + "]",
        XMSG::LogLevel::INFO, XMSG::OnceType::UNIQUEONLY);

    auto new_member = message->get_member(mapped_name, type);

    if (!(type & XMSG::Type::GENERIC) &&
        new_member.type_ & XMSG::Type::GENERIC && transform.empty() &&
        auto_extract_child_) {
      GenericMessage *msg = nullptr;

      if (new_member.type_ & XMSG::Type::ARRAY) {
        auto vec =
            static_cast<std::vector<GenericMessage *> *>(new_member.data_);
        if (!vec->empty()) {
          msg = vec->front(); // peek into vector to see what type of message it
                              // is // assumption is that all members of the
                              // vector will be of the same type
        }
      } else {
        msg = static_cast<GenericMessage *>(new_member.data_);
      }

      if (msg != nullptr && msg->is_simple()) {
        auto child = msg->get_members().front().member_name;
        transform =
            "GET(" + child + ")"; // if the type we are converting to is a base
                                  // type, no transformation has been set
        // and the type we are converting from is of a complex type that only
        // has one child member, auto extract that child, given that
        // auto_extract_child_ has been set
        log("Will auto extract '" + child + "' from message '" +
                message->get_type() +
                "' to satisfy needs of current message '" + name +
                "' which is a nested type containing only one member",
            XMSG::LogLevel::INFO, XMSG::OnceType::UNIQUEONLY);
      }
    }

    new_member = transform_member(new_member, transform);

    if (new_member.type_ & XMSG::NOTSET) {
      std::string str = "Message '" + get_type() + "' of generic type '" +
                        get_identifier() + "' is looking for '" + name +
                        "' which is not set in message '" +
                        message->get_type() + "' of generic type '" +
                        message->get_identifier() + "'";
      log(str, XMSG::LogLevel::WARN, XMSG::OnceType::UNIQUEONLY);
      continue;
    }

    if (member.type != new_member.type_) {
      if (is_numerical_type(member.type) &&
          is_numerical_type(new_member.type_)) {
        std::string mismatch =
            "Current member[" + name + "] " + "has type: '" +
            member.type_as_str +
            "'. Member from message we are converting from has type: '" +
            type_to_str(new_member.type_) +
            "'. Will set members but casting errors may be present";
        log(mismatch, XMSG::LogLevel::WARN, XMSG::OnceType::UNIQUEONLY);
        new_member.modify_type(member.type);
      } else {
        auto child = access_requested_child_member(name);

        if (child.empty() && member.type & XMSG::Type::GENERIC &&
            auto_set_child_) {
          GenericMessage *new_msg;

          if (!formless_) {
            new_msg = this->New(type_id);
          } else {
            new_msg = this->NewFormless();
          }

          if (new_msg->is_simple()) {
            child = new_msg->get_members().front().member_name;
            log("Will auto set member with name '" + child +
                    "', which is the only member of '" + new_msg->get_type() +
                    "' using incoming message",
                XMSG::LogLevel::INFO, XMSG::OnceType::UNIQUEONLY);
          }

          delete new_msg;
        }

        if (member.type & XMSG::Type::GENERIC && !child.empty()) {
          if (member.type & XMSG::Type::ARRAY &&
              !(new_member.type_ & XMSG::Type::ARRAY)) {
            std::string mismatch =
                "Current member[" + name + "] " +
                " is an array, but the member we are converting from [" +
                mapped_name +
                "] is not an array. Unable to set requested child member, "
                "ignoring";
            log(mismatch, XMSG::LogLevel::WARN, XMSG::OnceType::UNIQUEONLY);

            continue;
          }

          if (!(member.type & XMSG::Type::ARRAY) &&
              (new_member.type_ & XMSG::Type::ARRAY)) {
            std::string mismatch =
                "Current member[" + name + "] " +
                " is not array, but the member we are converting from [" +
                mapped_name +
                "] is an array. Unable to set requested child member, "
                "ignoring";
            log(mismatch, XMSG::LogLevel::WARN, XMSG::OnceType::UNIQUEONLY);

            continue;
          }

          if (member.type & XMSG::Type::ARRAY) {
            auto vec = create_single_child_populated_msg_array(
                type_id, name, child, new_member);

            XMSG::RawMemberWrapper memb;
            memb.data_ = vec;
            memb.type_ = type;

            set_member(name, memb);

            continue;

          } else {
            auto new_msg = create_single_child_populated_msg(type_id, name,
                                                             child, new_member);
            if (new_msg == nullptr) {
              continue;
            }

            XMSG::RawMemberWrapper memb;
            memb.data_ = new_msg;
            memb.type_ = type;

            set_member(name, memb);

            continue;
          }
        }

        std::string mismatch =
            "Current member[" + name + "] " + "has type: '" +
            member.type_as_str +
            "'. Member from message we are converting from has type: '" +
            type_to_str(new_member.type_) + "'. Will not set member";
        log(mismatch, XMSG::LogLevel::ERROR, XMSG::OnceType::UNIQUEONLY);

        if (new_member.type_ == XMSG::Type::STRING) {
          auto str = static_cast<std::string *>(new_member.data_);
          log("New member is a string with value: " + *str +
                  " and has type: " + message->type_,
              XMSG::LogLevel::ERROR, XMSG::OnceType::UNIQUEONLY);
        }
        continue;
      }
    }

    if (new_member.type_ & XMSG::Type::GENERIC) {
      if (new_member.type_ & XMSG::Type::ARRAY) {
        auto nested_msg = static_cast<std::vector<XMSG::GenericMessage *> *>(
            new_member.data_);

        auto vec = new std::vector<GenericMessage *>;

        for (size_t x = 0; x < nested_msg->size(); x++) {
          GenericMessage *new_msg;
          if (!formless_) {
            new_msg = this->New(type_id);
          } else {
            new_msg = this->NewFormless();
          }
          new_msg->set_helpful_name(name);
          new_msg->copy_from(nested_msg->at(x));
          vec->push_back(new_msg);
        }

        XMSG::RawMemberWrapper memb;
        memb.data_ = vec;
        memb.type_ = type;

        set_member(name, memb);
      } else {
        GenericMessage *new_msg;

        if (!formless_) {
          new_msg = this->New(type_id);
        } else {
          new_msg = this->NewFormless();
        }

        new_msg->set_helpful_name(name);
        auto nested_msg = static_cast<XMSG::GenericMessage *>(new_member.data_);

        new_msg->copy_from(nested_msg);

        XMSG::RawMemberWrapper memb;
        memb.data_ = new_msg;
        memb.type_ = type;

        set_member(name, memb);
      }
    } else {
      set_member(name, new_member);
    }
  }
}

void GenericMessage::log_members() {
  log("==== Listing members ====", XMSG::LogLevel::INFO);

  std::string str = "\n";
  str += all_members_to_str(0);

  log(str, XMSG::LogLevel::INFO);

  log("==== Done Listing ====", XMSG::LogLevel::INFO);
}

std::string GenericMessage::all_members_to_str(int depth) {
  std::string pad;

  for (int x = 0; x < depth; x++) {
    pad += " ";
  }

  std::string str_members;

  for (auto &[member, type, str, type_id] : members_) {
    std::string line = pad + "Name: " + member;

    if (type & XMSG::Type::GENERIC) {
      line += "[" + str + "]:\n";
      str_members += line;

      GenericMessage *submsg;

      if (!formless_) {
        submsg = this->New(type_id);
      } else {
        submsg = this->NewFormless();
      }

      auto sub_str = submsg->all_members_to_str(depth + 1);

      str_members += sub_str;
      delete submsg;
    } else {
      line += " of type " + std::to_string(type) + "[" + str + "][" + type_id +
              "]\n";
      str_members += line;
    }
  }

  return str_members;
}

std::string GenericMessage::get_mapped_name(const std::string &member_name) {
  if (configuration_) {
    return configuration_->get_mapped_name(this->get_identifier(),
                                           this->get_type(), member_name);
  }

  return member_name;
}

std::string GenericMessage::get_transformation(const std::string &member_name) {
  if (configuration_) {
    return configuration_->get_transformation(this->get_identifier(),
                                              this->get_type(), member_name);
  }

  return std::string();
}

std::string
GenericMessage::access_requested_child_member(const std::string &member_name) {
  if (configuration_) {
    return configuration_->check_if_child(this->get_identifier(),
                                          this->get_type(), member_name);
  }

  return std::string();
}

RawMemberWrapper
GenericMessage::transform_member(RawMemberWrapper &member,
                                 const std::string &transformation) {
  RawMemberWrapper ret;

  if (transformation.empty()) {
    ret = std::move(member);
    return ret;
  }

  if (transformation.compare("NaN") == 0) {
    auto data = calloc(1, sizeof(double));
    static_cast<double *>(data)[0] = std::nan("");

    ret.data_ = data;
    ret.type_ = XMSG::Type::DOUBLE;

    return ret;
  }

  std::regex constant_check("^(\\s+)?(\\d+)(\\.\\d+)?(\\s+)?$");
  std::smatch value_match;

  if (std::regex_search(transformation, value_match,
                        constant_check)) { // checks if its just a constant
    auto int_value = value_match.str(2);
    auto dec_value = value_match.str(3);

    if (dec_value.empty()) {
      auto data = calloc(1, sizeof(int64_t));
      static_cast<int64_t *>(data)[0] = std::stoi(int_value);

      ret.data_ = data;
      ret.type_ = XMSG::Type::INT;

      return ret;
    } else {
      auto data = calloc(1, sizeof(double));
      static_cast<double *>(data)[0] = std::stod(int_value + dec_value);

      ret.data_ = data;
      ret.type_ = XMSG::Type::DOUBLE;

      return ret;
    }
  }

  if (!is_transformable(member)) {
    log("Member has transformation listed in configuration but member is not "
        "transformable. Will "
        "ignore transformation, type is " +
            type_to_str(member.type_),
        XMSG::LogLevel::WARN, XMSG::OnceType::UNIQUEONLY);
    ret = std::move(member);
    return ret;
  }

  log("Transforming recieved member prior to setting value. Requested "
      "transformation is: " +
          transformation,
      XMSG::LogLevel::INFO, XMSG::OnceType::UNIQUEONLY);

  std::regex re(
      "((GET)\\((\\w+)\\))?((^|\\s)(\\*|\\+|-|\\/)\\s(-?\\d+(\\.\\d+)?))?");
  std::smatch match;

  if (!std::regex_search(transformation, match,
                         re)) { // checks if any matches was made
    log("Completely Invalid process given '" + transformation +
            "' - Will ignore transformation",
        XMSG::LogLevel::ERROR, XMSG::OnceType::UNIQUEONLY);
    ret = std::move(member);
    return ret;
  }

  if (match.str(0).compare(transformation) !=
      0) { // part of the transformation was matched, but the whole line needs
           // to be matched, there should be a full line option for regex but
           // this is easier
    log("Partially Invalid process given '" + transformation +
            "'. Double check configuration - Will ignore transformation",
        XMSG::LogLevel::ERROR, XMSG::OnceType::UNIQUEONLY);
    ret = std::move(member);
    return ret;
  }

  auto accessor = match.str(2);        // Should be GET
  auto sub_member_name = match.str(3); // GET(x) -> access name = x
  auto math_operator = match.str(6);   // * + - /
  auto math_term =
      match.str(7); // will either be an int or a decimal i.e., 3 or 2.16
  auto term_decimal =
      match.str(8); // decimal component including the '.' if decimal value.
                    // Will be empty otherwise, useful for determining if this
                    // is a decimal number or not

  if (!accessor.empty()) { // extract sub member if GET was given
    if (accessor.compare("GET") != 0) {
      log("Requested accessor was labelled as '" + accessor +
              "', only supported type is GET...Ignoring transformation. this "
              "shouldve been "
              "caught by the regex validator so check this",
          XMSG::LogLevel::ERROR, XMSG::OnceType::UNIQUEONLY);
      ret = std::move(member);
      return ret;
    }

    if (sub_member_name.empty()) {
      log("Requested access to sub member but no sub member name was provided, "
          "this shouldve "
          "been caught by the regex validator so check this",
          XMSG::LogLevel::ERROR, XMSG::OnceType::UNIQUEONLY);
      ret = std::move(member);
      return ret;
    }

    if (member.type_ & XMSG::Type::GENERIC) {
      auto sub_type = static_cast<GenericMessage *>(member.data_);
      auto sub_member = sub_type->get_member(sub_member_name, XMSG::Type::NONE);

      if (sub_member.type_ & XMSG::Type::NOTSET) {
        log("Accessor requested member " + sub_member_name + " from " +
                sub_type->get_type() + " but " + sub_type->get_type() +
                " does not contain a member with this name...Ignoring "
                "transformation",
            XMSG::LogLevel::ERROR, XMSG::OnceType::UNIQUEONLY);
        ret = std::move(member);
        return ret;
      }

      ret = std::move(sub_member);

    } else {
      log("Invalid accessor request. "
          "Transformation requested access to sub member from complex type, "
          "but member is of type: " +
              type_to_str(member.type_) + "... Ignoring transformation",
          XMSG::LogLevel::ERROR, XMSG::OnceType::UNIQUEONLY);
      ret = std::move(member);
      return ret;
    }
  }

  if (!math_operator.empty()) {
    auto type = member.type_;

    if (type & XMSG::Type::CHAR) {
      using tp = int64_t;

      if (math_operator.compare("+") == 0) {
        member.transformation_handle_addition<tp>(math_term, term_decimal);
      } else if (math_operator.compare("-") == 0) {
        member.transformation_handle_subtraction<tp>(math_term, term_decimal);
      } else if (math_operator.compare("*") == 0) {
        member.transformation_handle_multiplication<tp>(math_term,
                                                        term_decimal);
      } else if (math_operator.compare("/") == 0) {
        member.transformation_handle_division<tp>(math_term, term_decimal);
      } else {
        log("Unrecognised operator '" + math_operator + "'",
            XMSG::LogLevel::ERROR, XMSG::OnceType::UNIQUEONLY);
      }
    } else if (type & XMSG::Type::INT) {
      using tp = int64_t;

      if (math_operator.compare("+") == 0) {
        member.transformation_handle_addition<tp>(math_term, term_decimal);
      } else if (math_operator.compare("-") == 0) {
        member.transformation_handle_subtraction<tp>(math_term, term_decimal);
      } else if (math_operator.compare("*") == 0) {
        member.transformation_handle_multiplication<tp>(math_term,
                                                        term_decimal);
      } else if (math_operator.compare("/") == 0) {
        member.transformation_handle_division<tp>(math_term, term_decimal);
      } else {
        log("Unrecognised operator '" + math_operator + "'",
            XMSG::LogLevel::ERROR, XMSG::OnceType::UNIQUEONLY);
      }
    } else if (type & XMSG::Type::UINT) {
      using tp = uint64_t;

      if (math_operator.compare("+") == 0) {
        member.transformation_handle_addition<tp>(math_term, term_decimal);
      } else if (math_operator.compare("-") == 0) {
        member.transformation_handle_subtraction<tp>(math_term, term_decimal);
      } else if (math_operator.compare("*") == 0) {
        member.transformation_handle_multiplication<tp>(math_term,
                                                        term_decimal);
      } else if (math_operator.compare("/") == 0) {
        member.transformation_handle_division<tp>(math_term, term_decimal);
      } else {
        log("Unrecognised operator '" + math_operator + "'",
            XMSG::LogLevel::ERROR, XMSG::OnceType::UNIQUEONLY);
      }
    } else if (type & XMSG::Type::DOUBLE) {
      using tp = double;

      if (math_operator.compare("+") == 0) {
        member.transformation_handle_addition<tp>(math_term, term_decimal);
      } else if (math_operator.compare("-") == 0) {
        member.transformation_handle_subtraction<tp>(math_term, term_decimal);
      } else if (math_operator.compare("*") == 0) {
        member.transformation_handle_multiplication<tp>(math_term,
                                                        term_decimal);
      } else if (math_operator.compare("/") == 0) {
        member.transformation_handle_division<tp>(math_term, term_decimal);
      } else {
        log("Unrecognised operator '" + math_operator + "'",
            XMSG::LogLevel::ERROR, XMSG::OnceType::UNIQUEONLY);
      }
    } else if (type & XMSG::Type::BOOL) {
      using tp = bool;

      if (math_operator.compare("+") == 0) {
        member.transformation_handle_addition<tp>(math_term, term_decimal);
      } else if (math_operator.compare("-") == 0) {
        member.transformation_handle_subtraction<tp>(math_term, term_decimal);
      } else if (math_operator.compare("*") == 0) {
        member.transformation_handle_multiplication<tp>(math_term,
                                                        term_decimal);
      } else if (math_operator.compare("/") == 0) {
        member.transformation_handle_division<tp>(math_term, term_decimal);
      } else {
        log("Unrecognised operator '" + math_operator + "'",
            XMSG::LogLevel::ERROR, XMSG::OnceType::UNIQUEONLY);
      }
    } else {
      throw std::invalid_argument("Not a transformable type");
    }
  }

  return ret;
}

XMSG::GenericMessage *GenericMessage::create_single_child_populated_msg(
    const std::string &type_id, const std::string &name,
    const std::string &child, XMSG::RawMemberWrapper &new_member) {
  GenericMessage *new_msg;

  if (!formless_) {
    new_msg = this->New(type_id);
  } else {
    new_msg = this->NewFormless();
  }

  if (!new_msg->has_member(child)) {
    delete new_msg;
    std::string mismatch = "Current member[" + name + "] " +
                           "Does not contain a member with name [" + child +
                           "], but a request to set said member was made";
    log(mismatch, XMSG::LogLevel::WARN, XMSG::OnceType::UNIQUEONLY);

    return nullptr;
  }

  auto child_type = new_msg->get_member_type(child);

  if (child_type != new_member.type_) {
    if (is_numerical_type(child_type) && is_numerical_type(new_member.type_)) {
      std::string mismatch =
          "Current member[" + name + "/" + child + "] " + "has type: '" +
          type_to_str(child_type) +
          "'. Member from message we are converting from has type: '" +
          type_to_str(new_member.type_) +
          "'. Will set members but casting errors may be present";
      log(mismatch, XMSG::LogLevel::WARN, XMSG::OnceType::UNIQUEONLY);
      new_member.modify_type(child_type);

      new_msg->set_member(child, new_member);

      return new_msg;

    } else {
      return nullptr;
    }
  } else {
    new_msg->set_member(child, new_member);
    return new_msg;
  }
}

std::vector<XMSG::GenericMessage *> *
GenericMessage::create_single_child_populated_msg_array(
    const std::string &type_id, const std::string &name,
    const std::string &child, XMSG::RawMemberWrapper &new_member) {
  auto vec = new std::vector<GenericMessage *>;

  if (new_member.type_ & XMSG::Type::INT) {
    using tp = int64_t;
    auto nested_msg = static_cast<std::vector<tp> *>(new_member.data_);
    for (auto &val : *nested_msg) {
      auto memb_val = calloc(1, sizeof(tp));
      static_cast<tp *>(memb_val)[0] = val;

      XMSG::RawMemberWrapper memb;

      memb.data_ = memb_val;
      memb.type_ = new_member.type_;

      auto new_msg =
          create_single_child_populated_msg(type_id, name, child, memb);

      if (new_msg == nullptr) {
        break;
      }

      vec->push_back(new_msg);
    }
  } else if (new_member.type_ & XMSG::Type::UINT) {
    using tp = uint64_t;
    auto nested_msg = static_cast<std::vector<tp> *>(new_member.data_);
    for (auto &val : *nested_msg) {
      auto memb_val = calloc(1, sizeof(tp));
      static_cast<tp *>(memb_val)[0] = val;
      XMSG::RawMemberWrapper memb;

      memb.data_ = memb_val;
      memb.type_ = new_member.type_;

      auto new_msg =
          create_single_child_populated_msg(type_id, name, child, memb);

      if (new_msg == nullptr) {
        break;
      }

      vec->push_back(new_msg);
    }
  } else if (new_member.type_ & XMSG::Type::BOOL) {
    using tp = bool;
    auto nested_msg = static_cast<std::vector<tp> *>(new_member.data_);
    for (size_t x = 0; x < nested_msg->size(); x++) {
      auto val = nested_msg->at(x);
      auto memb_val = calloc(1, sizeof(tp));
      static_cast<tp *>(memb_val)[0] = val;
      XMSG::RawMemberWrapper memb;

      memb.data_ = memb_val;
      memb.type_ = new_member.type_;

      auto new_msg =
          create_single_child_populated_msg(type_id, name, child, memb);

      if (new_msg == nullptr) {
        break;
      }

      vec->push_back(new_msg);
    }
  } else if (new_member.type_ & XMSG::Type::DOUBLE) {
    using tp = double;
    auto nested_msg = static_cast<std::vector<tp> *>(new_member.data_);
    for (auto &val : *nested_msg) {
      auto memb_val = calloc(1, sizeof(tp));
      static_cast<tp *>(memb_val)[0] = val;
      XMSG::RawMemberWrapper memb;

      memb.data_ = memb_val;
      memb.type_ = new_member.type_;

      auto new_msg =
          create_single_child_populated_msg(type_id, name, child, memb);

      if (new_msg == nullptr) {
        break;
      }

      vec->push_back(new_msg);
    }
  } else if (new_member.type_ & XMSG::Type::STRING) {
    using tp = std::string;
    auto nested_msg = static_cast<std::vector<tp> *>(new_member.data_);
    for (auto &val : *nested_msg) {
      auto memb_val = calloc(1, sizeof(tp));
      static_cast<tp *>(memb_val)[0] = val;
      XMSG::RawMemberWrapper memb;

      memb.data_ = memb_val;
      memb.type_ = new_member.type_;

      auto new_msg =
          create_single_child_populated_msg(type_id, name, child, memb);

      if (new_msg == nullptr) {
        break;
      }

      vec->push_back(new_msg);
    }
  } else if (new_member.type_ & XMSG::Type::CHAR) {
    using tp = int64_t;
    auto nested_msg = static_cast<std::vector<tp> *>(new_member.data_);
    for (auto &val : *nested_msg) {
      auto memb_val = calloc(1, sizeof(tp));
      static_cast<tp *>(memb_val)[0] = val;
      XMSG::RawMemberWrapper memb;

      memb.data_ = memb_val;
      memb.type_ = new_member.type_;

      auto new_msg =
          create_single_child_populated_msg(type_id, name, child, memb);

      if (new_msg == nullptr) {
        break;
      }

      vec->push_back(new_msg);
    }
  }

  return vec;
}

Type GenericMessage::get_member_type(const std::string &name) {
  for (auto &member : members_) {
    if (compare(member.member_name, name)) {
      return member.type;
    }
  }

  return XMSG::Type::NONE;
}

size_t GenericMessage::get_member_count() { return members_.size(); }

bool GenericMessage::lax_compare(const std::string &str1,
                                 const std::string &str2) const noexcept {
  auto str1_ = to_lower(str1);
  auto str2_ = to_lower(str2);

  str1_ = remove_character(str1_, '_');
  str2_ = remove_character(str2_, '_');

  //  printf("%s:%s:%d\n",str1.c_str(), str2.c_str(), str1.compare(str2));

  return (str1_.compare(str2_) == 0);
}

bool GenericMessage::is_transformable(
    const RawMemberWrapper &member) const noexcept {
  if (member.type_ & (XMSG::Type::NOTSET | XMSG::Type::STRING |
                      XMSG::Type::NONE | XMSG::Type::ARRAY)) {
    return false;
  }

  return true;
}

bool GenericMessage::has_member(const std::string &member_name) const noexcept {
  for (auto &member : members_) {
    if (compare(member.member_name, member_name)) {
      return true;
    }
  }

  return false;
}

std::string GenericMessage::to_lower(std::string str) const noexcept {
  std::transform(str.begin(), str.end(), str.begin(),
                 [](unsigned char c) { return std::tolower(c); });
  return str;
}

std::string GenericMessage::to_upper(std::string str) const noexcept {
  std::transform(str.begin(), str.end(), str.begin(),
                 [](unsigned char c) { return std::toupper(c); });
  return str;
}

std::string GenericMessage::remove_character(std::string str,
                                             char c) const noexcept {
  str.erase(std::remove(str.begin(), str.end(), c), str.cend());
  return str;
}

void GenericMessage::log(const std::string &message, LogLevel level,
                         OnceType once) {
  if (once == XMSG::OnceType::UNIQUEONLY) {
    if (std::find(logged_messages_.begin(), logged_messages_.end(), message) !=
        logged_messages_.end()) {
      return;
    }

    logged_messages_.push_back(message);
  }
  auto type = type_;
  constexpr int max = 20;
  if (type.size() > max) {
    type = type.substr(0, max) + "...";
  }

  switch (level) {
  case XMSG::LogLevel::DEBUG:
    if (once == XMSG::OnceType::ROS) {
      RCLCPP_DEBUG_ONCE(rclcpp::get_logger(identifier_.c_str()), "[%s] %s",
                        type.c_str(), message.c_str());
    } else {
      RCLCPP_DEBUG(rclcpp::get_logger(identifier_.c_str()), "[%s] %s",
                   type.c_str(), message.c_str());
    }
    break;
  case XMSG::LogLevel::INFO:
    if (once == XMSG::OnceType::ROS) {
      RCLCPP_INFO_ONCE(rclcpp::get_logger(identifier_.c_str()), "[%s] %s",
                       type.c_str(), message.c_str());
    } else {
      RCLCPP_INFO(rclcpp::get_logger(identifier_.c_str()), "[%s] %s",
                  type.c_str(), message.c_str());
    }
    break;
  case XMSG::LogLevel::WARN:
    if (once == XMSG::OnceType::ROS) {
      RCLCPP_WARN_ONCE(rclcpp::get_logger(identifier_.c_str()), "[%s] %s",
                       type.c_str(), message.c_str());
    } else {
      RCLCPP_WARN(rclcpp::get_logger(identifier_.c_str()), "[%s] %s",
                  type.c_str(), message.c_str());
    }
    break;
  case XMSG::LogLevel::ERROR:
    if (once == XMSG::OnceType::ROS) {
      RCLCPP_ERROR_ONCE(rclcpp::get_logger(identifier_.c_str()), "[%s] %s",
                        type.c_str(), message.c_str());
    } else {
      RCLCPP_ERROR(rclcpp::get_logger(identifier_.c_str()), "[%s] %s",
                   type.c_str(), message.c_str());
    }
    break;
  case XMSG::LogLevel::FATAL:
    if (once == XMSG::OnceType::ROS) {
      RCLCPP_FATAL_ONCE(rclcpp::get_logger(identifier_.c_str()), "[%s] %s",
                        type.c_str(), message.c_str());
    } else {
      RCLCPP_FATAL(rclcpp::get_logger(identifier_.c_str()), "[%s] %s",
                   type.c_str(), message.c_str());
    }
    break;
  }
}

void GenericMessage::add_member(const std::string member, Type type,
                                const std::string &type_identifier) {
  auto type_as_str = type_to_str(type);
  members_.push_back({member, type, type_as_str, type_identifier});
}

bool GenericMessage::compare(const std::string &str1,
                             const std::string &str2) const noexcept {
  if (strict_) {
    return strict_compare(str1, str2);
  } else {
    return lax_compare(str1, str2);
  }
}

std::string GenericMessage::type_to_str(Type type) {
  std::string type_as_str;

  auto isArray = type & XMSG::Type::ARRAY;
  auto subType = type & 0b11111110;

  switch (subType) {
  case XMSG::Type::CHAR:
    type_as_str = "char";
    break;
  case XMSG::Type::INT:
    type_as_str = "int";
    break;
  case XMSG::Type::UINT:
    type_as_str = "uint";
    break;
  case XMSG::Type::DOUBLE:
    type_as_str = "double";
    break;
  case XMSG::Type::STRING:
    type_as_str = "string";
    break;
  case XMSG::Type::BOOL:
    type_as_str = "bool";
    break;
  case XMSG::Type::GENERIC:
    type_as_str = "sub message";
    break;
  default:
    type_as_str = "Unknown";
    break;
  }

  if (isArray) {
    type_as_str += ":-array";
  }

  return type_as_str;
}

RawMemberWrapper
GenericMessage::build_wrapper(void *data, const Type &type) const noexcept {
  RawMemberWrapper ret;
  ret.data_ = data;
  ret.type_ = type;

  return ret;
}

BrokenMessage::~BrokenMessage() {}
