#include "xmsg/genericmessage.h"

using namespace XMSG;

void GenericMessage::operator=(GenericMessage &message) { copy_from(&message); }

void GenericMessage::copy_from(GenericMessage *message) {
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
        new_member.type & XMSG::Type::GENERIC && transform.empty() &&
        auto_extract_child_) {
      GenericMessage *msg = nullptr;

      if (new_member.type & XMSG::Type::ARRAY) {
        auto vec =
            static_cast<std::vector<GenericMessage *> *>(new_member.data);
        if (!vec->empty()) {
          msg = vec->front(); // peek into vector to see what type of message it
                              // is // assumption is that all members of the
                              // vector will be of the same type
        }
      } else {
        msg = static_cast<GenericMessage *>(new_member.data);
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

    if (new_member.type & XMSG::NOTSET) {
      std::string str = "Message '" + get_type() + "' of generic type '" +
                        get_identifier() + "' is looking for '" + name +
                        "' which is not set in message '" +
                        message->get_type() + "' of generic type '" +
                        message->get_identifier() + "'";
      log(str, XMSG::LogLevel::WARN, XMSG::OnceType::UNIQUEONLY);
      continue;
    }

    try { // any of the processes below may throw, no point creating a new stack
          // for each possible option when theres only one thing we want to do
          // on a throw, see the catch
      if (member.type != new_member.type) {
        if (is_numerical_type(member.type) &&
            is_numerical_type(new_member.type)) {
          std::string mismatch =
              "Current member[" + name + "] " + "has type: '" +
              member.type_as_str +
              "'. Member from message we are converting from has type: '" +
              type_to_str(new_member.type) +
              "'. Will set members but casting errors may be present";
          log(mismatch, XMSG::LogLevel::WARN, XMSG::OnceType::UNIQUEONLY);
          modify_type(new_member, member.type);
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
                !(new_member.type & XMSG::Type::ARRAY)) {
              std::string mismatch =
                  "Current member[" + name + "] " +
                  " is an array, but the member we are converting from [" +
                  mapped_name +
                  "] is not an array. Unable to set requested child member, "
                  "ignoring";
              log(mismatch, XMSG::LogLevel::WARN, XMSG::OnceType::UNIQUEONLY);

              delete_member(new_member);

              continue;
            }

            if (!(member.type & XMSG::Type::ARRAY) &&
                (new_member.type & XMSG::Type::ARRAY)) {
              std::string mismatch =
                  "Current member[" + name + "] " +
                  " is not array, but the member we are converting from [" +
                  mapped_name +
                  "] is an array. Unable to set requested child member, "
                  "ignoring";
              log(mismatch, XMSG::LogLevel::WARN, XMSG::OnceType::UNIQUEONLY);

              delete_member(new_member);

              continue;
            }

            if (member.type & XMSG::Type::ARRAY) {
              auto vec = create_single_child_populated_msg_array(
                  type_id, name, child, new_member);

              XMSG::member memb = {vec, type};
              set_member(name, memb);
              delete_member(memb);

              continue;

            } else {
              auto new_msg = create_single_child_populated_msg(
                  type_id, name, child, new_member);
              if (new_msg == nullptr) {
                continue;
              }

              XMSG::member memb = {new_msg, type};
              set_member(name, memb);
              delete_member(memb);

              continue;
            }
          }

          std::string mismatch =
              "Current member[" + name + "] " + "has type: '" +
              member.type_as_str +
              "'. Member from message we are converting from has type: '" +
              type_to_str(new_member.type) + "'. Will not set member";
          log(mismatch, XMSG::LogLevel::ERROR, XMSG::OnceType::UNIQUEONLY);

          if (new_member.type == XMSG::Type::STRING) {
            auto str = static_cast<std::string *>(new_member.data);
            log("New member is a string with value: " + *str +
                    " and has type: " + message->type_,
                XMSG::LogLevel::ERROR, XMSG::OnceType::UNIQUEONLY);
          }
          delete_member(new_member);
          continue;
        }
      }

      if (new_member.type & XMSG::Type::GENERIC) {
        if (new_member.type & XMSG::Type::ARRAY) {
          auto nested_msg = static_cast<std::vector<XMSG::GenericMessage *> *>(
              new_member.data);

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

          XMSG::member memb = {vec, type};
          set_member(name, memb);
          delete_member(memb);
        } else {
          GenericMessage *new_msg;

          if (!formless_) {
            new_msg = this->New(type_id);
          } else {
            new_msg = this->NewFormless();
          }

          new_msg->set_helpful_name(name);
          auto nested_msg =
              static_cast<XMSG::GenericMessage *>(new_member.data);

          new_msg->copy_from(nested_msg);

          XMSG::member memb = {new_msg, type};
          set_member(name, memb);
          delete_member(memb);
        }
      } else {
        set_member(name, new_member);
      }
    } catch (...) { // catch everything, genericmessage is not aware enough to
                    // deal with the throw, its up to library user to handle the
                    // exception, all we do is catch, delete the heap'd memory
                    // that we own and then rethrow
      log("Cleaning up memory and rethrowing", XMSG::LogLevel::INFO);
      delete_member(new_member);
      throw;
    }

    delete_member(new_member);
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

member GenericMessage::transform_member(member &member,
                                        const std::string &transformation) {
  if (transformation.empty()) {
    return member;
  }

  if (transformation.compare("NaN") == 0) {
    delete_member(member);
    auto data = calloc(1, sizeof(double));
    static_cast<double *>(data)[0] = std::nan("");

    return {data, XMSG::Type::DOUBLE};
  }

  std::regex constant_check("^(\\s+)?(\\d+)(\\.\\d+)?(\\s+)?$");
  std::smatch value_match;

  if (std::regex_search(transformation, value_match,
                        constant_check)) { // checks if its just a constant
    auto int_value = value_match.str(2);
    auto dec_value = value_match.str(3);

    delete_member(member);

    if (dec_value.empty()) {
      auto data = calloc(1, sizeof(int64_t));
      static_cast<int64_t *>(data)[0] = std::stoi(int_value);

      return {data, XMSG::Type::INT};
    } else {
      auto data = calloc(1, sizeof(double));
      static_cast<double *>(data)[0] = std::stod(int_value + dec_value);

      return {data, XMSG::Type::DOUBLE};
    }
  }

  if (!is_transformable(member)) {
    log("Member has transformation listed in configuration but member is not "
        "transformable. Will "
        "ignore transformation, type is " +
            type_to_str(member.type),
        XMSG::LogLevel::WARN, XMSG::OnceType::UNIQUEONLY);
    return member;
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
    return member;
  }

  if (match.str(0).compare(transformation) !=
      0) { // part of the transformation was matched, but the whole line needs
           // to be matched, there should be a full line option for regex but
           // this is easier
    log("Partially Invalid process given '" + transformation +
            "'. Double check configuration - Will ignore transformation",
        XMSG::LogLevel::ERROR, XMSG::OnceType::UNIQUEONLY);
    return member;
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
      return member;
    }

    if (sub_member_name.empty()) {
      log("Requested access to sub member but no sub member name was provided, "
          "this shouldve "
          "been caught by the regex validator so check this",
          XMSG::LogLevel::ERROR, XMSG::OnceType::UNIQUEONLY);
      return member;
    }

    if (member.type & XMSG::Type::GENERIC) {
      auto sub_type = static_cast<GenericMessage *>(member.data);
      auto sub_member = sub_type->get_member(sub_member_name, XMSG::Type::NONE);

      if (sub_member.type & XMSG::Type::NOTSET) {
        log("Accessor requested member " + sub_member_name + " from " +
                sub_type->get_type() + " but " + sub_type->get_type() +
                " does not contain a member with this name...Ignoring "
                "transformation",
            XMSG::LogLevel::ERROR, XMSG::OnceType::UNIQUEONLY);
        return member;
      }

      delete_member(member);
      member = sub_member;

    } else {
      log("Invalid accessor request. "
          "Transformation requested access to sub member from complex type, "
          "but member is of type: " +
              type_to_str(member.type) + "... Ignoring transformation",
          XMSG::LogLevel::ERROR, XMSG::OnceType::UNIQUEONLY);
      return member;
    }
  }

  if (!math_operator.empty()) {
    auto type = member.type;

    if (type & XMSG::Type::CHAR) {
      using tp = int64_t;

      if (math_operator.compare("+") == 0) {
        transformation_handle_addition<tp>(member, math_term, term_decimal);
      } else if (math_operator.compare("-") == 0) {
        transformation_handle_subtraction<tp>(member, math_term, term_decimal);
      } else if (math_operator.compare("*") == 0) {
        transformation_handle_multiplication<tp>(member, math_term,
                                                 term_decimal);
      } else if (math_operator.compare("/") == 0) {
        transformation_handle_division<tp>(member, math_term, term_decimal);
      } else {
        log("Unrecognised operator '" + math_operator + "'",
            XMSG::LogLevel::ERROR, XMSG::OnceType::UNIQUEONLY);
      }
    } else if (type & XMSG::Type::INT) {
      using tp = int64_t;

      if (math_operator.compare("+") == 0) {
        transformation_handle_addition<tp>(member, math_term, term_decimal);
      } else if (math_operator.compare("-") == 0) {
        transformation_handle_subtraction<tp>(member, math_term, term_decimal);
      } else if (math_operator.compare("*") == 0) {
        transformation_handle_multiplication<tp>(member, math_term,
                                                 term_decimal);
      } else if (math_operator.compare("/") == 0) {
        transformation_handle_division<tp>(member, math_term, term_decimal);
      } else {
        log("Unrecognised operator '" + math_operator + "'",
            XMSG::LogLevel::ERROR, XMSG::OnceType::UNIQUEONLY);
      }
    } else if (type & XMSG::Type::UINT) {
      using tp = uint64_t;

      if (math_operator.compare("+") == 0) {
        transformation_handle_addition<tp>(member, math_term, term_decimal);
      } else if (math_operator.compare("-") == 0) {
        transformation_handle_subtraction<tp>(member, math_term, term_decimal);
      } else if (math_operator.compare("*") == 0) {
        transformation_handle_multiplication<tp>(member, math_term,
                                                 term_decimal);
      } else if (math_operator.compare("/") == 0) {
        transformation_handle_division<tp>(member, math_term, term_decimal);
      } else {
        log("Unrecognised operator '" + math_operator + "'",
            XMSG::LogLevel::ERROR, XMSG::OnceType::UNIQUEONLY);
      }
    } else if (type & XMSG::Type::DOUBLE) {
      using tp = double;

      if (math_operator.compare("+") == 0) {
        transformation_handle_addition<tp>(member, math_term, term_decimal);
      } else if (math_operator.compare("-") == 0) {
        transformation_handle_subtraction<tp>(member, math_term, term_decimal);
      } else if (math_operator.compare("*") == 0) {
        transformation_handle_multiplication<tp>(member, math_term,
                                                 term_decimal);
      } else if (math_operator.compare("/") == 0) {
        transformation_handle_division<tp>(member, math_term, term_decimal);
      } else {
        log("Unrecognised operator '" + math_operator + "'",
            XMSG::LogLevel::ERROR, XMSG::OnceType::UNIQUEONLY);
      }
    } else if (type & XMSG::Type::BOOL) {
      using tp = bool;

      if (math_operator.compare("+") == 0) {
        transformation_handle_addition<tp>(member, math_term, term_decimal);
      } else if (math_operator.compare("-") == 0) {
        transformation_handle_subtraction<tp>(member, math_term, term_decimal);
      } else if (math_operator.compare("*") == 0) {
        transformation_handle_multiplication<tp>(member, math_term,
                                                 term_decimal);
      } else if (math_operator.compare("/") == 0) {
        transformation_handle_division<tp>(member, math_term, term_decimal);
      } else {
        log("Unrecognised operator '" + math_operator + "'",
            XMSG::LogLevel::ERROR, XMSG::OnceType::UNIQUEONLY);
      }
    } else {
      throw std::invalid_argument("Not a transformable type");
    }
  }

  return member;
}

XMSG::GenericMessage *GenericMessage::create_single_child_populated_msg(
    const std::string &type_id, const std::string &name,
    const std::string &child, member &new_member) {
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

    delete_member(new_member);

    return nullptr;
  }

  auto child_type = new_msg->get_member_type(child);

  if (child_type != new_member.type) {
    if (is_numerical_type(child_type) && is_numerical_type(new_member.type)) {
      std::string mismatch =
          "Current member[" + name + "/" + child + "] " + "has type: '" +
          type_to_str(child_type) +
          "'. Member from message we are converting from has type: '" +
          type_to_str(new_member.type) +
          "'. Will set members but casting errors may be present";
      log(mismatch, XMSG::LogLevel::WARN, XMSG::OnceType::UNIQUEONLY);
      modify_type(new_member, child_type);

      new_msg->set_member(child, new_member);

      delete_member(new_member);

      return new_msg;

    } else {
      delete_member(new_member);
      return nullptr;
    }
  } else {
    new_msg->set_member(child, new_member);
    delete_member(new_member);

    return new_msg;
  }
}

std::vector<XMSG::GenericMessage *> *
GenericMessage::create_single_child_populated_msg_array(
    const std::string &type_id, const std::string &name,
    const std::string &child, member &new_member) {
  auto vec = new std::vector<GenericMessage *>;

  if (new_member.type & XMSG::Type::INT) {
    using tp = int64_t;
    auto nested_msg = static_cast<std::vector<tp> *>(new_member.data);
    for (auto &val : *nested_msg) {
      auto memb_val = calloc(1, sizeof(tp));
      static_cast<tp *>(memb_val)[0] = val;

      XMSG::member memb = {memb_val, new_member.type};

      auto new_msg =
          create_single_child_populated_msg(type_id, name, child, memb);

      if (new_msg == nullptr) {
        break;
      }

      vec->push_back(new_msg);
    }
  } else if (new_member.type & XMSG::Type::UINT) {
    using tp = uint64_t;
    auto nested_msg = static_cast<std::vector<tp> *>(new_member.data);
    for (auto &val : *nested_msg) {
      auto memb_val = calloc(1, sizeof(tp));
      static_cast<tp *>(memb_val)[0] = val;
      XMSG::member memb = {memb_val, new_member.type};

      auto new_msg =
          create_single_child_populated_msg(type_id, name, child, memb);

      if (new_msg == nullptr) {
        break;
      }

      vec->push_back(new_msg);
    }
  } else if (new_member.type & XMSG::Type::BOOL) {
    using tp = bool;
    auto nested_msg = static_cast<std::vector<tp> *>(new_member.data);
    for (size_t x = 0; x < nested_msg->size(); x++) {
      auto val = nested_msg->at(x);
      auto memb_val = calloc(1, sizeof(tp));
      static_cast<tp *>(memb_val)[0] = val;
      XMSG::member memb = {memb_val, new_member.type};

      auto new_msg =
          create_single_child_populated_msg(type_id, name, child, memb);

      if (new_msg == nullptr) {
        break;
      }

      vec->push_back(new_msg);
    }
  } else if (new_member.type & XMSG::Type::DOUBLE) {
    using tp = double;
    auto nested_msg = static_cast<std::vector<tp> *>(new_member.data);
    for (auto &val : *nested_msg) {
      auto memb_val = calloc(1, sizeof(tp));
      static_cast<tp *>(memb_val)[0] = val;
      XMSG::member memb = {memb_val, new_member.type};

      auto new_msg =
          create_single_child_populated_msg(type_id, name, child, memb);

      if (new_msg == nullptr) {
        break;
      }

      vec->push_back(new_msg);
    }
  } else if (new_member.type & XMSG::Type::STRING) {
    using tp = std::string;
    auto nested_msg = static_cast<std::vector<tp> *>(new_member.data);
    for (auto &val : *nested_msg) {
      auto memb_val = calloc(1, sizeof(tp));
      static_cast<tp *>(memb_val)[0] = val;
      XMSG::member memb = {memb_val, new_member.type};

      auto new_msg =
          create_single_child_populated_msg(type_id, name, child, memb);

      if (new_msg == nullptr) {
        break;
      }

      vec->push_back(new_msg);
    }
  } else if (new_member.type & XMSG::Type::CHAR) {
    using tp = int64_t;
    auto nested_msg = static_cast<std::vector<tp> *>(new_member.data);
    for (auto &val : *nested_msg) {
      auto memb_val = calloc(1, sizeof(tp));
      static_cast<tp *>(memb_val)[0] = val;
      XMSG::member memb = {memb_val, new_member.type};

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

bool GenericMessage::is_integer_type(Type type) const noexcept {
  return (type & XMSG::Type::INT) || (type & XMSG::Type::UINT);
}

bool GenericMessage::is_numerical_type(Type type) const noexcept {
  return is_integer_type(type) || (type & XMSG::Type::DOUBLE) ||
         (type & XMSG::Type::BOOL) || (type & XMSG::Type::CHAR);
}

bool GenericMessage::lax_compare(const std::string &str1,
                                 const std::string &str2) const noexcept {
  auto str1_ = to_lower(str1);
  auto str2_ = to_lower(str2);

  str1_ = remove_character(str1_, '_');
  str2_ = remove_character(str2_, '_');

  //  printf("%s:%s:%d\n",str1.c_str(), str2.c_str(), str1.compare(str2));

  return (str1_.compare(str2_) == 0);
}

bool GenericMessage::is_transformable(const member &member) const noexcept {
  if (member.type & (XMSG::Type::NOTSET | XMSG::Type::STRING |
                     XMSG::Type::NONE | XMSG::Type::ARRAY)) {
    return false;
    ;
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

void GenericMessage::delete_member(member &member) {
  if (member.data == nullptr) {
    return;
  }

  auto type = member.type;
  auto value = member.data;

  bool isArray = type & XMSG::ARRAY;
  auto cpp_type = static_cast<XMSG::Type>(type & 0b11111110);

  if (isArray) {
    delete_array(value, cpp_type);
    member.data = nullptr;
  } else {
    delete_member(value, cpp_type);
    member.data = nullptr;
  }
}

void GenericMessage::delete_array(void *data, Type type) {
  if (type == XMSG::Type::STRING) {
    delete static_cast<std::vector<std::string> *>(data);
  } else if (type == XMSG::Type::GENERIC) {
    auto vec = static_cast<std::vector<XMSG::GenericMessage *> *>(data);

    for (auto &msg : *vec) {
      delete msg;
    }

    delete vec;
  } else if (type & XMSG::Type::CHAR) {
    delete static_cast<std::vector<int64_t> *>(data);
  } else if (type & XMSG::Type::INT) {
    delete static_cast<std::vector<int64_t> *>(data);
  } else if (type & XMSG::Type::UINT) {
    delete static_cast<std::vector<uint64_t> *>(data);
  } else if (type & XMSG::Type::DOUBLE) {
    delete static_cast<std::vector<double> *>(data);
  } else if (type & XMSG::Type::BOOL) {
    delete static_cast<std::vector<bool> *>(data);
  }
}

void GenericMessage::delete_member(void *data, Type type) {
  if (type == XMSG::Type::STRING) {
    delete static_cast<std::string *>(data);
  } else if (type == XMSG::Type::GENERIC) {
    delete static_cast<GenericMessage *>(data);
  } else if (type & (XMSG::Type::NOTSET | XMSG::Type::NONE)) {
    return;
  } else {
    free(data);
  }
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

void GenericMessage::modify_type(member &member, Type new_type) {
  auto type = member.type;

  if (type & XMSG::Type::CHAR) {
    if (type & XMSG::Type::ARRAY) {
      auto x = extract_and_free_array<int64_t, int64_t>(member.data);
      set_new_array<int64_t>(member, new_type, x);
    } else {
      auto x = extract_and_free<int64_t, int64_t>(member.data);
      set_new_type<int64_t>(member, new_type, x);
    }
  } else if (type & XMSG::Type::INT) {
    if (type & XMSG::Type::ARRAY) {
      auto x = extract_and_free_array<int64_t, int64_t>(member.data);
      set_new_array<int64_t>(member, new_type, x);
    } else {
      auto x = extract_and_free<int64_t, int64_t>(member.data);
      set_new_type<int64_t>(member, new_type, x);
    }
  } else if (type & XMSG::Type::UINT) {
    if (type & XMSG::Type::ARRAY) {
      auto x = extract_and_free_array<uint64_t, uint64_t>(member.data);
      set_new_array<uint64_t>(member, new_type, x);
    } else {
      auto x = extract_and_free<uint64_t, uint64_t>(member.data);
      set_new_type<uint64_t>(member, new_type, x);
    }
  } else if (type & XMSG::Type::DOUBLE) {
    if (type & XMSG::Type::ARRAY) {
      auto x = extract_and_free_array<double, double>(member.data);
      set_new_array<double>(member, new_type, x);
    } else {
      auto x = extract_and_free<double, double>(member.data);
      set_new_type<double>(member, new_type, x);
    }
  } else if (type & XMSG::Type::BOOL) {
    if (type & XMSG::Type::ARRAY) {
      auto x = extract_and_free_array<bool, bool>(member.data);
      set_new_array<bool>(member, new_type, x);
    } else {
      auto x = extract_and_free<bool, bool>(member.data);
      set_new_type<bool>(member, new_type, x);
    }
  } else {
    throw std::invalid_argument("Not a modifyable type");
  }
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

BrokenMessage::~BrokenMessage() {}

template <typename RET_TYPE>
std::optional<RET_TYPE>
GenericMessage::get(const std::string &member_name) const {
  std::optional<RET_TYPE> ret;
}

template std::optional<int> GenericMessage::get<int>(const std::string &) const;
