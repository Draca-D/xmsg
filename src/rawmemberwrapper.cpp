#include "xmsg/rawmemberwrapper.h"
#include "xmsg/genericmessage.h"

namespace XMSG {

template <typename BTYPE> static constexpr XMSG::Type type_to_xmsg_type() {
  if constexpr (std::is_same<BTYPE, int64_t>()) {
    return XMSG::INT;
  } else if constexpr (std::is_same<BTYPE, double>()) {
    return XMSG::DOUBLE;
  } else if constexpr (std::is_same<BTYPE, uint64_t>()) {
    return XMSG::UINT;
  } else if constexpr (std::is_same<BTYPE, std::string>()) {
    return XMSG::STRING;
  } else if constexpr (std::is_same<BTYPE, bool>()) {
    return XMSG::BOOL;
  } else if constexpr (std::is_same<BTYPE, char>()) {
    return XMSG::CHAR;
  } else if constexpr (std::is_same<BTYPE, std::vector<int64_t>>()) {
    return static_cast<XMSG::Type>(static_cast<int>(XMSG::INT) &
                                   static_cast<int>(XMSG::ARRAY));
  } else if constexpr (std::is_same<BTYPE, std::vector<double>>()) {
    return static_cast<XMSG::Type>(static_cast<int>(XMSG::DOUBLE) &
                                   static_cast<int>(XMSG::ARRAY));
  } else if constexpr (std::is_same<BTYPE, std::vector<uint64_t>>()) {
    return static_cast<XMSG::Type>(static_cast<int>(XMSG::UINT) &
                                   static_cast<int>(XMSG::ARRAY));
  } else if constexpr (std::is_same<BTYPE, std::vector<std::string>>()) {
    return static_cast<XMSG::Type>(static_cast<int>(XMSG::STRING) &
                                   static_cast<int>(XMSG::ARRAY));
  } else if constexpr (std::is_same<BTYPE, std::vector<bool>>()) {
    return static_cast<XMSG::Type>(static_cast<int>(XMSG::BOOL) &
                                   static_cast<int>(XMSG::ARRAY));
  } else if constexpr (std::is_same<BTYPE, std::vector<char>>()) {
    return static_cast<XMSG::Type>(static_cast<int>(XMSG::CHAR) &
                                   static_cast<int>(XMSG::ARRAY));
  }

  return XMSG::NOTSET;
}

RawMemberWrapper::RawMemberWrapper() {}

RawMemberWrapper::~RawMemberWrapper() { delete_member(); }

void RawMemberWrapper::delete_member() {
  if (data_ == nullptr) {
    return;
  }

  bool isArray = type_ & XMSG::ARRAY;
  auto cpp_type = static_cast<XMSG::Type>(type_ & 0b11111110);

  if (isArray) {
    delete_array(data_, cpp_type);
  } else {
    delete_member(data_, cpp_type);
  }

  data_ = nullptr;
}

void RawMemberWrapper::delete_array(void *data, Type type) const {
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

void RawMemberWrapper::delete_member(void *data, Type type) const {
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

RawMemberWrapper::RawMemberWrapper(const RawMemberWrapper &member) {
  copy(member);
}

RawMemberWrapper::RawMemberWrapper(RawMemberWrapper &&member) noexcept
    : data_(member.data_), type_(member.type_) {
  member.data_ = nullptr;
  member.type_ = Type::NONE;
}

RawMemberWrapper &RawMemberWrapper::operator=(const RawMemberWrapper &member) {
  copy(member);
  return *this;
}

RawMemberWrapper &
RawMemberWrapper::operator=(RawMemberWrapper &&member) noexcept {
  if (this == &member) {
    return *this;
  }

  delete_member();

  data_ = member.data_;
  type_ = member.type_;

  member.data_ = nullptr;
  member.type_ = Type::NONE;

  return *this;
}

template <typename RET_TYPE> RET_TYPE RawMemberWrapper::as() const {
  RET_TYPE ret;

  assert(data_ != nullptr);

  if (type_ == type_to_xmsg_type<RET_TYPE>()) {
    ret = *reinterpret_cast<RET_TYPE *>(data_);
  } else {
    RawMemberWrapper temp = *this;
    temp.modify_type(type_to_xmsg_type<RET_TYPE>());
    ret = temp.as<RET_TYPE>();
  }

  return ret;
}

void RawMemberWrapper::modify_type(Type new_type) {
  if (type_ & XMSG::Type::CHAR) {
    if (type_ & XMSG::Type::ARRAY) {
      auto x = extract_and_free_array<int64_t, int64_t>(data_);
      set_new_array<int64_t>(new_type, x);
    } else {
      auto x = extract_and_free<int64_t, int64_t>(data_);
      set_new_type<int64_t>(new_type, x);
    }
  } else if (type_ & XMSG::Type::INT) {
    if (type_ & XMSG::Type::ARRAY) {
      auto x = extract_and_free_array<int64_t, int64_t>(data_);
      set_new_array<int64_t>(new_type, x);
    } else {
      auto x = extract_and_free<int64_t, int64_t>(data_);
      set_new_type<int64_t>(new_type, x);
    }
  } else if (type_ & XMSG::Type::UINT) {
    if (type_ & XMSG::Type::ARRAY) {
      auto x = extract_and_free_array<uint64_t, uint64_t>(data_);
      set_new_array<uint64_t>(new_type, x);
    } else {
      auto x = extract_and_free<uint64_t, uint64_t>(data_);
      set_new_type<uint64_t>(new_type, x);
    }
  } else if (type_ & XMSG::Type::DOUBLE) {
    if (type_ & XMSG::Type::ARRAY) {
      auto x = extract_and_free_array<double, double>(data_);
      set_new_array<double>(new_type, x);
    } else {
      auto x = extract_and_free<double, double>(data_);
      set_new_type<double>(new_type, x);
    }
  } else if (type_ & XMSG::Type::BOOL) {
    if (type_ & XMSG::Type::ARRAY) {
      auto x = extract_and_free_array<bool, bool>(data_);
      set_new_array<bool>(new_type, x);
    } else {
      auto x = extract_and_free<bool, bool>(data_);
      set_new_type<bool>(new_type, x);
    }
  } else {
    throw std::invalid_argument("Not a modifyable type");
  }

  type_ = new_type;
}

void RawMemberWrapper::copy(const RawMemberWrapper &member) {
  type_ = member.type_;

  if (type_ & Type::BOOL) {
    using undertype = bool;
    if (type_ & Type::ARRAY) {
      assign_and_set_array<undertype, undertype>(
          data_, *static_cast<std::vector<undertype> *>(member.data_));
    } else {
      assign_and_set<undertype, undertype>(
          data_, *static_cast<undertype *>(member.data_));
    }
  } else if (type_ & Type::CHAR) {
    using undertype = char;
    if (type_ & Type::ARRAY) {
      assign_and_set_array<undertype, undertype>(
          data_, *static_cast<std::vector<undertype> *>(member.data_));
    } else {
      assign_and_set<undertype, undertype>(
          data_, *static_cast<undertype *>(member.data_));
    }
  } else if (type_ & Type::DOUBLE) {
    using undertype = double;
    if (type_ & Type::ARRAY) {
      assign_and_set_array<undertype, undertype>(
          data_, *static_cast<std::vector<undertype> *>(member.data_));
    } else {
      assign_and_set<undertype, undertype>(
          data_, *static_cast<undertype *>(member.data_));
    }
  } else if (type_ & Type::INT) {
    using undertype = int64_t;
    if (type_ & Type::ARRAY) {
      assign_and_set_array<undertype, undertype>(
          data_, *static_cast<std::vector<undertype> *>(member.data_));
    } else {
      assign_and_set<undertype, undertype>(
          data_, *static_cast<undertype *>(member.data_));
    }
  } else if (type_ & Type::STRING) {
    using undertype = std::string;
    if (type_ & Type::ARRAY) {
      assign_and_set_array<undertype, undertype>(
          data_, *static_cast<std::vector<undertype> *>(member.data_));
    } else {
      assign_and_set<undertype, undertype>(
          data_, *static_cast<undertype *>(member.data_));
    }
  } else if (type_ & Type::UINT) {
    using undertype = uint64_t;
    if (type_ & Type::ARRAY) {
      assign_and_set_array<undertype, undertype>(
          data_, *static_cast<std::vector<undertype> *>(member.data_));
    } else {
      assign_and_set<undertype, undertype>(
          data_, *static_cast<undertype *>(member.data_));
    }
  } else if (type_ & Type::GENERIC) {
    if (type_ & Type::ARRAY) {
      data_ = new std::vector<GenericMessage *>();

      for (auto msg :
           *static_cast<std::vector<GenericMessage *> *>(member.data_)) {

        GenericMessage *sub_message;

        if (msg->is_formless()) {
          sub_message = msg->NewFormless();
        } else {
          sub_message = msg->New(msg->get_type());
        }

        sub_message->copy_from(msg);

        static_cast<std::vector<GenericMessage *> *>(data_)->push_back(
            sub_message);
      }
    } else {
      if (static_cast<GenericMessage *>(member.data_)->is_formless()) {
        data_ = static_cast<GenericMessage *>(member.data_)->NewFormless();
      } else {
        data_ =
            static_cast<GenericMessage *>(member.data_)
                ->New(static_cast<GenericMessage *>(member.data_)->get_type());
      }

      static_cast<GenericMessage *>(data_)->copy_from(
          static_cast<GenericMessage *>(member.data_));
    }
  } else {
    throw std::runtime_error("Invalid Message given");
  }
}

bool is_integer_type(Type type) noexcept {
  return (type & XMSG::Type::INT) || (type & XMSG::Type::UINT);
}

bool is_numerical_type(Type type) noexcept {
  return is_integer_type(type) || (type & XMSG::Type::DOUBLE) ||
         (type & XMSG::Type::BOOL) || (type & XMSG::Type::CHAR);
}

template int64_t RawMemberWrapper::as<int64_t>() const;
template uint64_t RawMemberWrapper::as<uint64_t>() const;
template char RawMemberWrapper::as<char>() const;
template bool RawMemberWrapper::as<bool>() const;
template std::string RawMemberWrapper::as<std::string>() const;
template double RawMemberWrapper::as<double>() const;
template GenericMessage *RawMemberWrapper::as<GenericMessage *>() const;

template std::vector<int64_t>
RawMemberWrapper::as<std::vector<int64_t>>() const;
template std::vector<uint64_t>
RawMemberWrapper::as<std::vector<uint64_t>>() const;
template std::vector<char> RawMemberWrapper::as<std::vector<char>>() const;
template std::vector<bool> RawMemberWrapper::as<std::vector<bool>>() const;
template std::vector<std::string>
RawMemberWrapper::as<std::vector<std::string>>() const;
template std::vector<double> RawMemberWrapper::as<std::vector<double>>() const;
template std::vector<GenericMessage *>
RawMemberWrapper::as<std::vector<GenericMessage *>>() const;

} // namespace XMSG
