#ifndef RAWMEMBERWRAPPER_H
#define RAWMEMBERWRAPPER_H

#include <cstdint>
#include <stdexcept>
#include <string>
#include <vector>

namespace XMSG {

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

bool is_integer_type(XMSG::Type type) noexcept;   // int uint
bool is_numerical_type(XMSG::Type type) noexcept; // int uint double bool char

class GenericMessage;

class RawMemberWrapper {
  friend class GenericMessage;

private:
  void *data_;
  Type type_;

private:
  RawMemberWrapper();

  void delete_member();
  void delete_array(void *data, Type type) const;
  void delete_member(void *data, Type type) const;

  void modify_type(Type new_type);
  void copy(const RawMemberWrapper &member);

public:
  ~RawMemberWrapper();
  RawMemberWrapper(const RawMemberWrapper &member);
  RawMemberWrapper &operator=(const RawMemberWrapper &member);
  RawMemberWrapper(RawMemberWrapper &&member) noexcept;
  RawMemberWrapper &operator=(RawMemberWrapper &&member) noexcept;

  Type get_type() const noexcept { return type_; }
  void *get_raw() const noexcept { return data_; }

  // Extract the raw data to an actual type
  // Note: This will attempt type coercion where it makes sense, i.e., if you
  // request a float, and the underlying type is an int, it will coerce the int
  // to a float. If it isnt possible, an empty optional will be returned.
  // Coercion will be done as a static cast, so check docs for any pitfalls
  template <typename RET_TYPE> RET_TYPE as() const;

private:
  template <typename undertype, typename basetype>
  std::vector<basetype> extract_and_free_array(void *data) {
    std::vector<basetype> x = static_cast<std::vector<basetype>>(
        *static_cast<std::vector<undertype> *>(data));
    delete static_cast<std::vector<undertype> *>(data);
    return x;
  }

  template <typename ty>
  void set_new_array(XMSG::Type new_type, std::vector<ty> x) {
    auto type = new_type;

    if (type & XMSG::Type::CHAR) {
      assign_and_set_array<int64_t, ty>(data_, x);
    } else if (type & XMSG::Type::INT) {
      assign_and_set_array<int64_t, ty>(data_, x);
    } else if (type & XMSG::Type::UINT) {
      assign_and_set_array<uint64_t, ty>(data_, x);
    } else if (type & XMSG::Type::DOUBLE) {
      assign_and_set_array<double, ty>(data_, x);
    } else if (type & XMSG::Type::BOOL) {
      assign_and_set_array<bool, ty>(data_, x);
    } else {
      throw std::invalid_argument("not a settable type");
    }
  }

  template <typename undertype, typename basetype>
  void assign_and_set_array(void *&data,
                            std::vector<basetype> &new_data) const {
    data = new std::vector<undertype>;

    for (basetype nd : new_data) {
      static_cast<std::vector<undertype> *>(data)->push_back(
          static_cast<undertype>(nd));
    }
  }

  template <typename undertype, typename basetype>
  basetype extract_and_free(void *data) const {
    basetype x = static_cast<basetype>(*static_cast<undertype *>(data));
    free(data);

    return x;
  }

  template <typename ty> void set_new_type(XMSG::Type new_type, ty x) {
    auto type = new_type;

    if (type & XMSG::Type::CHAR) {
      assign_and_set<int64_t, ty>(data_, x);
    } else if (type & XMSG::Type::INT) {
      assign_and_set<int64_t, ty>(data_, x);
    } else if (type & XMSG::Type::UINT) {
      assign_and_set<uint64_t, ty>(data_, x);
    } else if (type & XMSG::Type::DOUBLE) {
      assign_and_set<double, ty>(data_, x);
    } else if (type & XMSG::Type::BOOL) {
      assign_and_set<bool, ty>(data_, x);
    } else {
      throw std::invalid_argument("not a settable type");
    }
  }

  template <typename undertype, typename basetype>
  void assign_and_set(void *&data, basetype new_data) {
    data = calloc(1, sizeof(undertype));
    static_cast<undertype *>(data)[0] = static_cast<undertype>(new_data);
  }

  template <typename undertype>
  void transformation_handle_addition(const std::string &term,
                                      const std::string &decimal) {
    if (decimal.empty()) {
      *static_cast<undertype *>(data_) =
          *static_cast<undertype *>(data_) +
          static_cast<undertype>(std::stoi(term));
    } else {
      *static_cast<undertype *>(data_) =
          *static_cast<undertype *>(data_) +
          static_cast<undertype>(std::stod(term));
    }
  }

  template <typename undertype>
  void transformation_handle_subtraction(const std::string &term,
                                         const std::string &decimal) {
    if (decimal.empty()) {
      *static_cast<undertype *>(data_) =
          *static_cast<undertype *>(data_) -
          static_cast<undertype>(std::stoi(term));
    } else {
      *static_cast<undertype *>(data_) =
          *static_cast<undertype *>(data_) -
          static_cast<undertype>(std::stod(term));
    }
  }

  template <typename undertype>
  void transformation_handle_multiplication(const std::string &term,
                                            const std::string &decimal) {
    if (decimal.empty()) {
      *static_cast<undertype *>(data_) =
          *static_cast<undertype *>(data_) *
          static_cast<undertype>(std::stoi(term));
    } else {
      *static_cast<undertype *>(data_) =
          *static_cast<undertype *>(data_) *
          static_cast<undertype>(std::stod(term));
    }
  }

  template <typename undertype>
  void transformation_handle_division(const std::string &term,
                                      const std::string &decimal) {
    if (decimal.empty()) {
      *static_cast<undertype *>(data_) =
          *static_cast<undertype *>(data_) /
          static_cast<undertype>(std::stoi(term));
    } else {
      *static_cast<undertype *>(data_) =
          *static_cast<undertype *>(data_) /
          static_cast<undertype>(std::stod(term));
    }
  }
};
} // namespace XMSG

#endif // RAWMEMBERWRAPPER_H
