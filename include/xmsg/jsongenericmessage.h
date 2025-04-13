#ifndef JSONGenericMessage_H
#define JSONGenericMessage_H

#include <xmsg/genericmessage.h>

#include <nlohmann/json.hpp>

class XMSG::JSONGenericMessage : public XMSG::GenericMessage {
public:
  JSONGenericMessage(const std::string &json_data);
  JSONGenericMessage();
  ~JSONGenericMessage() override;

  const XMSG::member get_member(const std::string &member_name,
                                const Type expected_type) override;
  void set_member(const std::string &member_name,
                  XMSG::member new_data) override;

  void copy_from(XMSG::GenericMessage *message) override;
  void parse();

  std::string get_current_path_to() { return path_to_; }

  std::string dump() { return json_.dump(); }
  std::string dump(int ident) { return json_.dump(ident); }

private: // members
  std::string path_to_;

private: // methods
  GenericMessage *New(const std::string &type) override {
    return NewGenericJson(type, true);
  }
  GenericMessage *NewFormless() override { return NewGenericJson(); }

  void enumerate_members();
  void set_current_path_to(std::string &path) { path_to_ = path; }

  void *extract(const nlohmann::json &member, const std::string &key);
  void *extract_array(const nlohmann::json &member, const std::string &key);

  XMSG::Type member_to_type(const nlohmann::json &member);
  XMSG::Type peak_at_type(const nlohmann::json &member);

  nlohmann::json find_member(const std::string &member);
  nlohmann::json member_to_json(const XMSG::member &member);

  std::string find_member_key(const std::string &member);

protected:
  nlohmann::json json_;

  virtual JSONGenericMessage *NewGenericJson(const std::string &json_data,
                                             int headerless) {
    (void)headerless;
    return new XMSG::JSONGenericMessage(json_data);
  }
  virtual JSONGenericMessage *NewGenericJson() {
    return new XMSG::JSONGenericMessage();
  }

  virtual void pre_parse() {
    log("Pre Parse not performed", XMSG::LogLevel::DEBUG);
  }
  virtual void post_parse() {
    log("Post Parse not performed", XMSG::LogLevel::DEBUG);
  }

  bool member_exists(const std::string &member_name);

  std::string get_parent_name();
};

#endif // JSONGenericMessage_H
