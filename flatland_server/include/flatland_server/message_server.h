#ifndef FLATLAND_SERVER_MESSAGE_SERVER_H
#define FLATLAND_SERVER_MESSAGE_SERVER_H

#include <deque>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <ros/ros.h>

namespace flatland_server {

template <class T>
class Subscriber;
template <class T>
class Publisher;
class MessageServer;
class MessageTopicBase {};

template <class T>
class MessageTopic : public MessageTopicBase {
  std::unordered_set<Subscriber<T>*> subscribers_;

  friend class Subscriber<T>;
  friend class Publisher<T>;
  friend class MessageServer;
};

template <class T>
class Subscriber {
 public:
  Subscriber() = default;

  explicit Subscriber(MessageTopic<T>* topic,
                      const std::function<void(const T&)>& callback_function)
      : topic_(topic), callback_function_(callback_function) {}

 private:
  MessageTopic<T>* topic_;
  std::function<void(const T&)> callback_function_;

  friend class Publisher<T>;
};

template <class T>
class Publisher {
 public:
  Publisher() = default;
  explicit Publisher(MessageTopic<T>* topic) : topic_(topic) {}

  void publish(const T&);

 private:
  MessageTopic<T>* topic_;
};

class MessageServer {
 private:
  std::unordered_map<std::string, std::unique_ptr<MessageTopicBase>> topics_;

  template <class T>
  MessageTopic<T>* get_message_topic(const std::string& name);

 public:
  template <class T>
  Subscriber<T> subscribe(
      const std::string& name,
      const std::function<void(const T&)>& callback_function);

  template <class T>
  Publisher<T> advertise(const std::string& name);
};

///
/// Start of Implementation section
///

template <class T>
MessageTopic<T>* MessageServer::get_message_topic(const std::string& name) {
  auto msg_topic = topics_.find(name);
  if (msg_topic != topics_.end()) {
    return static_cast<MessageTopic<T>*>((msg_topic->second).get());
  }

  auto new_topic = topics_.emplace(
      name,
      std::unique_ptr<MessageTopic<T>>(new flatland_server::MessageTopic<T>()));
  return static_cast<MessageTopic<T>*>((new_topic.first->second).get());
}

template <class T>
Subscriber<T> MessageServer::subscribe(
    const std::string& name,
    const std::function<void(const T&)>& callback_function) {
  flatland_server::MessageTopic<T>* topic = get_message_topic<T>(name);

  Subscriber<T> subscriber(topic, callback_function);
  topic->subscribers_.insert(&subscriber);

  return std::move(subscriber);
}

template <class T>
Publisher<T> MessageServer::advertise(const std::string& name) {
  flatland_server::MessageTopic<T>* topic = get_message_topic<T>(name);
  return Publisher<T>(topic);
}

template <class T>
void Publisher<T>::publish(const T& t) {
  for (const Subscriber<T>* subsciber : topic_->subscribers_) {
    subsciber->callback_function_(t);
  }
}
}

#endif  // FLATLAND_SERVER_MESSAGE_H
