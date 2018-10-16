#ifndef FLATLAND_SERVER_MESSAGE_SERVER_H
#define FLATLAND_SERVER_MESSAGE_SERVER_H

#include <deque>
#include <unordered_map>
#include <vector>

#include <ros/ros.h>

namespace flatland_server {

class MessageServer;
class MessageTopicBase {
 public:
  virtual void clean_old_messages() = 0;
};
class PublisherBase {};
class SubscriberBase {};

template <class T>
class MessageView {
  using const_iterator =
      typename std::deque<std::pair<ros::Time, T>>::const_iterator;

  const_iterator begin_;
  const_iterator end_;

  void filterTimeRange(const ros::Time begin, const ros::Time end);

 public:
  const_iterator begin() const { return begin_; }
  const_iterator end() const { return end_; }

  bool empty() const { return begin_ == end_; }
  unsigned size() const;

  MessageView() {}
  MessageView(const std::deque<std::pair<ros::Time, T>>& t)
      : begin_(t.begin()), end_(t.end()) {}
  MessageView(const std::deque<std::pair<ros::Time, T>>& t,
              const ros::Time& start, const ros::Time& end)
      : MessageView(t) {
    filterTimeRange(start, end);
  }
};

template <class T>
class MessageTopic : public MessageTopicBase {
  ros::Duration message_life_;
  std::deque<std::pair<ros::Time, T>> messages_;

 public:
  MessageTopic(ros::Duration message_life) : message_life_(message_life) {}

  const std::deque<std::pair<ros::Time, T>>& get_messages() const {
    return messages_;
  }
  const MessageView<T> get_message_view() { return MessageView<T>(messages_); }
  const MessageView<T> get_message_view(const ros::Time& start,
                                        const ros::Time& end) {
    return MessageView<T>(messages_, start, end);
  }

  void add_new_message(const T& msg);
  void clean_old_messages() override;
};

template <class T>
class Subscriber : public SubscriberBase {
  MessageTopic<T>* topic_;
  ros::Time last_subscription_time;

  Subscriber(MessageTopic<T>* topic)
      : topic_(topic), last_subscription_time(ros::Time::now()) {}

 public:
  Subscriber() {}
  ~Subscriber() {}
  MessageView<T> receive();

  friend class MessageServer;
};

template <class T>
class Publisher : public PublisherBase {
  MessageTopic<T>* topic_;

  Publisher(MessageTopic<T>* topic) : topic_(topic) {}

 public:
  Publisher() {}
  ~Publisher() {}
  void publish(const T&);

  friend class MessageServer;
};

class MessageServer {
 public:
  std::unordered_map<std::string, std::unique_ptr<MessageTopicBase>>
      messageTopics_;

  template <class T>
  MessageTopic<T>* create_topic(
      const std::string& name,
      const ros::Duration& message_life = ros::Duration(1));

  template <class T>
  Subscriber<T> subscribe(
      const std::string& name,
      const ros::Duration& message_lifetime = ros::Duration(1));

  template <class T>
  Publisher<T> advertise(
      const std::string& name,
      const ros::Duration& message_lifetime = ros::Duration(1));

  void clean_old_topics();
};

///
/// Start of Implementation section
///

template <class T>
MessageTopic<T>* MessageServer::create_topic(
    const std::string& name, const ros::Duration& message_life) {
  flatland_server::MessageTopic<T>* topic;
  auto msgTopic = messageTopics_.find(name);
  if (msgTopic == messageTopics_.end()) {
    topic = new flatland_server::MessageTopic<T>(message_life);
    messageTopics_.emplace(name, std::unique_ptr<MessageTopic<T>>(topic));
    ROS_INFO_STREAM("Flatland Server topic created: " << name);
  } else {
    topic = static_cast<MessageTopic<T>*>((msgTopic->second).get());
  }
  return topic;
}

template <class T>
Subscriber<T> MessageServer::subscribe(const std::string& name,
                                       const ros::Duration& message_lifetime) {
  flatland_server::MessageTopic<T>* topic =
      create_topic<T>(name, message_lifetime);
  return Subscriber<T>(topic);
}

template <class T>
Publisher<T> MessageServer::advertise(const std::string& name,
                                      const ros::Duration& message_lifetime) {
  flatland_server::MessageTopic<T>* topic =
      create_topic<T>(name, message_lifetime);
  return Publisher<T>(topic);
}

template <class T>
void Publisher<T>::publish(const T& t) {
  topic_->add_new_message(t);
}

template <class T>
unsigned MessageView<T>::size() const {
  unsigned c = 0;
  for (auto it = this->begin_; it != this->end_; ++it) c++;
  return c;
}

template <class T>
void MessageView<T>::filterTimeRange(const ros::Time timeStart,
                                     const ros::Time timeEnd) {
  // Iterate and remove messages before time start
  while (begin_ != end_) {
    if (begin_->first < timeStart) {
      ++begin_;
    } else {
      break;
    }
  }
  // Iterate and remove messages after time End
  while (begin_ != end_) {
    if ((end_ - 1)->first >= timeEnd) {
      --end_;
    } else {
      break;
    }
  }
}

template <class T>
MessageView<T> Subscriber<T>::receive() {
  ros::Time now = ros::Time::now();
  MessageView<T> view = topic_->get_message_view(last_subscription_time, now);
  last_subscription_time = now;
  return view;
}

template <class T>
void MessageTopic<T>::clean_old_messages() {
  ros::Time now = ros::Time::now();
  // Delete old messages
  while (!messages_.empty()) {
    const std::pair<ros::Time, T>& msg = messages_.front();

    if (now - msg.first > message_life_) {
      messages_.pop_front();
      continue;
    }
    break;
  }
}

template <class T>
void MessageTopic<T>::add_new_message(const T& msg) {
  messages_.emplace_back(ros::Time::now(), msg);
}
}

#endif  // FLATLAND_SERVER_MESSAGE_H
