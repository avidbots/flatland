#include <flatland_server/message_server.h>

namespace flatland_server {

void MessageServer::clean_old_topics() {
  for (auto& topic : messageTopics_) {
    topic.second->clean_old_messages();
  }
}
}
