#ifndef NC_MESSAGE_REFLECTORS_H
#define NC_MESSAGE_REFLECTORS_H

#include <core_agent/message/message_handler.h>
#include <core_agent/message/message_publisher.h>

namespace NaviFra {
class NcMessageReflectors : public IMessageHandler {
public:
    NcMessageReflectors();
    virtual void handleMessage(Poco::JSON::Object::Ptr message) override;
};

}  // namespace NaviFra

#endif  // NC_MESSAGE_SLAM_NODE_H