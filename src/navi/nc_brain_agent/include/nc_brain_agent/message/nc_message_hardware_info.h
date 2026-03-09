#ifndef NC_MESSAGE_HARDWARE_INFO
#define NC_MESSAGE_HARDWARE_INFO

#include <core_agent/message/message_handler.h>
#include <core_agent/message/message_publisher.h>

namespace NaviFra {
class NcMessageHardwareInfo : public IMessageHandler {
public:
    NcMessageHardwareInfo();
    virtual void handleMessage(Poco::JSON::Object::Ptr message) override;
};

}  // namespace NaviFra

#endif  // NC_MESSAGE_SLAM_NODE_H