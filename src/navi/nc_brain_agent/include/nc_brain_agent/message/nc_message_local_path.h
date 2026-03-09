#ifndef NC_MESSAGE_LOCAL_PATH_H
#define NC_MESSAGE_LOCAL_PATH_H

#include <core_agent/message/message_handler.h>
#include <core_agent/message/message_publisher.h>

namespace NaviFra {
class NcMessageLocalPath : public IMessageHandler {
public:
    NcMessageLocalPath();
    virtual void handleMessage(Poco::JSON::Object::Ptr message) override;
};

}  // namespace NaviFra

#endif  // NC_MESSAGE_LOCAL_PATH_H