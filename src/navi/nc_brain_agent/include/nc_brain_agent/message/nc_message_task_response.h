#ifndef NC_MESSAGE_TASK_RESPONSE_H
#define NC_MESSAGE_TASK_RESPONSE_H

#include <core_agent/message/message_handler.h>
#include <core_agent/message/message_publisher.h>

namespace NaviFra {
class NcMessageTaskResponse : public IMessageHandler {
public:
    NcMessageTaskResponse();
    virtual void handleMessage(Poco::JSON::Object::Ptr message) override;
};

}  // namespace NaviFra

#endif  // NC_MESSAGE_TASK_RESPONSE_H