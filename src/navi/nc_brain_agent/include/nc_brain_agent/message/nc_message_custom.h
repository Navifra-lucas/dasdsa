#ifndef NC_MESSAGE_CUSTOM_H
#define NC_MESSAGE_CUSTOM_H

#include <core_agent/message/message_handler.h>
#include <core_agent/message/message_publisher.h>

namespace NaviFra {
class NcMessageCustom : public IMessageHandler {
public:
    NcMessageCustom();
    virtual void handleMessage(Poco::JSON::Object::Ptr message) override;
};

}  // namespace NaviFra

#endif  // NC_MESSAGE_CUSTOM_H