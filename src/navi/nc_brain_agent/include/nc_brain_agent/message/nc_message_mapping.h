#ifndef NC_MESSAGE_MAPPING_H
#define NC_MESSAGE_MAPPING_H

#include <core_agent/message/message_handler.h>
#include <core_agent/message/message_publisher.h>

namespace NaviFra {
class NcMessageMapping : public IMessageHandler {
public:
    NcMessageMapping();
    virtual void handleMessage(Poco::JSON::Object::Ptr message) override;
};

}  // namespace NaviFra

#endif  // NC_MESSAGE_MAPPING_H