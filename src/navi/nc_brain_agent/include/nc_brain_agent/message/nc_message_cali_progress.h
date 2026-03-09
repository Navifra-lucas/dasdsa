#ifndef NC_MESSAGE_CALI_PROGRESS_H
#define NC_MESSAGE_CALI_PROGRESS_H

#include <core_agent/message/message_handler.h>
#include <core_agent/message/message_publisher.h>

namespace NaviFra {
class NcMessageCaliProgress : public IMessageHandler {
public:
    NcMessageCaliProgress();
    virtual void handleMessage(Poco::JSON::Object::Ptr message) override;
};

}  // namespace NaviFra

#endif  // NC_MESSAGE_CALI_PROGRESS_H