#ifndef NC_MESSAGE_ALARM_H
#define NC_MESSAGE_ALARM_H

#include <core_agent/message/message_handler.h>
#include <core_agent/message/message_publisher.h>

namespace NaviFra {
class NcMessageAlarm : public IMessageHandler {
public:
    NcMessageAlarm();
    virtual void handleMessage(Poco::JSON::Object::Ptr message) override;
};

}  // namespace NaviFra

#endif  // NC_MESSAGE_CALI_PROGRESS_H