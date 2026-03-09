#ifndef NC_MESSAGE_POLYGON_H
#define NC_MESSAGE_POLYGON_H

#include <core_agent/message/message_handler.h>
#include <core_agent/message/message_publisher.h>

namespace NaviFra {
class NcMessagePolygon : public IMessageHandler {
public:
    NcMessagePolygon();
    virtual void handleMessage(Poco::JSON::Object::Ptr message) override;
};

}  // namespace NaviFra

#endif  // NC_MESSAGE_POLYGON_H