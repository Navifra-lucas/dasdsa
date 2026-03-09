#ifndef NAVIFRA_MESSAGE_HANDLER_H
#define NAVIFRA_MESSAGE_HANDLER_H

#include <Poco/JSON/Object.h>

namespace NaviFra {
class IMessageHandler {
public:
    virtual void handleMessage(Poco::JSON::Object::Ptr message) = 0;
    virtual ~IMessageHandler() = default;
};
}  // namespace NaviFra

#endif  // NAVIFRA_MESSAGE_HANDLER_H