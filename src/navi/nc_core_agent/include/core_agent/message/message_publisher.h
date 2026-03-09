#ifndef NAVIFRA_MESSAGE_PUBLISHER_H
#define NAVIFRA_MESSAGE_PUBLISHER_H

#if defined(_MSC_VER) && (_MSC_VER >= 1020)
    #pragma once
#endif

#include <Poco/SingletonHolder.h>

#include <memory>
#include <string>

namespace NaviFra {
class MessagePublisher {
public:
    using Ptr = std::shared_ptr<MessagePublisher>;

    virtual bool initialize() = 0;
    virtual bool isConnected() = 0;
    virtual void reconnect() = 0;
    virtual void publish(const std::string& channel, const std::string& message) = 0;

protected:
    virtual ~MessagePublisher() = default;
};

class MessagePublisherFactory {
public:
    template <typename C>
    MessagePublisher::Ptr createMessagePublisher()
    {
        return std::make_shared<C>();
    }
};

class DefaultPublisher {
public:
    MessagePublisher::Ptr publisher;

    static DefaultPublisher& instance()
    {
        static Poco::SingletonHolder<DefaultPublisher> sh;
        return *sh.get();
    }

    ~DefaultPublisher()
    {
        if (publisher.get()) {
            publisher.reset();
        }
    }
};

}  // namespace NaviFra

#endif  // !NC_MESSAGE_PUBLISHER
