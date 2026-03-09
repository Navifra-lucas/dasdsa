#ifndef NAVIFRA_MESSAGE_SUBSCRIBER_H
#define NAVIFRA_MESSAGE_SUBSCRIBER_H

#if defined(_MSC_VER) && (_MSC_VER >= 1020)
    #pragma once
#endif

#include <Poco/BasicEvent.h>
#include <Poco/SingletonHolder.h>

#include <memory>
#include <string>
#include <vector>

namespace NaviFra {
class MessageSubscriberArgs {
public:
    MessageSubscriberArgs(std::string channel, std::string message, std::string source = "redis")
        : channel_(channel)
        , message_(message)
        , source_(source)
    {
    }

    const std::string& channel() { return channel_; }
    const std::string& message() { return message_; }
    const std::string& source() { return source_; }

private:
    std::string channel_;
    std::string message_;
    std::string source_;
};

class MessageSubscriber {
public:
    using Ptr = std::shared_ptr<MessageSubscriber>;

    virtual bool initialize() = 0;
    virtual bool isConnected() = 0;
    virtual void reconnect() = 0;
    virtual void subscribe(std::vector<std::string> channel) = 0;
    Poco::BasicEvent<MessageSubscriberArgs> notify_;

protected:
    virtual ~MessageSubscriber() = default;
};

class MessageSubscriberFactory {
public:
    template <typename C>
    MessageSubscriber::Ptr createMessageSubscriber()
    {
        return std::make_shared<C>();
    }
};

class DefaultSubscriber {
public:
    MessageSubscriber::Ptr subscriber;

    static DefaultSubscriber& instance()
    {
        static Poco::SingletonHolder<DefaultSubscriber> sh;
        return *sh.get();
    }

    ~DefaultSubscriber()
    {
        if (subscriber.get()) {
            subscriber.reset();
        }
    }
};

}  // namespace NaviFra

#endif  // !NC_MESSAGE_SUBSCRIBER_H
