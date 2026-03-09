#ifndef NAVIFRA_REDIS_SUBSCRIBER_H
#define NAVIFRA_REDIS_SUBSCRIBER_H

#include <Poco/Activity.h>
#include <Poco/Redis/AsyncReader.h>
#include <Poco/Redis/Client.h>
#include <Poco/Redis/RedisEventArgs.h>
#include <core_agent/message/message_subscriber.h>

using Poco::Redis::RedisEventArgs;

namespace NaviFra {
class RedisSubscriber : public MessageSubscriber {
public:
    RedisSubscriber();
    ~RedisSubscriber();

    using Ptr = std::shared_ptr<RedisSubscriber>;

public:
    virtual void subscribe(std::vector<std::string> channel) override;
    virtual bool initialize() override;
    virtual bool isConnected() override;
    virtual void reconnect() override;

public:
    bool initialize(const char* hostname, int port, const char* password);
    void disconnect();
    void pSubscribe(std::vector<std::string> channels);

    void onRedisReaderEvent(const void*, Poco::Redis::RedisEventArgs&);
    void onRedisReaderErrorEvent(const void*, Poco::Redis::RedisEventArgs&);

    void start();
    void stop();
    bool isStopped();

protected:
    bool connect();
    void runActivity();
    void updateState();

private:
    std::string hostname_;
    std::string password_;
    int port_;
    bool use_auth_;

    Poco::Redis::Client client_;
    Poco::BasicEvent<RedisEventArgs> redisResponse_;
    /// Event that is fired when a message is received.

    Poco::BasicEvent<RedisEventArgs> redisException_;
    /// Event that is fired when an error occurred.

    std::vector<std::string> channels_;
    std::vector<std::string> patternChannels_;

    Poco::Activity<RedisSubscriber> activity_;
    Poco::Event waitpong_;

    Poco::FastMutex _fastMutex;

    bool preRedisState;
    bool curRedisState;
};

inline bool RedisSubscriber::isStopped()
{
    return activity_.isStopped();
}

inline void RedisSubscriber::start()
{
    activity_.start();
}

inline void RedisSubscriber::stop()
{
    activity_.stop();
    activity_.wait(2000);
}

}  // namespace NaviFra

#endif  // NAVIFRA_REDIS_SUBSCRIBER_H