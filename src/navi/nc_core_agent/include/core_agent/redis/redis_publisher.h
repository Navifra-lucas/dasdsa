#ifndef NAVIFRA_REDIS_PUBLISHER_H
#define NAVIFRA_REDIS_PUBLISHER_H

#include <Poco/ObjectPool.h>
#include <Poco/Redis/AsyncReader.h>
#include <Poco/Redis/Client.h>
#include <Poco/Redis/RedisEventArgs.h>
#include <core_agent/message/message_publisher.h>
#include <core_agent/redis/redis_poolable_connection_factory.h>

namespace NaviFra {

// using RedisPoolableObjectFactory = Poco::PoolableObjectFactory<Poco::Redis::Client, Poco::Redis::Client::Ptr>;
using RedisPoolableObjectFactoryPtr = std::unique_ptr<Poco::RedisPoolableObjectFactory>;
using RedisConnectionPool = Poco::ObjectPool<Poco::Redis::Client, Poco::Redis::Client::Ptr, Poco::RedisPoolableObjectFactory>;
using RedisConnectionPoolPtr = std::unique_ptr<RedisConnectionPool>;

class RedisPublisher : public MessagePublisher {
public:
    RedisPublisher();
    ~RedisPublisher();

    using Ptr = std::shared_ptr<RedisPublisher>;

private:
    std::string hostname_;
    std::string password_;
    int port_;
    bool use_auth_;

    size_t poolCapacity_;
    size_t poolPeakCapacity_;

    std::unique_ptr<Poco::RedisPoolableObjectFactory> connectionFactory_;
    std::unique_ptr<RedisConnectionPool> connectionPool_;

private:
    bool connect();
    void disconnect();

public:
    virtual bool initialize() override;
    virtual bool isConnected() override;
    virtual void reconnect() override;

    virtual void publish(const std::string& channel, const std::string& message) override;
    virtual void hset(const std::string& hash, const std::string& key, const std::string& value);
    virtual std::vector<std::string> hgetall(const std::string& hash);
    virtual std::string hget(const std::string& hash, const std::string& key);
    virtual void hdel(const std::string& hash, const std::string& key);
    virtual std::vector<std::string> getKeys(const std::string& pattern);
};
}  // namespace NaviFra

#endif  // NC_REDIS_PUBLISHER_H