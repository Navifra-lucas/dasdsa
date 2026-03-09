#ifndef NAVIFRA_REDIS_READER_H
#define NAVIFRA_REDIS_READER_H

#include <Poco/Redis/AsyncReader.h>
#include <Poco/Redis/Client.h>
#include <Poco/Redis/RedisEventArgs.h>
#include <core_agent/redis/redis_commnder.h>

namespace NaviFra {
class RedisReader : public RedisCommand {
public:
    RedisReader(bool use_auth = true);
    ~RedisReader();

    using Ptr = std::shared_ptr<RedisReader>;

public:
    bool initialize(const char* hostname, int port, const char* password);
    void disconnect();
    void reconnect();
    bool isConnected();

    Poco::Redis::Array HMGET(const std::string key, std::vector<std::string> fileds);
    std::string GET(const std::string key);

public:
    virtual void hset(const std::string& hash, const std::string& key, const std::string& value) override;
    virtual std::vector<std::string> hgetall(const std::string& hash) override;
    virtual std::string hget(const std::string& hash, const std::string& key) override;
    virtual void hdel(const std::string& hash, const std::string& key) override;
    virtual std::vector<std::string> getKeys(const std::string& pattern) override;

private:
    bool connect();
    bool initialize();

private:
    std::string hostname_;
    std::string password_;
    int port_;
    bool use_auth_;

    Poco::Redis::Client client_;

    Poco::FastMutex _fastMutex;
};
}  // namespace NaviFra

#endif  // NAVIFRA_REDIS_READER_H