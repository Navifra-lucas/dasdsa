#ifndef NC_REDIS_READER_H
#define NC_REDIS_READER_H

#include <Poco/Redis/AsyncReader.h>
#include <Poco/Redis/Client.h>
#include <Poco/Redis/RedisEventArgs.h>

#include <memory>

namespace NaviBrain {
class NcRedisReader {
public:
    NcRedisReader(bool use_auth = true);
    ~NcRedisReader();

    using Ptr = std::shared_ptr<NcRedisReader>;

private:
    std::string hostname_;
    std::string password_;
    int port_;
    bool use_auth_;

    Poco::Redis::Client client_;

    Poco::FastMutex _fastMutex;

private:
    bool connect();
    bool initialize();

public:
    bool initialize(const char* hostname, int port, const char* password);
    void disconnect();
    void reconnect();
    bool isConnected();

    Poco::Redis::Array HMGET(const std::string key, std::vector<std::string> fileds);
    std::string GET(const std::string key);
};
}  // namespace NaviBrain

#endif  // NC_REDIS_READER_H