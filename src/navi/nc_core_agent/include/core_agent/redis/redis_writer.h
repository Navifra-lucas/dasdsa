#ifndef NAVIFRA_REDIS_WRITER_H
#define NAVIFRA_REDIS_WRITER_H

#include <Poco/Logger.h>
#include <Poco/Redis/AsyncReader.h>
#include <Poco/Redis/Client.h>
#include <Poco/Redis/RedisEventArgs.h>

namespace NaviFra {
class RedisWriter {
public:
    RedisWriter(bool use_auth = true);
    ~RedisWriter();

public:
    bool initialize(const char* hostname, int port, const char* password);
    void disconnect();
    void reconnect();
    bool isConnected();

    void publish(std::string channel, std::string content);
    void set(const std::string& key, const std::string& value);
    std::string get(const std::string& key);
    std::vector<std::string> getKeys(const std::string& pattern);
    void del(const std::string& key);

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

#endif  // NC_REDIS_WRITER_H