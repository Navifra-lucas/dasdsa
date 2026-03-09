#ifndef MOCK_REDIS_PUBLISHER_H
#define MOCK_REDIS_PUBLISHER_H

#include "core_agent/redis/redis_commnder.h"

#include <gmock/gmock.h>

class MockRedisPublisher : public NaviFra::RedisCommand {
public:
    MOCK_METHOD(void, hset, (const std::string& hash, const std::string& key, const std::string& value), (override));
    MOCK_METHOD(std::vector<std::string>, hgetall, (const std::string& hash), (override));
    MOCK_METHOD(std::string, hget, (const std::string& hash, const std::string& key), (override));
    MOCK_METHOD(std::vector<std::string>, getKeys, (const std::string& pattern), (override));
    MOCK_METHOD(void, hdel, (const std::string& hash, const std::string& key), (override));
};

#endif  // MOCK_REDIS_PUBLISHER_H
