#ifndef NAVIFRA_REDIS_COMMNDER_H
#define NAVIFRA_REDIS_COMMNDER_H

namespace NaviFra {
class RedisCommand {
public:
    RedisCommand() = default;
    virtual ~RedisCommand() = default;

    virtual void hset(const std::string& hash, const std::string& key, const std::string& value) = 0;
    virtual std::vector<std::string> hgetall(const std::string& hash) = 0;
    virtual std::string hget(const std::string& hash, const std::string& key) = 0;
    virtual void hdel(const std::string& hash, const std::string& key) = 0;
    virtual std::vector<std::string> getKeys(const std::string& pattern) = 0;
};
}  // namespace NaviFra

#endif  // NAVIFRA_REDIS_COMMNDER_H