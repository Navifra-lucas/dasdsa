#include "nc_brain_agent/initializer/alarm_manager_initializer.h"

#include "core_agent/manager/alarm_manager.h"
#include "core_agent/redis/redis_reader.h"
#include "core_agent/util/config.h"
#include "util/logger.hpp"

#include <core_agent/redis/redis_commnder.h>

namespace NaviFra {

void AlarmManagerInitializer::initialize()
{
    // ---- Redis 설정 읽기 ----
    std::string redis_host = Config::instance().getString("redis_host", "127.0.0.1");
    std::string redis_port = Config::instance().getString("redis_port", "5000");
    std::string redis_pass = Config::instance().getString("redis_passwd", "navifra1@3$");
    bool use_auth = Config::instance().getBool("use_auth", true);
    bool use_redis = Config::instance().getBool("use_redis", false);

    if (use_redis) {
        LOG_WARNING("Redis usage disabled. AlarmManager will operate without Redis.");
        try {
            AlarmManager::instance().loadState();
            LOG_INFO("AlarmManager state loaded (no Redis).");
        }
        catch (const std::exception& e) {
            LOG_ERROR("Failed to load AlarmManager state: %s", e.what());
        }
        return;
    }

    // ---- RedisReader 생성 및 연결 ----
    auto redisReader = std::make_shared<NaviFra::RedisReader>(use_auth);
    if (!redisReader->initialize(redis_host.c_str(), std::atoi(redis_port.c_str()), redis_pass.c_str())) {
        LOG_ERROR("Can't initialize RedisReader (host=%s, port=%s)", redis_host.c_str(), redis_port.c_str());
        return;
    }
    LOG_INFO("RedisReader connected");

    // ---- RedisCommand로 캐스팅 후 AlarmManager에 전달 ----
    auto redisCommander = std::dynamic_pointer_cast<NaviFra::RedisCommand>(redisReader);
    if (redisCommander) {
        AlarmManager::instance().setCommander(redisCommander);
        LOG_INFO("AlarmManager commander set to RedisCommand");
    }
    else {
        LOG_WARNING("Failed to cast RedisReader to RedisCommand. Commander not set.");
    }

    // ---- 상태 로드 ----
    try {
        AlarmManager::instance().loadState();
        LOG_INFO("AlarmManager state loaded successfully.");
    }
    catch (const std::exception& e) {
        LOG_ERROR("Failed to load AlarmManager state: %s", e.what());
    }
}

}  // namespace NaviFra
