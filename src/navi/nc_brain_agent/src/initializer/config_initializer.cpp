#include "nc_brain_agent/initializer/config_initializer.h"

#include <core_agent/data/memory_repository.h>
#include <core_agent/data/robot_status.h>
#include <core_agent/manager/initializer_manager.h>
#include <core_agent/util/config.h>
#include <ros/ros.h>

namespace NaviFra {

void ConfigInitializer::initialize()
{
    std::string backend_host = Config::instance().getString("backend_host", "127.0.0.1");
    std::string backend_port = Config::instance().getString("backend_port", "5000");
    std::string redis_host = Config::instance().getString("redis_host", "127.0.0.1");
    std::string redis_port = Config::instance().getString("redis_port", "5000");
    std::string redis_pass = Config::instance().getString("redis_passwd", "navifra1@3$");
    std::string agent_type = Config::instance().getString("agent_type", "base");

    std::string mqtt_host = Config::instance().getString("mqtt_host", "127.0.0.1");
    std::string mqtt_port = Config::instance().getString("mqtt_port", "1883");

    bool use_auth = Config::instance().getBool("use_auth", true);
    bool use_redis = Config::instance().getBool("use_redis", false);
    bool use_mqtt = Config::instance().getBool("use_mqtt", false);
    Config::instance().setBool("mqtt_transport", false);


    if (!use_auth) {
        NLOG(info) << "The brain agent is running in a mode without using Redis password";
    }

    LOG_INFO(
        "REDIS_HOST : %s, REDIS_PORT : %s, BACKEND_HOST : %s, "
        "BACKEND_PORT : %s, AGENT_TYPE : %s",
        redis_host.c_str(), redis_port.c_str(), backend_host.c_str(), backend_port.c_str(), agent_type.c_str());
}

}  // namespace NaviFra
