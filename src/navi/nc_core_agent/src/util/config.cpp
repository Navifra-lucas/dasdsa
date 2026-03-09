#include "core_agent/core_agent.h"

#include <core_agent/util/config.h>

using namespace NaviFra;

Config::Config()
{
    const char* BACKEND_HOST = std::getenv("BACKEND_HOST");
    std::string backend_host = (BACKEND_HOST != NULL) ? BACKEND_HOST : "127.0.0.1";
    setString("backend_host", backend_host);

    const char* BACKEND_PORT = std::getenv("BACKEND_PORT");
    std::string backend_port = (BACKEND_PORT != NULL) ? BACKEND_PORT : "5000";
    setString("backend_port", backend_port);

    const char* REDIS_HOST = std::getenv("REDIS_HOST");
    std::string redis_host = (REDIS_HOST != NULL) ? REDIS_HOST : "127.0.0.1";
    setString("redis_host", redis_host);

    const char* REDIS_PORT = std::getenv("REDIS_PORT");
    std::string redis_port = (REDIS_PORT != NULL) ? REDIS_PORT : "6379";
    setString("redis_port", redis_port);

    const char* REDIS_PASS = std::getenv("REDIS_PASS");
    std::string redis_passwd = (REDIS_PASS != NULL) ? REDIS_PASS : "navifra1@3$";
    setString("redis_passwd", redis_passwd);

    const char* AGENT_TYPE = std::getenv("AGENT_TYPE");
    std::string agent_type = (AGENT_TYPE != NULL) ? AGENT_TYPE : "base";
    setString("agent_type", agent_type);

    const char* USE_NO_PASS_REDIS = std::getenv("USE_NO_PASS_REDIS");
    setBool("use_auth", ((USE_NO_PASS_REDIS != NULL && (std::string(USE_NO_PASS_REDIS).compare("true") == 0)) ? false : true));

    const char* MODEL_TYPE = std::getenv("MODEL_TYPE");
    std::string model_type = (MODEL_TYPE != NULL) ? MODEL_TYPE : "D10001";
    setString("model_type", model_type);

    const char* DRIVE_TYPE = std::getenv("DRIVE_TYPE");
    std::string drive_type = (DRIVE_TYPE != NULL) ? DRIVE_TYPE : "D30001";
    setString("drive_type", drive_type);

    const char* AGENT_HTTP_PORT = std::getenv("AGENT_HTTP_PORT");
    std::string agent_http_port = (AGENT_HTTP_PORT != NULL) ? AGENT_HTTP_PORT : "5555";
    setString("web.server.port", agent_http_port);

    const char* ROBOT_IP = std::getenv("ROBOT_IP");
    std::string robot_ip = (ROBOT_IP != NULL) ? ROBOT_IP : "localhost";
    setString("robot_ip", robot_ip);

    const char* MQTT_HOST = std::getenv("MQTT_HOST");
    std::string mqtt_host = (MQTT_HOST != NULL) ? MQTT_HOST : "localhost";
    setString("mqtt_host", mqtt_host);

    const char* MQTT_PORT = std::getenv("MQTT_PORT");
    std::string mqtt_port = (MQTT_PORT != NULL) ? MQTT_PORT : "1883";
    setString("mqtt_port", mqtt_port);

    setBool("use_redis", (std::getenv("USE_REDIS") ? std::string(std::getenv("USE_REDIS")) == "true" : true));

    const char* USE_MQTT = std::getenv("USE_MQTT");
    setBool("use_mqtt", ((USE_MQTT != NULL && (std::string(USE_MQTT).compare("true") == 0)) ? true : false));

    const char* USE_SSL = std::getenv("USE_SSL");
    setBool("use_ssl", ((USE_SSL != NULL && (std::string(USE_SSL).compare("true") == 0)) ? true : false));
}