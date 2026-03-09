#include "core_astra/initializer/config_initializer.h"

#include "core_agent/util/config.h"
#include "util/logger.hpp"

#include <Poco/Dynamic/Var.h>
#include <Poco/Environment.h>
#include <Poco/FileStream.h>
#include <Poco/JSON/Object.h>
#include <Poco/JSON/Parser.h>
#include <Poco/Path.h>

using namespace NaviFra;

void ConfigInitializer::initialize()
{
    try {
        // 1. ROBOT_IP 환경 변수
        std::string robotIp = Poco::Environment::get("ROBOT_IP", "localhost");

        // 2. JSON 파일 경로
        std::string homeDir = Poco::Environment::get("HOME");
        Poco::Path configPath(homeDir);
        configPath.append("navifra_solution/ros/configs");
        configPath.append("nc_brain_agent_develop_" + robotIp + ".json");

        LOG_INFO("Loading config: %s", configPath.toString().c_str());

        // 3. JSON 파싱
        Poco::FileInputStream fis(configPath.toString());
        Poco::JSON::Parser parser;
        Poco::Dynamic::Var result = parser.parse(fis);
        Poco::JSON::Object::Ptr root = result.extract<Poco::JSON::Object::Ptr>();

        // 4. env.id 값 읽기
        if (root->has("env")) {
            Poco::JSON::Object::Ptr env = root->getObject("env");
            if (env->has("id")) {
                std::string id = env->getValue<std::string>("id");
                LOG_INFO("Loaded Robot ID: %s", id.c_str());

                // Config(MapConfiguration 상속 기반) 저장
                Config::instance().setString("robot_id", id);
            }
            else {
                LOG_WARNING("'env' object has no 'id' field.");
            }
        }
        else {
            LOG_WARNING("Config file has no 'env' object.");
        }
    }
    catch (Poco::Exception& ex) {
        LOG_ERROR("ConfigInitializer error: %s", ex.displayText().c_str());
    }

    LOG_INFO("ConfigInitializer completed initialization");
}
