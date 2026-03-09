#include "nc_wia_agent/initializer/config_initializer.h"

#include "nc_wia_agent/data/robot_basic_status.h"

#include <Poco/Environment.h>
#include <core_agent/data/memory_repository.h>
#include <core_agent/manager/initializer_manager.h>
#include <core_agent/util/config.h>
#include <ros/ros.h>

namespace NaviFra {

void ConfigInitializer::initialize()
{
    std::string mqtt_host, robot_id;
    int mqtt_port;

    mqtt_host = Poco::Environment::get("MQTT_WIA_HOST", "192.168.15.9");
    mqtt_port = std::atoi(Poco::Environment::get("MQTT_WIA_PORT", "1883").c_str());
    robot_id = Poco::Environment::get("MQTT_ROBOT_ID", "R_201");

    Config::instance().setString("mqtt_host", mqtt_host);
    Config::instance().setInt("mqtt_port", mqtt_port);

    Config::instance().setBool("mqtt_transport", true);

    // Repository에도 저장
    auto status = InMemoryRepository::instance().get<RobotBasicStatus>(RobotBasicStatus::KEY);
    if (status) {
        status->setRobotID(robot_id);
    }

    NLOG(info) << "mqtt_host=" << mqtt_host << ", mqtt_port=" << mqtt_port << ", robot_id=" << robot_id;
}

}  // namespace NaviFra
