#include "nc_wia_agent/version.h"
#include "util/logger.hpp"

#include <Poco/Thread.h>
#include <core_agent/core/navicore.h>
#include <core_agent/manager/action_manager.h>
#include <core_agent/manager/initializer_manager.h>
#include <ros/ros.h>

using namespace NaviFra;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "nc_wia_agent");

    LOG_INFO("=============================");
    LOG_INFO("▶ Starting nc_wia_agent v%s.%s", PROJECT_VERSION, COMMIT_HASH);
    LOG_INFO("=============================");

    ActionManager::instance().initialize();
    ActionManager::instance().printActions();

    NaviFra::InitializerManager::instance().run();
    initailizeCore();

    // Poco::ThreadPool::defaultPool().

    ros::spin();
    return 0;
}
