#include "nc_brain_agent/nc_brain_agent.h"
#include "nc_brain_agent/nc_robot_agent.h"
#include "nc_brain_agent/version.h"

#include <core_agent/core/navicore.h>

using namespace NaviFra;

void displayHelp()
{
    // # 일단 ROS 로거를 활용해 로그 남기기 파일로 남길려면 별도 설정 해줘야함
    LOG_INFO(" _   _                _   __ ");
    LOG_INFO("| \\ | |              (_) / _|");
    LOG_INFO("|  \\| |  __ _ __   __ _ | |_  _ __  __ _ ");
    LOG_INFO("| . ` | / _` |\\ \\ / /| ||  _|| '__|/ _` |");
    LOG_INFO("| |\\  || (_| | \\ V / | || |  | |  | (_| |");
    LOG_INFO("|_| \\_| \\__,_|  \\_/  |_||_|  |_|   \\__,_|");
    LOG_INFO("Navifra Brain Agent version %s.git.%s", PROJECT_VERSION, COMMIT_HASH);
    LOG_INFO("Copyright (c) 2023 by Navifra All rights reserved.");
}

void signalhandler(int signum)
{
    LOG_WARNING("Received SIGTERM signal. Shutting down ROS node gracefully...");
    NcRobotAgent::get().finalize();
    ros::shutdown();
}

int main(int argc, char** argv)
{
    NaviFra::Logger::get().SetSeverityMin(severity_level::info);
    displayHelp();
    signal(SIGTERM, signalhandler);
    signal(SIGINT, signalhandler);

    ros::init(argc, argv, "nc_brain_agent");

    initailizeCore();

    try {
        if (NcRobotAgent::get().initialize()) {}
        else {
            LOG_ERROR("agent initialize fail");
            ros::shutdown();
            // 초기화 못하면 종료 하는게 맞는거 같은데
            return 0;
        }

        ros::spin();

        NcRobotAgent::get().finalize();
    }
    catch (Poco::Exception& ex) {
        LOG_ERROR("%s", ex.displayText().c_str());
    }

    ros::shutdown();

    return 0;
}