#include "nc_cheonil_agent/nc_cheonil_agent_pch.h"

#include <nc_cheonil_agent/manager/nc_cheonil_manager.h>
#include <nc_cheonil_agent/nc_cheonil_agent.h>
#include <nc_cheonil_agent/version.h>
#include <core_agent/core/navicore.h>

using namespace NaviFra;
void displayHelp()
{
    LOG_INFO(" _   _                _   __ ");
    LOG_INFO("| \\ | |              (_) / _|");
    LOG_INFO("|  \\| |  __ _ __   __ _ | |_  _ __  __ _ ");
    LOG_INFO("| . ` | / _` |\\ \\ / /| ||  _|| '__|/ _` |");
    LOG_INFO("| |\\  || (_| | \\ V / | || |  | |  | (_| |");
    LOG_INFO("|_| \\_| \\__,_|  \\_/  |_||_|  |_|   \\__,_|");
    LOG_INFO("Navifra Cheonil Agent version %s.git.%s", PROJECT_VERSION, COMMIT_HASH);
    LOG_INFO("Copyright (c) 2023 by Navifra All rights reserved.");
}

void signalhandler(int signum)
{
    LOG_WARNING("Received SIGTERM signal. Shutting down ROS node gracefully...");
    ros::shutdown();
}

int main(int argc, char** argv)
{
    displayHelp();
    ros::init(argc, argv, "nc_cheonil_agent");
    NaviFra::Logger::get().SetSeverityMin(severity_level::info);
    initailizeCore();

    try {
        NcCheonilManager agent;
        ros::NodeHandle nh;
        // ros::NodeHandle nh("~");
        if (agent.initialize(nh) != true) {
            LOG_ERROR("initialize failed");
            std::abort();
        }
        ros::spin();
    }
    catch (const std::exception& e) {
        NLOG(error) << e.what();
        std::abort();
    }

    ros::shutdown();

    return 0;
}
