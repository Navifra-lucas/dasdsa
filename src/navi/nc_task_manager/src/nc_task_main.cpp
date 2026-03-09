#include "nc_task_manager/nc_task_manager_pch.h"

#include <nc_task_manager/nc_task_manager.h>

void displayHelp()
{
    // # 일단 ROS 로거를 활용해 로그 남기기 파일로 남길려면 별도 설정 해줘야함
    LOG_INFO("Copyright (c) 2023 by Navifra All rights reserved.");
    LOG_INFO("Started Task Manager")
}

void signalhandler(int signum)
{
    LOG_WARNING("Received SIGTERM signal. Shutting down ROS node gracefully...");
    ros::shutdown();
}

int main(int argc, char** argv)
{
    displayHelp();
    ros::init(argc, argv, "nc_task_manager");

    NaviFra::TaskManager task_manager;
    NaviFra::Logger::get().SetSeverityMin(severity_level::info);
    ros::AsyncSpinner spinner(2);
    spinner.start();
    ros::waitForShutdown();
    // ros::spin();

    return 0;
}
