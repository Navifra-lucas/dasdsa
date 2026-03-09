#include "nc_io_manager/manager/nc_io_manager.h"
#include "util/logger.hpp"

#include <ros/ros.h>
#include <signal.h>

using namespace NaviFra;

// 전역 IOManager 포인터 (signal handler에서 사용)
void signalHandler(int signal)
{
    LOG_INFO("Received signal %d, shutting down gracefully...", signal);

    IOManager::instance().shutdown();

    ros::shutdown();
}

void _title()
{
    LOG_INFO("::::    :::  ::::::::     :::::::::::  ::::::::     ::::    ::::      :::     ::::    :::     :::      ::::::::  :::::::::: ::::::::: ");
    LOG_INFO(":+:+:   :+: :+:    :+:        :+:     :+:    :+:    +:+:+: :+:+:+   :+: :+:   :+:+:   :+:   :+: :+:   :+:    :+: :+:        :+:    :+:");
    LOG_INFO(":+:+:+  +:+ +:+               +:+     +:+    +:+    +:+ +:+:+ +:+  +:+   +:+  :+:+:+  +:+  +:+   +:+  +:+        +:+        +:+    +:+");
    LOG_INFO("+#+ +:+ +#+ +#+               +#+     +#+    +:+    +#+  +:+  +#+ +#++:++#++: +#+ +:+ +#+ +#++:++#++: :#:        +#++:++#   +#++:++#: ");
    LOG_INFO("+#+  +#+#+# +#+               +#+     +#+    +#+    +#+       +#+ +#+     +#+ +#+  +#+#+# +#+     +#+ +#+  +##+# +#+        +#+    +#+");
    LOG_INFO("#+#   #+#+# #+#    #+#        #+#     #+#    #+#    #+#       #+# #+#     #+# #+#   #+#+# #+#     #+# #+#    #+# #+#        #+#    #+#");
    LOG_INFO("###    ####  ########     ###########  ########     ###       ### ###     ### ###    #### ###     ###  ########  ########## ###    ###");
    LOG_INFO("                                          Copyright (c) 2023 by Navifra All rights reserved.                                          ");
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "nc_io_manager");


    // Signal handler 등록 (Ctrl+C 등)
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);
    
    _title();
    LOG_INFO("Starting NC IO Manager...");

    try {
        IOManager::instance().initialize();

        // ROS 메인 루프
        ros::spin();
    }
    catch (const std::exception& e) {
        LOG_ERROR("Exception occurred: %s", e.what());
        return -1;
    }
    catch (...) {
        LOG_ERROR("Unknown exception occurred");
        return -1;
    }

    LOG_INFO("NC IO Manager shutdown completed");
    return 0;
}