#include "nc_navican_remote/NaviCANRemote.h"
#include "ros/ros.h"
#include "util/logger.hpp"

#include <iostream>
#include <memory>

using namespace NaviFra::NaviCAN::Remote;

int main(int argc, char* argv[])
{
    static constexpr short ws_port = 8888;

    try {
        ros::init(argc, argv, "nc_navican_remote");

        ros::NodeHandle nh;
        ros::NodeHandle nhp("~");
        NaviCANRemote remote(ws_port);

        if (!remote.initialize()) {
            NLOG(severity_level::error) << "Failed to initialize NaviCANRemote";
            return 1;
        }

        remote.run();

        ros::spin();

        remote.finalize();
    }
    catch (const std::exception& e) {
        NLOG(severity_level::error) << "NaviCANRemote server error: " << e.what();
        return 1;
    }

    return 0;
}
