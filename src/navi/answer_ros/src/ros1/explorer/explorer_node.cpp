#include "ros/ros.h"
#include "ros1/explorer/explorer_ros1.hpp"

#include <signal.h>

void SigHandler(sig_atomic_t sig)
{
    ros::requestShutdown();
}

int main(int argc, char **argv)
{
    signal(SIGINT, SigHandler);
    signal(SIGTERM, SigHandler);
    ros::init(argc, argv, "explorer", ros::init_options::NoSigintHandler);
    auto node = std::make_shared<NVFR::ExplorerRos1>("explorer");

    if (node->Initialized()) {
        ros::AsyncSpinner spinner(0);
        spinner.start();
        ros::waitForShutdown();
        spinner.stop();
    }
    else {
        std::cerr << "[ExplorerNode] Failed to initialize\n";
    }
    node->Terminator();
    ros::shutdown();
    std::cout << "[ExplorerNode] Normal Terminate\n";

    return 0;
}
