#include "ros/ros.h"
#include "ros1/navi/navigator_ros1.hpp"

#include <iostream>
#include <signal.h>

#include "common/exe_arguments.h"

void SigHandler(sig_atomic_t sig)
{
    ros::requestShutdown();
}

int main(int argc, char **argv)
{
    signal(SIGINT, SigHandler);
    signal(SIGTERM, SigHandler);

    ANSWER::ExeArguments::GetInstance().Parse(argc, argv);

    ros::init(argc, argv, "navigator_node", ros::init_options::NoSigintHandler);
    auto node = std::make_unique<NVFR::NavigatorRos1>("navigator_node");

    if (node->Initialized()) {
        ros::AsyncSpinner spinner(0);
        spinner.start();
        ros::waitForShutdown();
        spinner.stop();
    }
    else {
        std::cerr << "[NavigatorNode] Failed to initialize\n";
    }
    node->Terminator();
    ros::shutdown();
    std::cout << "[NavigatorNode] Normal Terminate\n";

    return 0;
}
