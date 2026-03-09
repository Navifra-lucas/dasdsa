#include "ros/ros.h"
#include "ros1/graph_map/graph_map_ros1.hpp"

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

    ros::init(argc, argv, "graph_map_node", ros::init_options::NoSigintHandler);
    auto node = std::make_unique<NVFR::GraphMapRos1>("graph_map_node");

    if (node->Initialized()) {
        ros::AsyncSpinner spinner(0);
        spinner.start();
        ros::waitForShutdown();
        spinner.stop();
    }
    else {
        std::cerr << "[GraphMapNode] Failed to initialize\n";
    }
    node->Terminator();
    ros::shutdown();
    std::cout << "[GraphMapNode] Normal Terminate\n";

    return 0;
}
