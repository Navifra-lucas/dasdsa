#include "answer_core_bridge/answer_core_bridge.h"
#include "ros/ros.h"

#include <signal.h>
void SigHandler(sig_atomic_t sig)
{
    ros::requestShutdown();
}
int main(int argc, char **argv)
{
    signal(SIGINT, SigHandler);
    signal(SIGTERM, SigHandler);
    ros::init(
        argc, argv, "answer_core_bridge", ros::init_options::NoSigintHandler);

    AnswerCoreBridge answer_core_bridge;
    ros::AsyncSpinner spinner(0);
    spinner.start();
    ros::waitForShutdown();

    return 0;
}