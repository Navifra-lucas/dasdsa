
#include "common/exe_arguments.h"
#include "ros/ros.h"
#include "ros1/perception/perception_ros1.h"

#include <signal.h>
// std::unique_ptr<ANSWER::AnswerRos1> answer_ptr;
void SigHandler(sig_atomic_t sig)
{
    ros::requestShutdown();
}
int main(int argc, char **argv)
{
    signal(SIGINT, SigHandler);
    signal(SIGTERM, SigHandler);
    ANSWER::ExeArguments::GetInstance().Parse(argc, argv);

    ros::init(argc, argv, "perception", ros::init_options::NoSigintHandler);
    std::unique_ptr<ANSWER::PERCEPTION::PerceptionRos1> answer_ptr;
    answer_ptr =
        std::make_unique<ANSWER::PERCEPTION::PerceptionRos1>("perception");
    ros::AsyncSpinner spinner(0);
    spinner.start();
    ros::waitForShutdown();
    spinner.stop();
    answer_ptr->TerminateNode();
    std::cout << "ROS shutdown complete." << std::endl;
    // LOG_INFO("ROS shutdown complete.");

    return 0;
}