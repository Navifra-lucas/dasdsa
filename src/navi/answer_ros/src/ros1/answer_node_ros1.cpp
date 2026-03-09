#include "common/answer_interface.h"
#include "common/exe_arguments.h"
#include "ros/ros.h"
// #include "ros1/answer_3d_ros1.h"
#include "ros1/answer_ros1.h"

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
    if (ANSWER::ExeArguments::GetInstance().Parse(argc, argv) == false) {
        return -1;
    }

    ros::init(argc, argv, "answer", ros::init_options::NoSigintHandler);
    std::string mode = ANSWER::ExeArguments::GetInstance().GetArgument(
        ANSWER::KEY::EXE_ARGUMENTS::DOF);
    // std::unique_ptr<ANSWER::AnswerRos1> answer_ptr;
    std::unique_ptr<ANSWER::AnswerInterface> answer_ptr;
    if (mode == ANSWER::KEY::EXE_ARGUMENTS::ANSWER_2D) {
        answer_ptr = std::make_unique<ANSWER::AnswerRos1>("answer");
    }
    else if (mode == ANSWER::KEY::EXE_ARGUMENTS::ANSWER_3D) {
        // answer_ptr = std::make_unique<ANSWER::Answer3DRos1>("answer3d");
    }

    ros::AsyncSpinner spinner(0);
    spinner.start();
    ros::waitForShutdown();
    spinner.stop();
    answer_ptr->TerminateNode();
    std::cout << "ROS shutdown complete." << std::endl;
    // LOG_INFO("ROS shutdown complete.");

    return 0;
}