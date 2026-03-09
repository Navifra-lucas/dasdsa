#include "ros/service_server.h"

#include <ros/ros.h>

#include <chrono>
#include <thread>

#ifdef ANSWER_NAVICORE
    #include "answer/common_service.h"
using SRV = answer::common_service;
#else
    #include "answer_msgs/CommonService.h"
using SRV = answer_msgs::CommonService;
#endif

class update_param {
public:
    update_param()
    {
        answer_service_ = nh_.serviceClient<SRV>("answer/update_param");
        SRV srv;
        srv.request.data = "update";
        // srv.request.data = "{\"command\":\"start_slam\"}";
        std::chrono::system_clock::time_point start_time =
            std::chrono::system_clock::now();

        // Call the service
        while (!answer_service_.call(srv) &&
               std::chrono::duration_cast<std::chrono::seconds>(
                   std::chrono::system_clock::now() - start_time)
                       .count() < 2) {
            std::cout << "Retrying to call service..." << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }

        if (srv.response.success == false)
            std::cout << "Fail to update param" << std::endl;
        else
            std::cout << "Successfully updated param" << std::endl;
        answer_service_.shutdown();
        ros::shutdown();
    }
    ~update_param() {}
    ros::NodeHandle nh_;
    ros::ServiceClient answer_service_;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "update_param");

    if (argc != 1) {
        std::cout << "Usage: rosrun answer update_param" << std::endl;
        return 0;
    }
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "-h" || arg == "--help") {
            std::cout << "Usage: rosrun answer update_param" << std::endl;
            // std::cout << "* Change record data command *" << std::endl;
            // std::cout << "      COMMAND = start" << std::endl;
            // std::cout << "      COMMAND = stop" << std::endl;
            return 0;
        }
    }

    update_param update_param;

    ros::spin();
    return 0;
}