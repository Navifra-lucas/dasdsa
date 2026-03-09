#include "ros/service_server.h"

#include <ros/ros.h>

#ifdef ANSWER_NAVICORE
    #include "answer/common_service.h"
using SRV = answer::common_service;
#else
    #include "answer_msgs/CommonService.h"
using SRV = answer_msgs::CommonService;
#endif

class log_level {
public:
    log_level(int level)
    {
        answer_service_ = nh_.serviceClient<SRV>("answer/log_level");
        SRV srv;
        if (level == 0)
            srv.request.data = "trace";
        else if (level == 1)
            srv.request.data = "debug";
        else if (level == 2)
            srv.request.data = "info";
        else if (level == 3)
            srv.request.data = "warning";
        else if (level == 4)
            srv.request.data = "error";
        else if (level == 5)
            srv.request.data = "critical";
        else
            srv.request.data = "info";
        // srv.request.data = "{\"command\":\"start_slam\"}";
        if (answer_service_.call(srv)) {
            ROS_INFO("Log level changed: %s", srv.response.reason.c_str());
            answer_service_.shutdown();
            ros::shutdown();
        }
    }
    ~log_level() {}
    ros::NodeHandle nh_;
    ros::ServiceClient answer_service_;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "log_level");

    int level = 2;
    if (argc < 2) {
        std::cout << "Usage: rosrun answer log_level [LEVEL]" << std::endl;
        return 0;
    }
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "-h" || arg == "--help") {
            std::cout << "Usage: rosrun answer log_level [LEVEL]" << std::endl;
            std::cout
                << "* Change log level * Each level includes all upper levels."
                << std::endl;
            std::cout << "      LEVEL = 0 --> trace" << std::endl;
            std::cout << "      LEVEL = 1 --> debug" << std::endl;
            std::cout << "      LEVEL = 2 --> info" << std::endl;
            std::cout << "      LEVEL = 3 --> warning" << std::endl;
            std::cout << "      LEVEL = 4 --> error" << std::endl;
            std::cout << "      LEVEL = 5 --> critical" << std::endl;
            return 0;
        }
        if (arg == "0" || arg == "1" || arg == "2" || arg == "3" ||
            arg == "4" || arg == "5") {
            level = std::stoi(arg);
            std::cout << "Log level set to: " << level << std::endl;
        }
        else {
            std::cout << "Invalid log level: " << arg << std::endl;
            return -1;
        }
    }

    log_level log_level(level);

    ros::spin();
    return 0;
}