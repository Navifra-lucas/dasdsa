#include "ros/service_server.h"

#include <ros/ros.h>

#ifdef ANSWER_NAVICORE
    #include "answer/common_service.h"
using SRV = answer::common_service;
#else
    #include "answer_msgs/CommonService.h"
using SRV = answer_msgs::CommonService;
#endif

class slam_terminate {
public:
    slam_terminate()
    {
        answer_service_ = nh_.serviceClient<SRV>("answer/control");
        SRV srv;
        srv.request.data = "{\"command\":\"terminate_slam\"}";
        if (answer_service_.call(srv)) {
            ROS_INFO("SLAM terminated: %s", srv.response.reason.c_str());
            answer_service_.shutdown();
            ros::shutdown();
        }
    }
    ~slam_terminate() {}
    ros::NodeHandle nh_;
    ros::ServiceClient answer_service_;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "slam_terminate");
    slam_terminate slam_terminate;

    ros::spin();
    return 0;
}