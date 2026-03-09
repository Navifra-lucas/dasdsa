#include "ros/service_server.h"

#include <ros/ros.h>

#ifdef ANSWER_NAVICORE
    #include "answer/common_service.h"
using SRV = answer::common_service;
#else
    #include "answer_msgs/CommonService.h"
using SRV = answer_msgs::CommonService;
#endif

class slam_start {
public:
    slam_start()
    {
        answer_service_ = nh_.serviceClient<SRV>("answer/control");
        SRV srv;
        srv.request.data = "{\"command\":\"start_slam\"}";
        if (answer_service_.call(srv)) {
            ROS_INFO("SLAM started: %s", srv.response.reason.c_str());
            answer_service_.shutdown();
            ros::shutdown();
        }
    }
    ~slam_start() {}
    ros::NodeHandle nh_;
    ros::ServiceClient answer_service_;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "slam_start");
    slam_start slam_start;

    ros::spin();
    return 0;
}