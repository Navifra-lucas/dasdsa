#include "Poco/JSON/Object.h"
#include "ros/service_client.h"

#include <ros/ros.h>

#ifdef ANSWER_NAVICORE
    #include "answer/common_service.h"
using SRV = answer::common_service;
#else
    #include "answer_msgs/CommonService.h"
using SRV = answer_msgs::CommonService;
#endif

class sampling_match {
public:
    sampling_match()
    {
        answer_service_ = nh_.serviceClient<SRV>("answer/sampling_match");
        SRV srv;

        Poco::JSON::Object obj;
        obj.set("method", "random");
        obj.set("sample_num", 50);
        obj.set("sample_max_dist", 10.0);
        obj.set("sample_max_angle", 12);
        obj.set("sample_dist_resolution", 1.0);
        obj.set("sample_angle_resolution", 3.0);
        std::stringstream ss;
        obj.stringify(ss);
        std::string command = ss.str();
        srv.request.data = command;

        if (answer_service_.call(srv)) {
            ROS_INFO("SLAM started: %s", srv.response.reason.c_str());
            answer_service_.shutdown();
            ros::shutdown();
        }
    }
    ~sampling_match() {}
    ros::NodeHandle nh_;
    ros::ServiceClient answer_service_;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "sampling_match");
    sampling_match sampling_match;

    ros::spin();
    return 0;
}