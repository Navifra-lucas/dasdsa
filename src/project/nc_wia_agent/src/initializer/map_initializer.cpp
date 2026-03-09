#include "nc_wia_agent/initializer/map_initializer.h"

#include "nc_wia_agent/data/robot_motor_feed_store.h"
#include "util/logger.hpp"

#include <Poco/Environment.h>
#include <Poco/File.h>
#include <std_msgs/String.h>

namespace NaviFra {

void MapInitializer::initialize()
{
    ros::NodeHandle nh;
    map_pub_ = nh.advertise<std_msgs::String>("/map_request", 10);

    ros::Duration(1.0).sleep();
    std::string s_map_info_folder_path = Poco::Environment::get("HOME") + "navifra_solution/navicore/configs/map/latest/map.png";
    NLOG(info) << "s_map_info_folder_path : " << s_map_info_folder_path;

    if (!Poco::File(s_map_info_folder_path).exists()) {
        NLOG(info) << "Detected outdated map version.";

        ros::Rate rate(10);
        while (map_pub_.getNumSubscribers() == 0) {
            if (!ros::ok())
                return;
            rate.sleep();
        }

        std_msgs::String msg;
        msg.data = "";
        map_pub_.publish(msg);
    }
    NLOG(info) << "Map update completed successfully";
}

}  // namespace NaviFra
