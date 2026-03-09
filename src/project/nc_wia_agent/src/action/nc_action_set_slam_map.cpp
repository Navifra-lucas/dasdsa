#include "nc_wia_agent/nc_wia_agent.h"

#include <Poco/Base64Decoder.h>  // base64 인코딩
#include <Poco/InflatingStream.h>
#include <core_agent/data/memory_repository.h>
#include <nc_wia_agent/action/nc_action_set_slam_map.h>
#include <ros/ros.h>
#include <std_msgs/String.h>

using namespace NaviFra;

NcActionSetSlamMap::NcActionSetSlamMap()
{
    map_pub_ = nh.advertise<std_msgs::String>("/map_request", 10);
}

NcActionSetSlamMap::~NcActionSetSlamMap()
{
}

std::string NcActionSetSlamMap::implName()
{
    return "NcActionSetSlamMap";
}

void NcActionSetSlamMap::implonAction(std::string source, Poco::JSON::Object::Ptr obj)
{
    try {
        std::string s_map_path = Poco::Environment::get("HOME") + "navifra_solution/navicore/configs/map/latest";

        Poco::JSON::Object::Ptr root = obj;

        std::string resolution = obj->getValue<std::string>("resolution");
        int width = obj->getValue<int>("width");
        int height = obj->getValue<int>("height");
        std::string content_type = obj->getValue<std::string>("content_type");
        Poco::JSON::Object::Ptr position = obj->getObject("Position");
        double pos_x = position->getValue<double>("x");
        double pos_y = position->getValue<double>("y");
        double pos_z = position->getValue<double>("z");

        ros::Rate rate(10);
        while (map_pub_.getNumSubscribers() == 0) {
            if (!ros::ok())
                return;
            rate.sleep();
        }
        std_msgs::String msg;
        msg.data = " ";
        map_pub_.publish(msg);

        // file로 보내기로 했음.. file인 경우랑

        NLOG(debug) << "position: x=" << pos_x << ", y=" << pos_y << ", z=" << pos_z;

        std::string map_version = obj->getValue<std::string>("map_version");

        int msg_id = obj->getValue<int>("msg_id");

        std::string rid = obj->getValue<std::string>("RID");

        // response

        Poco::JSON::Object::Ptr response = new Poco::JSON::Object;
        response->set("Cmd", obj->getValue<std::string>("Cmd"));
        response->set("msg_time", Poco::DateTimeFormatter::format(Poco::LocalDateTime(), "%Y-%m-%d %H:%M:%S.%i"));
        response->set("msg_id", msg_id);
        response->set("RID", rid);
        response->set("Result", "S");
        response->set("AmrId", rid);

        std::ostringstream oss;
        response->stringify(oss);

        NLOG(debug) << "Sending map message: " << oss.str();

        MessageBroker::instance().publish(source, rid + ".ACS", oss.str());
    }
    catch (const Poco::Exception& ex) {
        NLOG(error) << "Exception: " << ex.displayText();
    }
}