#ifndef NAVIFRA_AUDIODRIVER_HPP_
#define NAVIFRA_AUDIODRIVER_HPP_

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "core/util/logger.hpp"
#include <boost/any.hpp>
#include <thread>
#include <gst/gst.h>

using namespace std;
namespace NaviFra {

class AudioDriver {
public:
    AudioDriver(ros::NodeHandle& nh, ros::NodeHandle& nhp);
    ~AudioDriver();

    void RegistListener();
    void SubscribeAudio(const std_msgs::String::ConstPtr& msg);

    void play(const std::string& filepath);
    void stop();
    
private:
    ros::NodeHandle nh_;
    ros::NodeHandle nhp_;

    ros::Subscriber sub_audio_;

    std::thread th_heartbeat_;
    bool b_terminate_ = false;

    GstElement *pipeline_;
    std::string current_file_;
};
};  // namespace NaviFra
#endif