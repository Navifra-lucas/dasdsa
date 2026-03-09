#pragma once
#include <ros/ros.h>
#include <std_msgs/String.h>
#include "Tcp_scanner_port.hpp"
#include "core_msgs/NaviAlarm.h"
#include <std_msgs/Int64.h>
#include <memory>
#include <thread>

namespace NaviFra {

class ImageCodeReader
{
public:
    ImageCodeReader(ros::NodeHandle& nh, ros::NodeHandle& nhp);
    ~ImageCodeReader();

    void readCmdCallback(const std_msgs::String::ConstPtr& msg);
    void naviCmdCallback(const std_msgs::String::ConstPtr& msg);
    void startScanner();
    void shutdownScanner();

private:
    void stopScanner();

private:
    ros::NodeHandle nh_;
    ros::NodeHandle nhp_;
    ros::Subscriber read_cmd_sub_;
    ros::Subscriber navi_cmd_sub_;
    ros::Publisher pallet_id_pub_;
    ros::Publisher navi_error_pub_;

    std::unique_ptr<TCPScannerPort> scanner_;
    boost::asio::io_context io_;
    std::thread io_thread_;
    std::string host_;
    std::string port_;
    bool b_terminate_ = false;
    bool b_stop_cmd_received_ = false;
};

} // namespace NaviFra
