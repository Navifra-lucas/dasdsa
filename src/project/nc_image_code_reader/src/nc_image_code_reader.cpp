#include "nc_image_code_reader.hpp"
#include <iostream>

using namespace NaviFra;

ImageCodeReader::ImageCodeReader(ros::NodeHandle& nh, ros::NodeHandle& nhp)
    : nh_(nh), nhp_(nhp)
{
    nhp_.param<std::string>("scanner_ip", host_, "192.168.0.10");
    nhp_.param<std::string>("scanner_port", port_, "2112");

    NLOG(info) << "[ImageCodeReader] Scanner IP: " << host_ << " / PORT: " << port_;

    read_cmd_sub_ = nh_.subscribe<std_msgs::String>("pallet_id_read_cmd", 10,
        boost::bind(&ImageCodeReader::readCmdCallback, this, _1));

    navi_cmd_sub_ = nh_.subscribe<std_msgs::String>("navifra/cmd", 10,
        boost::bind(&ImageCodeReader::naviCmdCallback, this, _1));
        
    pallet_id_pub_ = nh_.advertise<std_msgs::String>("pallet_id", 5);
    navi_error_pub_ = nh_.advertise<std_msgs::Int64>("navifra/error", 5);

    scanner_ = std::make_unique<TCPScannerPort>(io_, host_, port_);

    startScanner();
}

ImageCodeReader::~ImageCodeReader()
{
    // 계속 실행해야 하면 stop 제거
    NLOG(info) << "[ImageCodeReader] Destructor called";
}

void ImageCodeReader::readCmdCallback(const std_msgs::String::ConstPtr& msg)
{
    std::vector<uint8_t> packet;
    NLOG(info) << "[ImageCodeReader] Receive Read command : "<<msg->data;

    if (msg->data == "start") {
        packet = {0x02, 'r','e','a','d', 0x03};
        b_stop_cmd_received_ = false;
    } else if (msg->data == "stop") {
        packet = {0x02, 'e','n','d', 0x03};
        b_stop_cmd_received_ = true;
    }
    else if (msg->data == "protective_stop") {
        packet = {0x02, 'e','n','d', 0x03};
        b_stop_cmd_received_ = false;
    }

    scanner_->asyncWrite(packet, [](){
        NLOG(info) << "[ImageCodeReader] Read command sent";
    });
}

void ImageCodeReader::naviCmdCallback(const std_msgs::String::ConstPtr& msg)
{
    if(msg->data == "cancel") {
        std::vector<uint8_t> packet;
        packet = {0x02, 'e','n','d', 0x03};
        
        scanner_->asyncWrite(packet, [](){
            NLOG(info) << "[ImageCodeReader] Read command sent";
        });
    }
}

void ImageCodeReader::startScanner()
{
    try {
        scanner_->connect();
        NLOG(info) << "[ImageCodeReader] Scanner connected.";
    } catch(const std::exception& e) {
        NLOG(error) << "[ImageCodeReader] Scanner connection failed: " << e.what();
        return;
    }

    // async read 시작
    scanner_->asyncReadLine([this](const std::string code){
        if (b_terminate_)
            return;

        if (!code.empty()) {
            std_msgs::String msg;
            msg.data = code;
            NLOG(info)<<"[ImageCodeReader] Scanner Read Data : "<<code;
            pallet_id_pub_.publish(msg);
            if (code == "NoRead" && b_stop_cmd_received_) {
                std_msgs::Int64 error_msg;
                error_msg.data = core_msgs::NaviAlarm::ERROR_NO_BARCODE;
                navi_error_pub_.publish(error_msg);
                b_stop_cmd_received_ = false;
            }
        }
    });

    // io_context 별도 스레드
    io_thread_ = std::thread([this](){
        try {
            io_.run();
        } catch(const std::exception& e) {
            NLOG(error) << "[io_context] exception: " << e.what();
        }
    });
}

void ImageCodeReader::shutdownScanner()
{
    b_terminate_ = true;
    if (scanner_) scanner_->disconnect();

    io_.stop();
    if (io_thread_.joinable()) io_thread_.join();
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "nc_image_code_reader");
    ros::NodeHandle nh;
    ros::NodeHandle nhp("~");

    NaviFra::ImageCodeReader reader(nh, nhp);

    ros::spin();  // 계속 실행
    return 0;
}