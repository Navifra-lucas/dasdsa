#ifndef NC_REPEAT_DRIVE_SERVICE_H
#define NC_REPEAT_DRIVE_SERVICE_H

#include <Poco/Runnable.h>
// #include <nav_msgs/Path.h>
#include <core_agent/data/robot_info.h>
// #include <sensor_msgs/PointCloud.h>
#include <core_msgs/RepeatTestMsg.h>
#include <std_msgs/String.h>

namespace NaviFra {
class NcRepeatDriveService
    : public Poco::Runnable
    , public boost::noncopyable
    , public boost::serialization::singleton<NcRepeatDriveService> {
public:
    NcRepeatDriveService();
    ~NcRepeatDriveService();

    static NcRepeatDriveService& get() { return boost::serialization::singleton<NcRepeatDriveService>::get_mutable_instance(); }

    void initialize(ros::NodeHandle& nh);

private:
    void onRepeatLog(const std_msgs::String msg);
    void onRepeatStatus(const std_msgs::String msg);

    virtual void run() override;

public:
    void start();
    void stop();
    bool isRunning();

private:
    bool isRunning_;
    std::string repeatlog_;
    std::string repeatstatus_;
    bool start_flag;

    std::vector<ros::Subscriber> rosSubscriber_;
    Poco::Thread worker_;
    Poco::FastMutex mutex_;
};

inline bool NcRepeatDriveService::isRunning()
{
    return isRunning_;
}

inline void NcRepeatDriveService::start()
{
    if (isRunning() != true) {
        isRunning_ = true;
        worker_.start(*this);
    }
}

inline void NcRepeatDriveService::stop()
{
    try {
        if (isRunning()) {
            isRunning_ = false;
            start_flag = false;
            worker_.tryJoin(1000);
        }
    }
    catch (Poco::TimeoutException& ex) {
        NLOG(error) << "Worker Stop timeout";
    }
}

}  // namespace NaviFra
#endif