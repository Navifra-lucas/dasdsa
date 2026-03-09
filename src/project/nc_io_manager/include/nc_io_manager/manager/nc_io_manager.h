#ifndef NC_IO_MANAGER_H
#define NC_IO_MANAGER_H

#include "nc_io_manager/controller/nc_io_controller.h"

#include <Poco/SingletonHolder.h>
#include <ros/ros.h>
#include <std_msgs/String.h>

#include <memory>
#include <string>

namespace NaviFra {
class IOManager {
public:
    IOManager();
    virtual ~IOManager();

    static IOManager& instance()
    {
        static Poco::SingletonHolder<IOManager> sh;
        return *sh.get();
    }

private:
    std::shared_ptr<IOController> controller_;

    // 시스템 상태 관리
    bool initialized_;
    std::string system_state_;
    ros::Time start_time_;

    // ROS 인터페이스
    ros::Timer status_timer_;

public:
    void initialize();
    void shutdown();

private:
    // 초기화 관련
    void setupROSInterface();
};
}  // namespace NaviFra

#endif  // NC_IO_MANAGER_H