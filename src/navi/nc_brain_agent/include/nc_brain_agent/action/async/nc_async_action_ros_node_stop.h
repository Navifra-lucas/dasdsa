#ifndef NC_ASYNC_ACTION_ROS_NODE_STOP_H
#define NC_ASYNC_ACTION_ROS_NODE_STOP_H

#include <Poco/Runnable.h>

namespace NaviFra {
class NcAsyncActionROSNodeStop : public Poco::Runnable {
public:
    NcAsyncActionROSNodeStop(std::string uuid, std::string ros_node_id);
    ~NcAsyncActionROSNodeStop();

    using Ptr = std::shared_ptr<NcAsyncActionROSNodeStop>;

public:
    virtual void run() override;

private:
    std::string uuid_;
    std::string ros_node_id_;
};
}  // namespace NaviFra
#endif  // NC_ASYNC_ACTION_ROS_NODE_STOP_H