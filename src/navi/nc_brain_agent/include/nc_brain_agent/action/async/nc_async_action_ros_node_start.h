#ifndef NC_ASYNC_ACTION_ROS_NODE_START_H
#define NC_ASYNC_ACTION_ROS_NODE_START_H

#include <Poco/Runnable.h>

namespace NaviFra {
class NcAsyncActionROSNodeStart : public Poco::Runnable {
public:
    NcAsyncActionROSNodeStart(std::string uuid, std::string ros_node_id);
    ~NcAsyncActionROSNodeStart();

    using Ptr = std::shared_ptr<NcAsyncActionROSNodeStart>;

public:
    virtual void run() override;

private:
    std::string uuid_;
    std::string ros_node_id_;
};
}  // namespace NaviFra
#endif  // NC_ASYNC_ACTION_ROS_NODE_START_H