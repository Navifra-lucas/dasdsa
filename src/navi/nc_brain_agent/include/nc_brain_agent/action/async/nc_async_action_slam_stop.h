#ifndef NC_ASYNC_ACTION_SLAM_STOP_H
#define NC_ASYNC_ACTION_SLAM_STOP_H

#include <Poco/Runnable.h>

namespace NaviFra {
class NcAsyncActionSLAMStop : public Poco::Runnable {
public:
    NcAsyncActionSLAMStop(std::string uuid);
    ~NcAsyncActionSLAMStop();

    using Ptr = std::shared_ptr<NcAsyncActionSLAMStop>;

public:
    virtual void run() override;

private:
    void sendResponse(std::string result = "success");

private:
    std::string uuid_;
};
}  // namespace NaviFra
#endif  // NC_ASYNC_ACTION_SLAM_STOP_H