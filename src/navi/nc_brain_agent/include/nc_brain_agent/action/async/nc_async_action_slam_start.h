#ifndef NC_ASYNC_ACTION_SLAM_START_H
#define NC_ASYNC_ACTION_SLAM_START_H

#include <Poco/Runnable.h>

namespace NaviFra {
class NcAsyncActionSLAMStart : public Poco::Runnable {
public:
    NcAsyncActionSLAMStart(std::string uuid);
    ~NcAsyncActionSLAMStart();

    using Ptr = std::shared_ptr<NcAsyncActionSLAMStart>;

public:
    virtual void run() override;

private:
    std::string uuid_;
};
}  // namespace NaviFra
#endif  // NC_ASYNC_ACTION_SLAM_START_H