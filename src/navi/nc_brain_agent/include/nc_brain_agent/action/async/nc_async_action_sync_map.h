#ifndef NC_ASYNC_ACTION_SYNC_MAP_H
#define NC_ASYNC_ACTION_SYNC_MAP_H

#include <Poco/JSON/Object.h>
#include <Poco/Runnable.h>

namespace NaviFra {
class NcAsyncActionSyncMap : public Poco::Runnable {
public:
    NcAsyncActionSyncMap(std::string robotid, std::string uuid, std::string token, Poco::JSON::Object::Ptr obj);
    ~NcAsyncActionSyncMap();
    using Ptr = std::shared_ptr<NcAsyncActionSyncMap>;

    virtual void run() override;

private:
    std::string robotid_;
    std::string uuid_;
    std::string token_;
    Poco::JSON::Object::Ptr obj_;
};
}  // namespace NaviFra
#endif  // NC_ASYNC_ACTION_MAP_UPDATE_H