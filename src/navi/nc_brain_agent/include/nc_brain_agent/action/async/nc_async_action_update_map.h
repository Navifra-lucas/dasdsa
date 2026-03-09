#ifndef NC_ASYNC_ACTION_MAP_UPDATE_H
#define NC_ASYNC_ACTION_MAP_UPDATE_H

#include <Poco/JSON/Object.h>
#include <Poco/Runnable.h>

namespace NaviFra {
class NcAsyncActionMapUpdate : public Poco::Runnable {
public:
    NcAsyncActionMapUpdate(std::string robotid, std::string uuid, std::string token, Poco::JSON::Object::Ptr obj);
    ~NcAsyncActionMapUpdate();
    using Ptr = std::shared_ptr<NcAsyncActionMapUpdate>;

    virtual void run() override;

private:
    std::string robotid_;
    std::string uuid_;
    std::string token_;
    Poco::JSON::Object::Ptr obj_;
};
}  // namespace NaviFra
#endif  // NC_ASYNC_ACTION_MAP_UPDATE_H