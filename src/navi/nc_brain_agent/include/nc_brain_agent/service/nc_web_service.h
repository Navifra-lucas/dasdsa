#ifndef NC_WEB_SERVICE_H
#define NC_WEB_SERVICE_H

#include <Poco/Net/HTTPServer.h>
#include <nc_brain_agent/web/common/webServerDispatcher.h>

namespace NaviFra {
class NcWEBService
    : public boost::noncopyable
    , public boost::serialization::singleton<NcWEBService> {
public:
    NcWEBService();
    ~NcWEBService();

    static NcWEBService& get() { return boost::serialization::singleton<NcWEBService>::get_mutable_instance(); }

    void initialize();

private:
    Poco::AutoPtr<Poco::Net::HTTPServerParams> _httpServerParams;
    Poco::AutoPtr<WebServerDispatcher> _webServerDispatcher;
    Poco::Net::HTTPServer* _httpServer;
};

}  // namespace NaviFra
#endif