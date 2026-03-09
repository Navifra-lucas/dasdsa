#ifndef NAVICPP_SERVER_REQUEST_HANDELER
#define NAVICPP_SERVER_REQUEST_HANDELER

#include "Poco/Net/HTTPRequestHandler.h"

namespace NaviFra {
class WebServerDispatcher;
class WebServerRequestHandler : public Poco::Net::HTTPRequestHandler {
public:
    WebServerRequestHandler(WebServerDispatcher& dispatcher, bool secure);

    ~WebServerRequestHandler();

    void handleRequest(Poco::Net::HTTPServerRequest& request, Poco::Net::HTTPServerResponse& response);

private:
    WebServerDispatcher& _dispatcher;
    bool _secure;
};

}  // namespace NaviFra

#endif  //  NAVICPP_SERVER_REQUEST_HANDELER
