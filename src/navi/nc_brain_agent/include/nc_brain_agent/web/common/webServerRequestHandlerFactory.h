
#ifndef NAVICPP_WEBSERVER_REQUESTHANDLER_FACTORY
#define NAVICPP_WEBSERVER_REQUESTHANDLER_FACTORY

#include "Poco/Net/HTTPRequestHandlerFactory.h"

namespace NaviFra {
class WebServerRequestHandler;
class WebServerDispatcher;

class WebServerRequestHandlerFactory : public Poco::Net::HTTPRequestHandlerFactory {
public:
    WebServerRequestHandlerFactory(WebServerDispatcher& dispatcher, bool secure);

    ~WebServerRequestHandlerFactory();

    Poco::Net::HTTPRequestHandler* createRequestHandler(const Poco::Net::HTTPServerRequest& request);

private:
    WebServerDispatcher& _dispatcher;
    bool _secure;
};
}  // namespace NaviFra

#endif  //  NAVICPP_WEBSERVER_REQUESTHANDLER_FACTORY
