#include <nc_brain_agent/web/common/webServerRequestHandler.h>
#include <nc_brain_agent/web/common/webServerRequestHandlerFactory.h>

namespace NaviFra {
WebServerRequestHandlerFactory::WebServerRequestHandlerFactory(WebServerDispatcher& dispatcher, bool secure)
    : _dispatcher(dispatcher)
    , _secure(secure)
{
}

WebServerRequestHandlerFactory::~WebServerRequestHandlerFactory()
{
}

Poco::Net::HTTPRequestHandler* WebServerRequestHandlerFactory::createRequestHandler(const Poco::Net::HTTPServerRequest& request)
{
    return new WebServerRequestHandler(_dispatcher, _secure);
}

}  // namespace NaviFra