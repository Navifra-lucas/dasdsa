#include <nc_brain_agent/web/common/webServerDispatcher.h>
#include <nc_brain_agent/web/common/webServerRequestHandler.h>

namespace NaviFra {
WebServerRequestHandler::WebServerRequestHandler(WebServerDispatcher& dispatcher, bool secure)
    : _dispatcher(dispatcher)
    , _secure(secure)
{
}

WebServerRequestHandler::~WebServerRequestHandler()
{
}

void WebServerRequestHandler::handleRequest(Poco::Net::HTTPServerRequest& request, Poco::Net::HTTPServerResponse& response)
{
    _dispatcher.handleRequest(request, response, _secure);
}

}  // namespace NaviFra