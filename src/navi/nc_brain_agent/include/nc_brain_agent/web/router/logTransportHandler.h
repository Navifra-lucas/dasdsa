#ifndef NAVICPP_LOG_TRANSPORT_REQUEST_HANDELER_H
#define NAVICPP_LOG_TRANSPORT_REQUEST_HANDELER_H

#include "Poco/Net/HTTPRequestHandler.h"
#include "Poco/Net/HTTPRequestHandlerFactory.h"
#include "Poco/Net/HTTPResponse.h"

#include <map>

namespace NaviFra {
class LogTransportHandler : public Poco::Net::HTTPRequestHandler {
    typedef void (LogTransportHandler::*controller)(Poco::Net::HTTPServerRequest&, Poco::Net::HTTPServerResponse&);

public:
    LogTransportHandler();
    ~LogTransportHandler();

    void handleRequest(Poco::Net::HTTPServerRequest& request, Poco::Net::HTTPServerResponse& response);

private:
    void handleRequest_get_category(Poco::Net::HTTPServerRequest& request, Poco::Net::HTTPServerResponse& response);
    void handleRequest_get_list(Poco::Net::HTTPServerRequest& request, Poco::Net::HTTPServerResponse& response);
    void handleRequest_get_log(Poco::Net::HTTPServerRequest& request, Poco::Net::HTTPServerResponse& response);
    void sendJSONResponse(Poco::Net::HTTPServerRequest& request, Poco::Net::HTTPResponse::HTTPStatus status, const std::string& message);

private:
    std::map<std::string, controller> _router;
    std::string LOGFILE_ROOT;
};

class LogTransportHandlerFactory : public Poco::Net::HTTPRequestHandlerFactory {
public:
    LogTransportHandlerFactory();

    ~LogTransportHandlerFactory();

    Poco::Net::HTTPRequestHandler* createRequestHandler(const Poco::Net::HTTPServerRequest& request);
};
}  // namespace NaviFra

#endif  // NAVICPP_LOG_TRANSPORT_REQUEST_HANDELER_H
