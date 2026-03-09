#include <Poco/JSON/Object.h>
#include <Poco/Net/HTTPClientSession.h>
#include <Poco/Net/HTTPRequest.h>
#include <Poco/Net/HTTPRequestHandler.h>
#include <Poco/Net/HTTPServer.h>
#include <Poco/Net/HTTPServerRequest.h>
#include <Poco/Net/HTTPServerResponse.h>
#include <Poco/StreamCopier.h>
#include <Poco/Timestamp.h>
#include <Poco/URI.h>

using Poco::JSON::Object;
using Poco::Net::HTTPClientSession;
using Poco::Net::HTTPMessage;
using Poco::Net::HTTPRequest;
using Poco::Net::HTTPResponse;
using Poco::Net::HTTPServer;

namespace NaviFra {

class BrainTransportHandler : public Poco::Net::HTTPRequestHandler {
    typedef void (BrainTransportHandler::*controller)(Poco::Net::HTTPServerRequest&, Poco::Net::HTTPServerResponse&);

    struct robot_create_type {
        std::string model_type_cd;
        std::string driving_type_cd;
        std::string ip_addr;
        std::string robot_acs_reg_code;
    };

public:
    BrainTransportHandler();
    ~BrainTransportHandler();

    void handleRequest(Poco::Net::HTTPServerRequest& request, Poco::Net::HTTPServerResponse& response);

private:
    std::map<std::string, robot_create_type> robots_;

    void handleRequest_post_healthcheck(Poco::Net::HTTPServerRequest& request, Poco::Net::HTTPServerResponse& response);
    void handleRequest_post_robotcreate(Poco::Net::HTTPServerRequest& request, Poco::Net::HTTPServerResponse& response);
    void handleRequest_get_scanMapInfo(Poco::Net::HTTPServerRequest& request, Poco::Net::HTTPServerResponse& response);
    void handleRequest_get_scanMap(Poco::Net::HTTPServerRequest& request, Poco::Net::HTTPServerResponse& response);

    void sendJSONResponse(Poco::Net::HTTPServerRequest& request, Poco::Net::HTTPResponse::HTTPStatus status, const std::string& message);

private:
    std::map<std::string, controller> _router;
    std::map<std::string, Object> _robotObject;
};

class BrainTransportHandlerFactory : public Poco::Net::HTTPRequestHandlerFactory {
public:
    BrainTransportHandlerFactory();
    ~BrainTransportHandlerFactory();

    Poco::Net::HTTPRequestHandler* createRequestHandler(const Poco::Net::HTTPServerRequest& request);
};
}  // namespace NaviFra