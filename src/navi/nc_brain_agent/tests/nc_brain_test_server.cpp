#include "nc_brain_test_server.h"

#include "nc_brain_map_test.h"

#include <Poco/File.h>
#include <Poco/FileStream.h>
#include <Poco/JSON/Object.h>
#include <Poco/JSON/Parser.h>
#include <Poco/Path.h>
#include <Poco/StreamCopier.h>

namespace NaviFra {

BrainTransportHandlerFactory::BrainTransportHandlerFactory()
{
}

BrainTransportHandlerFactory::~BrainTransportHandlerFactory()
{
}

Poco::Net::HTTPRequestHandler* BrainTransportHandlerFactory::createRequestHandler(const Poco::Net::HTTPServerRequest& request)
{
    return new BrainTransportHandler();
}

BrainTransportHandler::BrainTransportHandler()
{
    _router.insert(std::make_pair<std::string, controller>("/healthcheck", &BrainTransportHandler::handleRequest_post_healthcheck));
    _router.insert(std::make_pair<std::string, controller>("/robots/create", &BrainTransportHandler::handleRequest_post_robotcreate));
    _router.insert(std::make_pair<std::string, controller>("/scan-maps/main-info", &BrainTransportHandler::handleRequest_get_scanMapInfo));
    _router.insert(std::make_pair<std::string, controller>("/scan-maps/main", &BrainTransportHandler::handleRequest_get_scanMap));
}
BrainTransportHandler::~BrainTransportHandler()
{
}

void BrainTransportHandler::handleRequest(Poco::Net::HTTPServerRequest& request, Poco::Net::HTTPServerResponse& response)
{
    try {
        std::string path(request.getURI());
        if (_router.find(path) != _router.end()) {
            std::map<std::string, controller>::iterator it = _router.find(path);
            controller ct = it->second;
            (this->*ct)(request, response);
        }
        else {
            return sendJSONResponse(request, HTTPResponse::HTTP_METHOD_NOT_ALLOWED, "path: " + request.getURI() + " not found");
        }
    }
    catch (const std::exception&) {
        response.setStatusAndReason(HTTPResponse::HTTPStatus::HTTP_BAD_REQUEST);
        response.send();
    }
}

void BrainTransportHandler::handleRequest_post_healthcheck(Poco::Net::HTTPServerRequest& request, Poco::Net::HTTPServerResponse& response)
{
    response.setContentType("application/json");
    response.setStatusAndReason(HTTPResponse::HTTPStatus::HTTP_OK);

    Object message, item;
    message.set("result", "success");
    message.set("item", item);
    message.set("message", "");

    std::ostringstream ostr;
    message.stringify(ostr);

    std::ostream& responseStream = response.send();
    message.stringify(responseStream);
    response.end();
}

void BrainTransportHandler::handleRequest_post_robotcreate(Poco::Net::HTTPServerRequest& request, Poco::Net::HTTPServerResponse& response)
{
    Poco::JSON::Parser parser;
    Poco::JSON::Object::Ptr jsonObj = parser.parse(request.stream()).extract<Poco::JSON::Object::Ptr>();

    robot_create_type robot;

    robot.model_type_cd = jsonObj->get("model_type_cd").convert<std::string>();
    robot.driving_type_cd = jsonObj->get("driving_type_cd").convert<std::string>();
    robot.ip_addr = jsonObj->get("ip_addr").convert<std::string>();
    robot.robot_acs_reg_code = jsonObj->get("robot_acs_reg_code").convert<std::string>();

    response.setContentType("application/json");

    if (robots_.find(robot.ip_addr) != robots_.end()) {
        response.setStatusAndReason(HTTPResponse::HTTPStatus::HTTP_BAD_REQUEST);
        std::ostream& responseStream = response.send();
        _robotObject[robot.ip_addr].stringify(responseStream);
        response.end();
    }
    else {
        robots_[robot.ip_addr] = robot;
        response.setStatusAndReason(status == Poco::Net::HTTPResponse::HTTPStatus::HTTP_OK);

        Object message, item;
        message.set("result", "success");
        message.set("message", "");

        item.set("id", "de8b7c36-69ed-46a6-981c-9cb03f0ff66b");
        item.set("model_type_cd", "D10001");
        item.set("driving_type_cd", "D30001");
        item.set("name", "QD-Frk-004");
        item.set(
            "token",
            "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJpZCI6ImRlOGI3YzM2LTY5ZWQtNDZhNi05ODFjLTljYjAzZjBmZjY2YiIsIm1vZGVsX3R5cGUiOiJEMTAwMDEiLCJuYW1lIjoiUUQtRnJrLTAwNCIsImxldmVsIjo1LCJ0b2tlbl90eXBlIjoiZGV2aWNlIiwiaWF0IjoxNjg5NzUxMjU1LCJleHAiOjMzMjI1NzUxMjU1fQ.ZwrydYGdioUVRrSHmN88wlbvZ6judl26KCkLAkkyhM8");
        item.set("token_expire_at", "2024-07-19 01:20:55");
        item.set("is_registered", true);
        item.set("token_expire_at", "2024-07-19 01:20:55");
        item.set("is_enabled", false);
        item.set("desc", "");
        item.set("ip_addr", "192.168.0.4");
        item.set("alias_name", "QD-Frk-004");
        item.set("job_is_active", false);
        item.set("created_at", "2023-07-19 16:20:55");
        item.set("updated_at", "2023-07-19 16:20:55");
        item.set("deleted_at", NULL);

        message.set("item", item);

        std::ostream& responseStream = response.send();
        message.stringify(responseStream);
        response.end();

        _robotObject[robot.ip_addr] = message;
    }
}

void BrainTransportHandler::handleRequest_get_scanMapInfo(Poco::Net::HTTPServerRequest& request, Poco::Net::HTTPServerResponse& response)
{
    response.setContentType("application/json");
    response.setStatusAndReason(HTTPResponse::HTTPStatus::HTTP_OK);

    Object message, item;
    message.set("result", "success");

    item.set("resolution", 0.01);
    item.set("offset_x", 0);
    item.set("offset_y", 0);
    item.set("id", "de8b7c36-69ed-46a6-981c-9cb03f0ff66b");
    item.set("floor", 1);
    item.set("revision", 103);
    item.set("title", "office_volvo");
    item.set("is_master", true);
    item.set("width", 14);
    item.set("height", 15);
    item.set("created_at", "2023-07-18 11:17:07.312000");
    item.set("updated_at", "2023-07-18 13:44:26.202000");
    item.set("deleted_at", NULL);
    message.set("item", item);
    message.set("message", "");

    std::ostream& responseStream = response.send();
    message.stringify(responseStream);
    response.end();
}

void BrainTransportHandler::handleRequest_get_scanMap(Poco::Net::HTTPServerRequest& request, Poco::Net::HTTPServerResponse& response)
{
    response.setContentType("application/json");
    response.setStatusAndReason(HTTPResponse::HTTPStatus::HTTP_OK);

    Object message;
    message.set("result", "success");
    if (Poco::File(std::string(TEST_FILE_PATH) + "scan_map.json").exists()) {
        std::ostringstream ostr;
        Poco::FileInputStream fis(std::string(TEST_FILE_PATH) + "scan_map.json");
        Poco::StreamCopier::copyStream(fis, ostr);
        Poco::JSON::Parser parser;
        Poco::Dynamic::Var result = parser.parse(ostr.str());
        Poco::JSON::Object::Ptr data = result.extract<Poco::JSON::Object::Ptr>();
        message.set("item", data->getObject("item"));
    }

    message.set("message", "");

    std::ostream& responseStream = response.send();
    message.stringify(responseStream);
    response.end();
}

void BrainTransportHandler::sendJSONResponse(
    Poco::Net::HTTPServerRequest& request, Poco::Net::HTTPResponse::HTTPStatus status, const std::string& message)
{
    request.response().setContentType("application/json");
    request.response().setStatusAndReason(HTTPResponse::HTTP_METHOD_NOT_ALLOWED);

    std::string json(Poco::format(
        "{ \"error\": \"%s\", \"detail\": \"%s\", \"code\": %d }",
        request.response().getReasonForStatus(HTTPResponse::HTTP_METHOD_NOT_ALLOWED), message,
        static_cast<int>(HTTPResponse::HTTP_METHOD_NOT_ALLOWED)));
    request.response().sendBuffer(json.data(), json.size());
}
}  // namespace NaviFra