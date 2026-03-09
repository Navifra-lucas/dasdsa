#include "Poco/DateTimeFormat.h"
#include "Poco/DateTimeFormatter.h"
#include "Poco/File.h"
#include "Poco/FileStream.h"
#include "Poco/JSON/Object.h"
#include "Poco/Net/HTTPServerRequest.h"
#include "Poco/Net/HTTPServerResponse.h"
#include "Poco/StreamCopier.h"
#include "Poco/Timestamp.h"
#include "Poco/URI.h"
#include "nc_brain_agent/nc_brain_agent.h"

#include <Poco/DirectoryIterator.h>
#include <nc_brain_agent/web/router/logTransportHandler.h>

using namespace Poco::Net;
using namespace Poco::JSON;

std::string g_HOME = std::getenv("HOME");

namespace NaviFra {
LogTransportHandlerFactory::LogTransportHandlerFactory()
{
}

LogTransportHandlerFactory::~LogTransportHandlerFactory()
{
}

Poco::Net::HTTPRequestHandler* LogTransportHandlerFactory::createRequestHandler(const Poco::Net::HTTPServerRequest& request)
{
    return new LogTransportHandler();
}

LogTransportHandler::LogTransportHandler()
{
    _router.insert(std::make_pair<std::string, controller>("/api/log", &LogTransportHandler::handleRequest_get_log));
    _router.insert(std::make_pair<std::string, controller>("/api/log/category", &LogTransportHandler::handleRequest_get_category));
    _router.insert(std::make_pair<std::string, controller>("/api/log/list", &LogTransportHandler::handleRequest_get_list));
}

LogTransportHandler::~LogTransportHandler()
{
}

void LogTransportHandler::handleRequest(Poco::Net::HTTPServerRequest& request, Poco::Net::HTTPServerResponse& response)
{
    try {
        Poco::URI uri(request.getURI());
        std::string path(uri.getPath());
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

void LogTransportHandler::handleRequest_get_category(Poco::Net::HTTPServerRequest& request, Poco::Net::HTTPServerResponse& response)
{
    try {
        response.setContentType("application/json");
        response.setStatusAndReason(HTTPResponse::HTTPStatus::HTTP_OK);
        std::vector<std::string> category;

        Poco::Path basePath(g_HOME + "/navifra_solution/navicore/logs");  // 원하는 폴더의 경로를 지정합니다.

        Poco::DirectoryIterator it(basePath);
        Poco::DirectoryIterator end;

        for (; it != end; ++it) {
            if (it->isDirectory()) {
                category.push_back(it.name());
            }
        }

        Object message;
        message.set("category", category);
        std::ostringstream ostr;
        message.stringify(ostr);

        std::ostream& responseStream = response.send();
        message.stringify(responseStream);
        response.end();
    }
    catch (const Poco::Exception& ex) {
        NLOG(error) << ex.displayText();
        response.end();
    }
}

void LogTransportHandler::handleRequest_get_list(Poco::Net::HTTPServerRequest& request, Poco::Net::HTTPServerResponse& response)
{
    try {
        response.setContentType("application/json");
        response.setStatusAndReason(HTTPResponse::HTTPStatus::HTTP_OK);
        Poco::URI uri(request.getURI());
        Poco::URI::QueryParameters query(uri.getQueryParameters());

        std::string category;

        if (query.size() != 0) {
            for (std::vector<std::pair<std::string, std::string>>::iterator curParamIt = query.begin(); curParamIt != query.end();
                 ++curParamIt) {
                if (std::strcmp("category", curParamIt->first.c_str()) == 0)
                    category = curParamIt->second;
            }
        }
        else {
            return sendJSONResponse(
                request, HTTPResponse::HTTP_METHOD_NOT_ALLOWED, "path: " + request.getURI() + " category has Not found");
        }

        std::vector<std::string> files;
        Poco::File f(g_HOME + "/navifra_solution/navicore/logs/" + category);
        if (f.exists()) {
            f.list(files);
        }

        Poco::JSON::Array logfiles, errorfiles;

        for (std::vector<std::string>::iterator it = files.begin(); it != files.end(); it++) {
            if (it->find(".log") != std::string::npos)
                logfiles.add(*it);
        }

        Poco::File ef(g_HOME + "/navifra_solution/navicore/logs/" + category + "/error");
        if (ef.exists()) {
            ef.list(files);
        }

        for (std::vector<std::string>::iterator it = files.begin(); it != files.end(); it++) {
            if (it->find(".log") != std::string::npos)
                errorfiles.add(*it);
        }

        Object message;
        message.set("files", logfiles);
        message.set("error", errorfiles);
        std::ostringstream ostr;
        message.stringify(ostr);

        std::ostream& responseStream = response.send();
        message.stringify(responseStream);
        response.end();
    }
    catch (const Poco::Exception& ex) {
        NLOG(error) << ex.displayText();
        response.end();
    }
}

void LogTransportHandler::handleRequest_get_log(Poco::Net::HTTPServerRequest& request, Poco::Net::HTTPServerResponse& response)
{
    response.setContentType("text/plain");
    response.setStatusAndReason(HTTPResponse::HTTPStatus::HTTP_OK);

    Poco::URI uri(request.getURI());
    Poco::URI::QueryParameters query(uri.getQueryParameters());

    std::string fileName;
    std::string category;

    if (query.size() != 0) {
        for (std::vector<std::pair<std::string, std::string>>::iterator curParamIt = query.begin(); curParamIt != query.end();
             ++curParamIt) {
            if (std::strcmp("fileName", curParamIt->first.c_str()) == 0)
                fileName = curParamIt->second;

            if (std::strcmp("category", curParamIt->first.c_str()) == 0)
                category = curParamIt->second;
        }
    }
    else {
        return sendJSONResponse(request, HTTPResponse::HTTP_METHOD_NOT_ALLOWED, "path: " + request.getURI() + " fileName has Not found");
    }

    std::string logfilePath;

    if (fileName.find("error") != std::string::npos) {
        logfilePath = g_HOME + "/navifra_solution/navicore/logs/" + category + "/error/" + fileName;
    }
    else {
        logfilePath = g_HOME + "/navifra_solution/navicore/logs/" + category + "/" + fileName;
    }

    Poco::File logFile(logfilePath);

    if (!logFile.exists() || fileName.empty() || category.empty()) {
        return sendJSONResponse(request, HTTPResponse::HTTP_METHOD_NOT_ALLOWED, "path: " + request.getURI() + " file does Not exits");
    }

    ////response.
    std::unique_ptr<std::istream> pResourceStream(new Poco::FileInputStream(logfilePath));

    if (pResourceStream.get()) {
        std::ostream& responseStream = response.send();
        Poco::StreamCopier::copyStream(*pResourceStream, responseStream);
    }
    else {
        return sendJSONResponse(request, HTTPResponse::HTTP_METHOD_NOT_ALLOWED, "path: " + request.getURI() + " file has Not found");
    }

    response.end();
}

void LogTransportHandler::sendJSONResponse(
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