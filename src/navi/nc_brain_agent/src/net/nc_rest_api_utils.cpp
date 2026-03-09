

#include "nc_brain_agent/nc_brain_agent.h"

#include <Poco/Net/AcceptCertificateHandler.h>
#include <Poco/Net/Context.h>
#include <Poco/Net/HTTPSClientSession.h>
#include <Poco/Net/SSLManager.h>
#include <nc_brain_agent/net/nc_rest_api_utils.h>
#include <nc_brain_agent/utils/nc_agent_config.h>

using Poco::Net::HTTPClientSession;
using Poco::Net::HTTPMessage;
using Poco::Net::HTTPRequest;
using Poco::Net::HTTPResponse;

namespace NaviFra::Net::HTTP {

void initSSL()
{
    static std::once_flag sslInitFlag;
    std::call_once(sslInitFlag, [] {
        Poco::SharedPtr<Poco::Net::InvalidCertificateHandler> pCert = new Poco::Net::AcceptCertificateHandler(false);
        Poco::Net::Context::Ptr pContext =
            new Poco::Net::Context(Poco::Net::Context::CLIENT_USE, "", "", "", Poco::Net::Context::VERIFY_RELAXED);
        Poco::Net::SSLManager::instance().initializeClient(0, pCert, pContext);
    });
}

std::tuple<std::string, Poco::Net::HTTPResponse::HTTPStatus, std::string> postWithRetryInf(std::string chanel)
{
    try {
        std::unique_ptr<Poco::Net::HTTPClientSession> session;

        const auto& config = NaviFra::Config::instance();
        bool useSSL = config.getBool("use_ssl", false);
        std::string host = config.getString("backend_host", "127.0.0.1");
        int port = config.getInt("backend_port", 5000);
        std::string basePath = useSSL ? "/api/" : "";

        if (!basePath.empty() && basePath.back() == '/' && chanel.front() == '/') {
            basePath.pop_back();  // 슬래시 중복 제거
        }

        std::string fullPath = basePath + chanel;

        if (useSSL) {
            initSSL();
            session = std::make_unique<Poco::Net::HTTPSClientSession>(
                NaviFra::Config::instance().getString("backend_host", "127.0.0.1"),
                NaviFra::Config::instance().getInt("backend_port", 5000));
        }
        else {
            session = std::make_unique<Poco::Net::HTTPClientSession>(
                NaviFra::Config::instance().getString("backend_host", "127.0.0.1"),
                NaviFra::Config::instance().getInt("backend_port", 5000));
        }

        session->setKeepAlive(false);
        HTTPRequest request("GET", fullPath, HTTPMessage::HTTP_1_1);
        request.setContentType("application/json");
        request.setChunkedTransferEncoding(true);
        session->sendRequest(request);
        HTTPResponse response;
        std::istream& is = session->receiveResponse(response);
        std::string str;
        Poco::StreamCopier::copyToString(is, str);
        return {str, response.getStatus(), response.getReason()};
    }
    catch (Poco::Exception ex) {
        NLOG(error) << Poco::format("postWithRetryInf error route %s %s", chanel, ex.displayText());
        return {"", HTTPResponse::HTTPStatus::HTTP_EXPECTATION_FAILED, "Throw Exception"};
    }
}

std::tuple<std::string, Poco::Net::HTTPResponse::HTTPStatus, std::string> post(
    std::string chanel, Poco::JSON::Object data, std::string token)
{
    std::unique_ptr<Poco::Net::HTTPClientSession> session;

    const auto& config = NaviFra::Config::instance();
    bool useSSL = config.getBool("use_ssl", false);
    std::string host = config.getString("backend_host", "127.0.0.1");
    int port = config.getInt("backend_port", 5000);
    std::string basePath = useSSL ? "/api/" : "";

    if (!basePath.empty() && basePath.back() == '/' && chanel.front() == '/') {
        basePath.pop_back();  // 슬래시 중복 제거
    }

    std::string fullPath = basePath + chanel;

    if (useSSL) {
        initSSL();
        session = std::make_unique<Poco::Net::HTTPSClientSession>(
            NaviFra::Config::instance().getString("backend_host", "127.0.0.1"), NaviFra::Config::instance().getInt("backend_port", 5000));
    }
    else {
        session = std::make_unique<Poco::Net::HTTPClientSession>(
            NaviFra::Config::instance().getString("backend_host", "127.0.0.1"), NaviFra::Config::instance().getInt("backend_port", 5000));
    }

    session->setKeepAlive(false);
    HTTPRequest request("POST", fullPath, HTTPMessage::HTTP_1_1);
    request.setContentType("application/json");
    request.setChunkedTransferEncoding(true);

    if (token.compare("") != 0) {
        std::string authorizationHeader = "Bearer " + token;
        request.set("authorization", authorizationHeader);
    }

    std::ostringstream jsonStream;
    data.stringify(jsonStream);
    std::string jsonStr = jsonStream.str();
    std::ostream& requestStream = session->sendRequest(request);
    requestStream << jsonStr;

    std::ostringstream teststream;
    request.write(teststream);

    HTTPResponse response;
    std::istream& is = session->receiveResponse(response);
    std::string str;
    Poco::StreamCopier::copyToString(is, str);
    return {str, response.getStatus(), response.getReason()};
}

std::tuple<std::string, Poco::Net::HTTPResponse::HTTPStatus, std::string> get(std::string chanel, std::string token)
{
    std::unique_ptr<Poco::Net::HTTPClientSession> session;

    const auto& config = NaviFra::Config::instance();
    bool useSSL = config.getBool("use_ssl", false);
    std::string host = config.getString("backend_host", "127.0.0.1");
    int port = config.getInt("backend_port", 5000);
    std::string basePath = useSSL ? "/api/" : "";

    if (!basePath.empty() && basePath.back() == '/' && chanel.front() == '/') {
        basePath.pop_back();  // 슬래시 중복 제거
    }

    std::string fullPath = basePath + chanel;

    if (useSSL) {
        initSSL();
        session = std::make_unique<Poco::Net::HTTPSClientSession>(
            NaviFra::Config::instance().getString("backend_host", "127.0.0.1"), NaviFra::Config::instance().getInt("backend_port", 5000));
    }
    else {
        session = std::make_unique<Poco::Net::HTTPClientSession>(
            NaviFra::Config::instance().getString("backend_host", "127.0.0.1"), NaviFra::Config::instance().getInt("backend_port", 5000));
    }

    session->setKeepAlive(false);
    HTTPRequest request("GET", fullPath, HTTPMessage::HTTP_1_1);
    request.setContentType("application/json");
    request.setChunkedTransferEncoding(true);

    if (token.compare("") != 0) {
        std::string authorizationHeader = "Bearer " + token;
        request.set("authorization", authorizationHeader);
    }

    session->sendRequest(request);
    HTTPResponse response;
    std::istream& is = session->receiveResponse(response);
    std::string str;
    Poco::StreamCopier::copyToString(is, str);
    return {str, response.getStatus(), response.getReason()};
}
}  // namespace NaviFra::Net::HTTP