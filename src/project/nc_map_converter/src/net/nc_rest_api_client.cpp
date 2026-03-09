

#include "nc_map_converter/net/nc_rest_api_client.h"

#include <Poco/Net/HTTPClientSession.h>
#include <Poco/Net/HTTPRequest.h>
#include <Poco/StreamCopier.h>

#include <cassert>
#include <memory>

using Poco::Net::HTTPClientSession;
using Poco::Net::HTTPMessage;
using Poco::Net::HTTPRequest;
using Poco::Net::HTTPResponse;

using namespace NaviBrain;

NcRESTAPIClient::NcRESTAPIClient(std::string host, std::string port)
{
    backend_host_ = host;
    backend_port_ = std::atoi(port.c_str());
}

NcRESTAPIClient::~NcRESTAPIClient()
{
}

void NcRESTAPIClient::setBackend(std::string host, std::string port)
{
    assert(!host.empty());
    assert(!port.empty());

    backend_host_ = host;
    backend_port_ = std::atoi(port.c_str());
}

std::tuple<std::string, Poco::Net::HTTPResponse::HTTPStatus, std::string> NcRESTAPIClient::postWithRetryInf(std::string chanel)
{
    HTTPClientSession cs(backend_host_, backend_port_);
    cs.setKeepAlive(false);
    HTTPRequest request("GET", chanel, HTTPMessage::HTTP_1_1);
    request.setContentType("application/json");
    request.setChunkedTransferEncoding(true);
    cs.sendRequest(request);
    HTTPResponse response;
    std::istream& is = cs.receiveResponse(response);
    std::string str;
    Poco::StreamCopier::copyToString(is, str);
    return {str, response.getStatus(), response.getReason()};
}

std::tuple<std::string, Poco::Net::HTTPResponse::HTTPStatus, std::string> NcRESTAPIClient::post(
    std::string chanel, Poco::JSON::Object data, std::string token)
{
    HTTPClientSession cs(backend_host_, backend_port_);
    cs.setKeepAlive(false);
    HTTPRequest request("POST", chanel, HTTPMessage::HTTP_1_1);
    request.setContentType("application/json");
    request.setChunkedTransferEncoding(true);

    if (token.compare("") != 0) {
        std::string authorizationHeader = "Bearer " + token;
        request.set("authorization", authorizationHeader);
    }

    std::ostringstream jsonStream;
    data.stringify(jsonStream);
    std::string jsonStr = jsonStream.str();

    std::ostream& requestStream = cs.sendRequest(request);
    requestStream << jsonStr;

    HTTPResponse response;
    std::istream& is = cs.receiveResponse(response);
    std::string str;
    Poco::StreamCopier::copyToString(is, str);
    return {str, response.getStatus(), response.getReason()};
}

std::tuple<std::string, Poco::Net::HTTPResponse::HTTPStatus, std::string> NcRESTAPIClient::get(std::string chanel, std::string token)
{
    HTTPClientSession cs(backend_host_, backend_port_);
    cs.setKeepAlive(false);
    HTTPRequest request("GET", chanel, HTTPMessage::HTTP_1_1);
    request.setContentType("application/json");
    request.setChunkedTransferEncoding(true);

    if (token.compare("") != 0) {
        std::string authorizationHeader = "Bearer " + token;
        request.set("authorization", authorizationHeader);
    }

    cs.sendRequest(request);
    HTTPResponse response;
    std::istream& is = cs.receiveResponse(response);
    std::string str;
    Poco::StreamCopier::copyToString(is, str);
    return {str, response.getStatus(), response.getReason()};
}