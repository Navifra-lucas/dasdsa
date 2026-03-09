
#ifndef NC_REST_API_CLIENT_H
#define NC_REST_API_CLIENT_H

#include <Poco/JSON/Object.h>
#include <Poco/Net/HTTPResponse.h>

namespace NaviFra {
namespace Net {
namespace HTTP {
std::tuple<std::string, Poco::Net::HTTPResponse::HTTPStatus, std::string> postWithRetryInf(std::string chanel);
std::tuple<std::string, Poco::Net::HTTPResponse::HTTPStatus, std::string> post(
    std::string chanel, Poco::JSON::Object data, std::string token = "");
std::tuple<std::string, Poco::Net::HTTPResponse::HTTPStatus, std::string> get(std::string chanel, std::string token = "");
}  // namespace HTTP
}  // namespace Net
}  // namespace NaviFra

#endif  // NC_REST_API_CLIENT_H