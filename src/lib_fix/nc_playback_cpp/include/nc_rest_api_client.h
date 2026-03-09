
#ifndef NC_LOG_REST_API_CLIENT_H
#define NC_LOG_REST_API_CLIENT_H

#include <Poco/JSON/Object.h>
#include <Poco/Net/HTMLForm.h>
#include <Poco/Net/HTTPResponse.h>

#include <memory>

namespace NaviBrain {
class NcRESTAPIClient {
public:
    NcRESTAPIClient(std::string host = "127.0.0.1", std::string port = "5010");
    ~NcRESTAPIClient();

    using Ptr = std::shared_ptr<NcRESTAPIClient>;

public:
    std::tuple<std::string, Poco::Net::HTTPResponse::HTTPStatus, std::string> postWithRetryInf(std::string chanel);
    std::tuple<std::string, Poco::Net::HTTPResponse::HTTPStatus, std::string> post(
        std::string chanel, Poco::JSON::Object data, std::string token = "");
    std::tuple<std::string, Poco::Net::HTTPResponse::HTTPStatus, std::string> post_file(
        std::string chanel, Poco::Net::HTMLForm& data, std::string token = "");
    std::tuple<std::string, Poco::Net::HTTPResponse::HTTPStatus, std::string> get(std::string chanel, std::string token = "");

    void setBackend(std::string host, std::string port);

private:
    std::string backend_host_;
    Poco::Int16 backend_port_;
};
}  // namespace NaviBrain

#endif  // NC_REST_API_CLIENT_H