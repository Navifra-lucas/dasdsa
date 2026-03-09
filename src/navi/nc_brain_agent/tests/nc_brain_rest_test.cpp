#include "nc_brain_agent/nc_brain_agent.h"
#include "nc_brain_map_test.h"
#include "nc_brain_test_server.h"

#include <Poco/JSON/Parser.h>
#include <gtest/gtest.h>
#include <nc_brain_agent/net/nc_rest_api_utils.h>
#include <nc_brain_agent/utils/nc_agent_config.h>

namespace NaviFra {
class BrainRESTFixture : public ::testing::Test {
protected:
    void SetUp() override
    {
        srv.reset(new HTTPServer(new BrainTransportHandlerFactory, 5000));
        srv->start();

        Config::instance().setString("backend_host", "127.0.0.1");
        Config::instance().setInt("backend_port", 5000);
    }

    void TearDown() override
    {
        srv->stop();
        srv.reset();
    }

private:
    std::shared_ptr<HTTPServer> srv;
};

TEST_F(BrainRESTFixture, postWithRetryInf)
{
    std::tuple<std::string, Poco::Net::HTTPResponse::HTTPStatus, std::string> res = NaviFra::Net::HTTP::postWithRetryInf("/healthcheck");

    std::string is = std::get<0>(res);
    HTTPResponse::HTTPStatus response = std::get<1>(res);

    EXPECT_EQ(response, HTTPResponse::HTTPStatus::HTTP_OK);
    Poco::JSON::Parser parser;

    Poco::Dynamic::Var result = parser.parse(is);
    Poco::JSON::Object::Ptr data = result.extract<Poco::JSON::Object::Ptr>();

    EXPECT_EQ(data->has("item"), true);
    EXPECT_EQ(data->has("message"), true);
    EXPECT_EQ(data->has("result"), true);

    EXPECT_EQ(data->get("message").convert<std::string>(), "");
    EXPECT_EQ(data->get("result").convert<std::string>(), "success");
}

}  // namespace NaviFra