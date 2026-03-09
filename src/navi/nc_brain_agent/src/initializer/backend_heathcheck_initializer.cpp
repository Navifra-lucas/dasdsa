#include "nc_brain_agent/initializer/backend_heathcheck_initializer.h"

#include "util/logger.hpp"

#include <Poco/Net/HTTPResponse.h>
#include <nc_brain_agent/net/nc_rest_api_utils.h>

#include <future>
#include <tuple>

using Poco::Net::HTTPResponse;

using namespace NaviFra;

void BackendHealthCheckInitializer::initialize()
{
    const int max_retry = 30;  // 최대 5번 재시도
    const int retry_delay_ms = 1000;  // 1초 대기

    // Launch a new thread to perform the operation asynchronously
    std::future<bool> result = std::async(std::launch::async, [&]() -> bool {
        for (int attempt = 0; attempt < max_retry; ++attempt) {
            try {
                std::string ip_address_ = "127.0.0.1";
                if (std::getenv("ROBOT_IP") != NULL)
                    ip_address_ = std::string(std::getenv("ROBOT_IP"));

                LOG_INFO("ROBOT_IP: %s ", ip_address_.c_str());

                auto res = Net::HTTP::postWithRetryInf("/healthcheck");

                HTTPResponse::HTTPStatus response = std::get<1>(res);
                if (response == HTTPResponse::HTTPStatus::HTTP_OK) {
                    LOG_INFO("BackendHealthCheckInitializer success");
                    return true;
                }

                LOG_WARNING("Backend health check failed (attempt %d/%d)", attempt + 1, max_retry);
                Poco::Thread::sleep(retry_delay_ms);
            }
            catch (const std::exception& ex) {
                LOG_ERROR("Exception: %s", ex.what());
                Poco::Thread::sleep(retry_delay_ms);
            }
        }
        LOG_ERROR("Backend health check timeout after %d attempts", max_retry);
        return false;
    });

    // Wait for the asynchronous task to complete (타임아웃 10초)
    if (result.wait_for(std::chrono::seconds(10)) == std::future_status::timeout) {
        LOG_ERROR("Backend health check operation timed out");
        return;
    }

    bool success = result.get();
    if (!success) {
        LOG_ERROR("Backend health check failed");
    }
}
