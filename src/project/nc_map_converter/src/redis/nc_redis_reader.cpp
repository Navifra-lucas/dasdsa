#include "nc_map_converter/redis/nc_redis_reader.h"

#include "core/util/logger.hpp"

#include <Poco/Delegate.h>
#include <Poco/JSON/Object.h>
#include <Poco/Redis/Array.h>
#include <Poco/Redis/Command.h>
#include <Poco/Redis/RedisEventArgs.h>
#include <Poco/Util/Application.h>

#include <memory>

using namespace Poco::Redis;
using namespace Poco::JSON;
using namespace NaviBrain;

using Poco::Redis::Array;
using Poco::Redis::Command;

NcRedisReader::NcRedisReader(bool use_auth)
    : use_auth_(use_auth)
{
}

NcRedisReader::~NcRedisReader()
{
    disconnect();
}

bool NcRedisReader::initialize(const char* hostname, int port, const char* password)
{
    hostname_ = hostname, port_ = port, password_ = password;
    return initialize();
}

bool NcRedisReader::initialize()
{
    try {
        if (connect()) {
            if (use_auth_) {
                Poco::FastMutex::ScopedLock lock(_fastMutex);
                Poco::Redis::Array command;
                command.add("AUTH").add(Poco::format("%s", password_));
                std::string reply = client_.execute<std::string>(command);

                if (reply.compare("OK") != 0) {
                    LOG_ERROR("인증 실패");
                    return false;
                }
            }
            LOG_INFO("NcRedisReader initialize success");
            return true;
        }
    }
    catch (const Poco::ReadFileException& e) {
        LOG_ERROR("%s", e.what());
    }

    return false;
}

bool NcRedisReader::isConnected()
{
    if (!client_.isConnected())
        return false;
    // Poco::FastMutex::ScopedLock lock(_fastMutex);
    Poco::Redis::Array command;
    command.add("PING");
    std::string reply = client_.execute<std::string>(command);
    client_.flush();
    if (reply.compare("PONG") == 0 || reply.compare("pong") == 0)
        return true;
    else
        return false;
    return true;
}

bool NcRedisReader::connect()
{
    /*
    이미 연결 되어 있을 경우? 예외 처리를 어떻게 할 것 인가? 포트를 바꿔서 연결 할
    일이 있나?
    */
    if (!client_.isConnected()) {
        try {
            client_.connect(hostname_, port_);
            if (client_.isConnected()) {
                return true;
            }
            else {
                return false;
            }
        }
        catch (const std::exception& e) {
            LOG_ERROR("%s", e.what());
            return false;
        }
    }

    return true;
}

void NcRedisReader::disconnect()
{
    if (client_.isConnected()) {
        client_.disconnect();
    }
}

void NcRedisReader::reconnect()
{
    disconnect();
    initialize();
}

Poco::Redis::Array NcRedisReader::HMGET(const std::string hash, std::vector<std::string> fileds)
{
    Poco::Redis::Array result;
    if (isConnected()) {
        try {
            Poco::Redis::Command hmget = Command::hmget(hash, fileds);
            result = client_.execute<Poco::Redis::Array>(hmget);
            return result;
        }
        catch (const std::exception& e) {
            LOG_ERROR("%s", e.what());
            return result;
        }
    }

    return result;
}

std::string NcRedisReader::GET(const std::string key)
{
    Poco::Redis::Command command = Poco::Redis::Command::get(key);
    BulkString result = client_.execute<BulkString>(command);

    return result.value();
}