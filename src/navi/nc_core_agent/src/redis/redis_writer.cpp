#include "core_agent/core_agent.h"

#include <Poco/Delegate.h>
#include <Poco/Redis/Command.h>
#include <Poco/Redis/Type.h>
#include <core_agent/redis/redis_writer.h>

using namespace Poco::Redis;
using namespace Poco::JSON;

using namespace NaviFra;

RedisWriter::RedisWriter(bool use_auth)
    : use_auth_(use_auth)
{
    hostname_ = Config::instance().getString("redis_host", "127.0.0.1");
    port_ = Config::instance().getInt("redis_port", 6379);
    password_ = Config::instance().getString("redis_passwd", "navifra1@3$");
    use_auth_ = Config::instance().getBool("use_auth", true);

    initialize();
}

RedisWriter::~RedisWriter()
{
    disconnect();
}

bool RedisWriter::initialize(const char* hostname, int port, const char* password)
{
    hostname_ = hostname, port_ = port, password_ = password;
    return initialize();
}

bool RedisWriter::initialize()
{
    try {
        if (connect()) {
            if (use_auth_) {
                Poco::Redis::Array command;
                command.add("AUTH").add(Poco::format("%s", password_));
                std::string reply = client_.execute<std::string>(command);

                if (reply.compare("OK") != 0) {
                    LOG_ERROR("인증 실패");
                    return false;
                }
            }
            LOG_INFO("RedisWriter initialize success");
            return true;
        }
    }
    catch (const std::exception& e) {
        LOG_ERROR("%s", e.what());
    }

    return false;
}

bool RedisWriter::isConnected()
{
    if (!client_.isConnected())
        return false;
    return true;
    Poco::Redis::Array command;
    command.add("PING");
    std::string reply = client_.execute<std::string>(command);
    if (reply.compare("PONG") == 0 || reply.compare("pong") == 0)
        return true;
    else
        return false;
}

bool RedisWriter::connect()
{
    /*
    이미 연결 되어 있을 경우? 예외 처리를 어떻게 할 것 인가? 포트를 바꿔서 연결 할 일이 있나?
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

void RedisWriter::disconnect()
{
    if (client_.isConnected()) {
        client_.disconnect();
    }
}

void RedisWriter::reconnect()
{
    disconnect();
    initialize();
}

void RedisWriter::publish(std::string channel, std::string content)
{
    Poco::FastMutex::ScopedLock lock(_fastMutex);
    if (client_.isConnected() != true) {
        reconnect();
    }

    Poco::Redis::Array command;
    command.add("PUBLISH").add(channel).add(content);
    client_.sendCommand(command);
}

void RedisWriter::set(const std::string& key, const std::string& value)
{
    try {
        Poco::FastMutex::ScopedLock lock(_fastMutex);
        if (client_.isConnected() != true) {
            reconnect();
        }
        Poco::Redis::Command cmd = Poco::Redis::Command::set(key, value);
        client_.execute<void>(cmd);
        client_.flush();
    }
    catch (const Poco::Exception& ex) {
        std::cerr << "Redis set error: " << ex.displayText() << std::endl;
    }
}

std::string RedisWriter::get(const std::string& key)
{
    try {
        Poco::FastMutex::ScopedLock lock(_fastMutex);
        if (client_.isConnected() != true) {
            reconnect();
        }
        Poco::Redis::Command cmd = Poco::Redis::Command::get(key);
        Poco::Redis::BulkString result = client_.execute<Poco::Redis::BulkString>(cmd);
        return result.value();
    }
    catch (const Poco::Exception& ex) {
        std::cerr << "Redis get error: " << ex.displayText() << std::endl;
        return "";
    }
}

std::vector<std::string> RedisWriter::getKeys(const std::string& pattern)
{
    try {
        Poco::FastMutex::ScopedLock lock(_fastMutex);
        if (client_.isConnected() != true) {
            reconnect();
        }
        Poco::Redis::Command cmd("KEYS");
        cmd << pattern;
        Poco::Redis::Array result = client_.execute<Poco::Redis::Array>(cmd);
        std::vector<std::string> keys;

        for (size_t index = 0; index < result.size(); index++) {
            keys.push_back(result.get<BulkString>(index).value());
        }
        return keys;
    }
    catch (const Poco::Exception& ex) {
        std::cerr << "Redis get keys error: " << ex.displayText() << std::endl;
        return {};
    }
}

void RedisWriter::del(const std::string& key)
{
    try {
        Poco::FastMutex::ScopedLock lock(_fastMutex);
        if (client_.isConnected() != true) {
            reconnect();
        }
        Poco::Redis::Command cmd = Poco::Redis::Command::del(key);
        client_.execute<void>(cmd);
        client_.flush();
    }
    catch (const Poco::Exception& ex) {
        std::cerr << "Redis delete error: " << ex.displayText() << std::endl;
    }
}