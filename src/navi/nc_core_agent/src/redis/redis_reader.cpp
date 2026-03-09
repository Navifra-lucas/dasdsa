#include "core_agent/core_agent.h"

#include <Poco/Delegate.h>
#include <Poco/Redis/Array.h>
#include <Poco/Redis/Command.h>
#include <Poco/Redis/RedisEventArgs.h>
#include <Poco/Util/Application.h>
#include <core_agent/redis/redis_reader.h>

using namespace Poco::Redis;
using namespace Poco::JSON;
using namespace NaviFra;

using Poco::Redis::Array;
using Poco::Redis::Command;

RedisReader::RedisReader(bool use_auth)
    : use_auth_(use_auth)
{
}

RedisReader::~RedisReader()
{
    disconnect();
}

bool RedisReader::initialize(const char* hostname, int port, const char* password)
{
    hostname_ = hostname, port_ = port, password_ = password;
    return initialize();
}

bool RedisReader::initialize()
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
            LOG_INFO("RedisReader initialize success");
            return true;
        }
    }
    catch (const Poco::Exception& e) {
        LOG_ERROR("%s", e.what());
    }

    return false;
}

bool RedisReader::isConnected()
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

bool RedisReader::connect()
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

void RedisReader::disconnect()
{
    if (client_.isConnected()) {
        client_.disconnect();
    }
}

void RedisReader::reconnect()
{
    disconnect();
    initialize();
}

Poco::Redis::Array RedisReader::HMGET(const std::string hash, std::vector<std::string> fileds)
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

std::string RedisReader::GET(const std::string key)
{
    try {
        Poco::Redis::Command command = Poco::Redis::Command::get(key);
        BulkString result = client_.execute<BulkString>(command);
        return result.value();
    }
    catch (Poco::Redis::RedisException& ex) {
        NLOG(error) << ex.displayText();
        return "";
    }
}

void RedisReader::hset(const std::string& hash, const std::string& key, const std::string& value)
{
    try {
        if (client_.isConnected()) {
            if (use_auth_) {
                Poco::Redis::Array auth;
                auth.add("AUTH").add(password_);
                auto authResult = client_.execute<std::string>(auth);

                if (authResult.compare("OK") != 0) {
                    LOG_ERROR("인증 실패");
                    return;
                }
            }

            std::vector<Poco::Redis::Array> commands;
            Poco::Redis::Array command;
            command.add("HSET").add(hash).add(key).add(value);
            commands.push_back(command);

            client_.sendCommands(commands);
        }
    }
    catch (Poco::TimeoutException& timeoutEx) {
        NLOG(error) << "Timeout occurred: " << timeoutEx.displayText();
        try {
            client_.disconnect();  // 소켓 연결을 안전하게 닫음
        }
        catch (const Poco::Exception& ex) {
            NLOG(error) << "Failed to remove client from pool: " << ex.displayText();
        }
    }
    catch (Poco::Redis::RedisException& ex) {
        NLOG(error) << ex.message();
    }
    catch (Poco::Exception& ex) {
        NLOG(error) << "Other Poco exception: " << ex.displayText();
    }
    catch (std::exception& ex) {
        NLOG(error) << "Standard exception: " << ex.what();
    }
}

std::string RedisReader::hget(const std::string& hash, const std::string& key)
{
    try {
        if (client_.isConnected()) {
            if (use_auth_) {
                Poco::Redis::Array auth;
                auth.add("AUTH").add(password_);
                auto authResult = client_.execute<std::string>(auth);
                if (authResult.compare("OK") != 0) {
                    LOG_ERROR("인증 실패");
                    return "";
                }
            }
            Poco::Redis::Array command;
            command.add("HGET").add(hash).add(key);
            auto result = client_.execute<Poco::Redis::BulkString>(command);
            return result.value();
        }
    }
    catch (Poco::TimeoutException& timeoutEx) {
        NLOG(error) << "Timeout occurred: " << timeoutEx.displayText();
        try {
            client_.disconnect();  // 소켓 연결을 안전하게 닫음
        }
        catch (const Poco::Exception& ex) {
            NLOG(error) << "Failed to remove client from pool: " << ex.displayText();
        }
    }
    catch (Poco::Redis::RedisException& ex) {
        NLOG(error) << ex.message();
    }
    catch (Poco::Exception& ex) {
        NLOG(error) << "Other Poco exception: " << ex.displayText();
    }
    catch (std::exception& ex) {
        NLOG(error) << "Standard exception: " << ex.what();
    }

    return "";
}

std::vector<std::string> RedisReader::getKeys(const std::string& pattern)
{
    try {
        if (client_.isConnected()) {
            if (use_auth_) {
                Poco::Redis::Array auth;
                auth.add("AUTH").add(password_);
                auto authResult = client_.execute<std::string>(auth);
                if (authResult.compare("OK") != 0) {
                    LOG_ERROR("인증 실패");
                    return {};
                }
            }
            Poco::Redis::Array command;
            command.add("HKEYS").add(pattern);
            auto result = client_.execute<Poco::Redis::Array>(command);
            std::vector<std::string> keys;
            for (size_t index = 0; index < result.size(); index++) {
                keys.push_back(result.get<BulkString>(index).value());
            }
            return keys;
        }
    }
    catch (Poco::TimeoutException& timeoutEx) {
        NLOG(error) << "Timeout occurred: " << timeoutEx.displayText();
        try {
            client_.disconnect();  // 소켓 연결을 안전하게 닫음
        }
        catch (const Poco::Exception& ex) {
            NLOG(error) << "Failed to remove client from pool: " << ex.displayText();
        }
    }
    catch (Poco::Redis::RedisException& ex) {
        NLOG(error) << ex.message();
    }
    catch (Poco::Exception& ex) {
        NLOG(error) << "Other Poco exception: " << ex.displayText();
    }
    catch (std::exception& ex) {
        NLOG(error) << "Standard exception: " << ex.what();
    }

    return {};
}

std::vector<std::string> RedisReader::hgetall(const std::string& hash)
{
    try {
        if (client_.isConnected()) {
            if (use_auth_) {
                Poco::Redis::Array auth;
                auth.add("AUTH").add(password_);
                auto authResult = client_.execute<std::string>(auth);

                if (authResult.compare("OK") != 0) {
                    LOG_ERROR("인증 실패");
                    return {};
                }
            }

            Poco::Redis::Array command;
            command.add("HGETALL").add(hash);

            auto result = client_.execute<Poco::Redis::Array>(command);
            std::vector<std::string> keys;

            for (size_t index = 0; index < result.size(); index++) {
                keys.push_back(result.get<BulkString>(index).value());
            }
            return keys;
        }
    }
    catch (Poco::TimeoutException& timeoutEx) {
        NLOG(error) << "Timeout occurred: " << timeoutEx.displayText();
        try {
            client_.disconnect();  // 소켓 연결을 안전하게 닫음
        }
        catch (const Poco::Exception& ex) {
            NLOG(error) << "Failed to remove client from pool: " << ex.displayText();
        }
    }
    catch (Poco::Redis::RedisException& ex) {
        NLOG(error) << ex.message();
    }
    catch (Poco::Exception& ex) {
        NLOG(error) << "Other Poco exception: " << ex.displayText();
    }
    catch (std::exception& ex) {
        NLOG(error) << "Standard exception: " << ex.what();
    }

    return {};
}

void RedisReader::hdel(const std::string& hash, const std::string& key)
{
    try {
        if (client_.isConnected()) {
            if (use_auth_) {
                Poco::Redis::Array auth;
                auth.add("AUTH").add(password_);
                auto authResult = client_.execute<std::string>(auth);
                if (authResult.compare("OK") != 0) {
                    LOG_ERROR("인증 실패");
                    return;
                }
            }
            Poco::Redis::Array command;
            command.add("HDEL").add(hash).add(key);
            client_.sendCommand(command);
        }
    }
    catch (Poco::TimeoutException& timeoutEx) {
        NLOG(error) << "Timeout occurred: " << timeoutEx.displayText();
        try {
            client_.disconnect();  // 소켓 연결을 안전하게 닫음
        }
        catch (const Poco::Exception& ex) {
            NLOG(error) << "Failed to remove client from pool: " << ex.displayText();
        }
    }
    catch (Poco::Redis::RedisException& ex) {
        NLOG(error) << ex.message();
    }
    catch (Poco::Exception& ex) {
        NLOG(error) << "Other Poco exception: " << ex.displayText();
    }
    catch (std::exception& ex) {
        NLOG(error) << "Standard exception: " << ex.what();
    }
}