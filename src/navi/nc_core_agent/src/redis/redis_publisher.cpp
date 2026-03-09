#include "core_agent/core_agent.h"

#include <boost/algorithm/string.hpp>
#include <core_agent/redis/redis_poolable_connection_factory.h>
#include <core_agent/redis/redis_publisher.h>

using namespace Poco::Redis;
using namespace Poco::JSON;
using namespace NaviFra;

RedisPublisher::RedisPublisher()
{
    hostname_ = Config::instance().getString("redis_host", "127.0.0.1");
    port_ = Config::instance().getInt("redis_port", 6379);
    password_ = Config::instance().getString("redis_passwd", "navifra1@3$");
    use_auth_ = Config::instance().getBool("use_auth", true);
    poolCapacity_ = 10;
    poolPeakCapacity_ = 15;
}

RedisPublisher::~RedisPublisher()
{
}

bool RedisPublisher::initialize()
{
    try {
        Poco::Net::SocketAddress sa(hostname_, port_);

        connectionFactory_ = std::make_unique<Poco::RedisPoolableObjectFactory>(sa);
        connectionPool_ = std::make_unique<RedisConnectionPool>(*connectionFactory_, poolCapacity_, poolPeakCapacity_);
        LOG_INFO("RedisPublisher initialize success");
        return true;
    }
    catch (Poco::Redis::RedisException ex) {
        NLOG(error) << ex.message();
    }

    return false;
}

bool RedisPublisher::isConnected()
{
    try {
        Poco::RedisPooledConnection client(*connectionPool_);
        if (!((Client::Ptr)client)->isConnected())
            return false;
        return true;
    }
    catch (Poco::Redis::RedisException ex) {
        NLOG(error) << ex.message();
    }
    return false;
}

bool RedisPublisher::connect()
{
    return true;
}

void RedisPublisher::disconnect()
{
}

void RedisPublisher::reconnect()
{
}

void RedisPublisher::publish(const std::string& channel, const std::string& message)
{
    Poco::RedisPooledConnection client(*connectionPool_);
    try {
        std::vector<Poco::Redis::Array> commands;

        if (((Client::Ptr)client)->isConnected()) {
            if (use_auth_) {
                Poco::Redis::Array auth;
                auth.add("AUTH").add(Poco::format("%s", password_));
                commands.push_back(auth);

                Poco::Redis::Array command;
                command.add("PUBLISH").add(channel).add(message);
                commands.push_back(command);
                auto result = ((Client::Ptr)client)->sendCommands(commands);
                if (result.getType(0) == RedisType::Types::REDIS_ERROR) {
                    std::string strerror = result.toString();
                    strerror = strerror.substr(2);
                    boost::replace_all(strerror, "\r\n-", " ");
                    NLOG(error) << strerror;
                }
            }
            else {
                Poco::Redis::Array command;
                command.add("PUBLISH").add(channel).add(message);
                Poco::Int64 subscribers = ((Client::Ptr)client)->execute<Poco::Int64>(command);
            }
        }
    }
    catch (Poco::TimeoutException& timeoutEx) {
        NLOG(error) << "Timeout occurred: " << timeoutEx.displayText();
        try {
            ((Client::Ptr)client)->disconnect();  // 소켓 연결을 안전하게 닫음
            connectionPool_->returnObject((Client::Ptr)client);
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

void RedisPublisher::hset(const std::string& hash, const std::string& key, const std::string& value)
{
    Poco::RedisPooledConnection client(*connectionPool_);
    try {
        if (((Client::Ptr)client)->isConnected()) {
            if (use_auth_) {
                Poco::Redis::Array auth;
                auth.add("AUTH").add(password_);
                auto authResult = ((Poco::Redis::Client::Ptr)client)->execute<std::string>(auth);

                if (authResult.compare("OK") != 0) {
                    LOG_ERROR("인증 실패");
                    return;
                }
            }

            std::vector<Poco::Redis::Array> commands;
            Poco::Redis::Array command;
            command.add("HSET").add(hash).add(key).add(value);
            commands.push_back(command);

            ((Client::Ptr)client)->sendCommands(commands);
        }
    }
    catch (Poco::TimeoutException& timeoutEx) {
        NLOG(error) << "Timeout occurred: " << timeoutEx.displayText();
        try {
            ((Client::Ptr)client)->disconnect();  // 소켓 연결을 안전하게 닫음
            connectionPool_->returnObject((Client::Ptr)client);
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

std::string RedisPublisher::hget(const std::string& hash, const std::string& key)
{
    Poco::RedisPooledConnection client(*connectionPool_);
    try {
        if (((Client::Ptr)client)->isConnected()) {
            if (use_auth_) {
                Poco::Redis::Array auth;
                auth.add("AUTH").add(password_);
                auto authResult = ((Poco::Redis::Client::Ptr)client)->execute<std::string>(auth);
                if (authResult.compare("OK") != 0) {
                    LOG_ERROR("인증 실패");
                    return "";
                }
            }
            Poco::Redis::Array command;
            command.add("HGET").add(hash).add(key);
            auto result = ((Client::Ptr)client)->execute<Poco::Redis::BulkString>(command);
            return result.value();
        }
    }
    catch (Poco::TimeoutException& timeoutEx) {
        NLOG(error) << "Timeout occurred: " << timeoutEx.displayText();
        try {
            ((Client::Ptr)client)->disconnect();  // 소켓 연결을 안전하게 닫음
            connectionPool_->returnObject((Client::Ptr)client);
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

std::vector<std::string> RedisPublisher::getKeys(const std::string& pattern)
{
    Poco::RedisPooledConnection client(*connectionPool_);
    try {
        if (((Client::Ptr)client)->isConnected()) {
            if (use_auth_) {
                Poco::Redis::Array auth;
                auth.add("AUTH").add(password_);
                auto authResult = ((Poco::Redis::Client::Ptr)client)->execute<std::string>(auth);
                if (authResult.compare("OK") != 0) {
                    LOG_ERROR("인증 실패");
                    return {};
                }
            }
            Poco::Redis::Array command;
            command.add("HKEYS").add(pattern);
            auto result = ((Client::Ptr)client)->execute<Poco::Redis::Array>(command);
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
            ((Client::Ptr)client)->disconnect();  // 소켓 연결을 안전하게 닫음
            connectionPool_->returnObject((Client::Ptr)client);
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

std::vector<std::string> RedisPublisher::hgetall(const std::string& hash)
{
    Poco::RedisPooledConnection client(*connectionPool_);
    try {
        if (((Client::Ptr)client)->isConnected()) {
            if (use_auth_) {
                Poco::Redis::Array auth;
                auth.add("AUTH").add(password_);
                auto authResult = ((Poco::Redis::Client::Ptr)client)->execute<std::string>(auth);

                if (authResult.compare("OK") != 0) {
                    LOG_ERROR("인증 실패");
                    return {};
                }
            }

            Poco::Redis::Array command;
            command.add("HGETALL").add(hash);

            auto result = ((Client::Ptr)client)->execute<Poco::Redis::Array>(command);
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
            ((Client::Ptr)client)->disconnect();  // 소켓 연결을 안전하게 닫음
            connectionPool_->returnObject((Client::Ptr)client);
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

void RedisPublisher::hdel(const std::string& hash, const std::string& key)
{
    Poco::RedisPooledConnection client(*connectionPool_);
    try {
        if (((Client::Ptr)client)->isConnected()) {
            if (use_auth_) {
                Poco::Redis::Array auth;
                auth.add("AUTH").add(password_);
                auto authResult = ((Poco::Redis::Client::Ptr)client)->execute<std::string>(auth);
                if (authResult.compare("OK") != 0) {
                    LOG_ERROR("인증 실패");
                    return;
                }
            }
            Poco::Redis::Array command;
            command.add("HDEL").add(hash).add(key);
            ((Client::Ptr)client)->sendCommand(command);
        }
    }
    catch (Poco::TimeoutException& timeoutEx) {
        NLOG(error) << "Timeout occurred: " << timeoutEx.displayText();
        try {
            ((Client::Ptr)client)->disconnect();  // 소켓 연결을 안전하게 닫음
            connectionPool_->returnObject((Client::Ptr)client);
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