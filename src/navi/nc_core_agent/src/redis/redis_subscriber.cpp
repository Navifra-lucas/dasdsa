#include "core_agent/core_agent.h"

#include <Poco/Delegate.h>
#include <Poco/Event.h>
#include <Poco/Util/Application.h>
#include <core_agent/redis/redis_subscriber.h>

using namespace Poco::Redis;
using namespace Poco::JSON;

using namespace NaviFra;
RedisSubscriber::RedisSubscriber()
    : activity_(this, &RedisSubscriber::runActivity)
    , preRedisState(true)
    , curRedisState(true)
{
    hostname_ = Config::instance().getString("redis_host", "127.0.0.1");
    port_ = Config::instance().getInt("redis_port", 6379);
    password_ = Config::instance().getString("redis_passwd", "navifra1@3$");
    use_auth_ = Config::instance().getBool("use_auth", true);
}

RedisSubscriber::~RedisSubscriber()
{
    disconnect();
}

bool RedisSubscriber::initialize(const char* hostname, int port, const char* password)
{
    hostname_ = hostname, port_ = port, password_ = password;
    return initialize();
}

bool RedisSubscriber::initialize()
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
            redisResponse_ += Poco::delegate(this, &RedisSubscriber::onRedisReaderEvent);
            redisException_ += Poco::delegate(this, &RedisSubscriber::onRedisReaderErrorEvent);
            LOG_INFO("RedisSubscriber initialize success");
            return true;
        }
    }
    catch (const Poco::Redis::RedisException& e) {
        LOG_ERROR("%s", e.displayText().c_str());
    }

    return false;
}

bool RedisSubscriber::isConnected()
{
    if (!client_.isConnected())
        return false;

    if (activity_.isRunning()) {
        static Poco::Timestamp preStateTime;
        if (curRedisState != preRedisState) {
            preStateTime.update();
        }

        Poco::Redis::Array command;
        command.add("PING");
        client_.execute<void>(command);
        client_.flush();

        Poco::Timestamp now;
        if ((now - preStateTime) / 1000 > 3000) {
            preStateTime.update();
            return false;
        }
        else {
            return true;
        }
    }
    else {
        Poco::Redis::Array command;
        command.add("PING");
        std::string reply = client_.execute<std::string>(command);
        client_.flush();
        if (reply.compare("PONG") == 0 || reply.compare("pong") == 0)
            return true;
        else
            return false;
    }
}

bool RedisSubscriber::connect()
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
            LOG_INFO("%s", e.what());
            return false;
        }
    }

    return true;
}

void RedisSubscriber::disconnect()
{
    waitpong_.set();
    if (client_.isConnected()) {
        Poco::Redis::Array unsubscribe;
        unsubscribe.add("UNSUBSCRIBE");
        client_.execute<void>(unsubscribe);
        client_.flush();
    }

    stop();
    client_.disconnect();
    redisResponse_ -= Poco::delegate(this, &RedisSubscriber::onRedisReaderEvent);
    redisException_ -= Poco::delegate(this, &RedisSubscriber::onRedisReaderErrorEvent);
}

void RedisSubscriber::reconnect()
{
    disconnect();
    if (initialize()) {
        if (patternChannels_.size() > 0 && client_.isConnected()) {
            Poco::Redis::Array subscribe;
            subscribe.add("PSUBSCRIBE").add(patternChannels_);
            client_.execute<void>(subscribe);
            client_.flush();
        }

        if (channels_.size() > 0 && client_.isConnected()) {
            Poco::Redis::Array subscribe;
            subscribe.add("PSUBSCRIBE").add(channels_);
            client_.execute<void>(subscribe);
            client_.flush();
        }
        start();
    }
}

void RedisSubscriber::runActivity()
{
    while (!activity_.isStopped()) {
        try {
            RedisType::Ptr reply = client_.readReply();

            RedisEventArgs args(reply);
            redisResponse_.notify(this, args);

            if (args.isStopped())
                stop();
        }
        catch (Poco::Exception& e) {
            RedisEventArgs args(&e);
            redisException_.notify(this, args);
            stop();
        }
        if (!activity_.isStopped())
            Poco::Thread::sleep(1);
    }
}

void RedisSubscriber::updateState()
{
    preRedisState = curRedisState;
    curRedisState = !curRedisState;
}

void RedisSubscriber::onRedisReaderEvent(const void* pSender, RedisEventArgs& args)
{
    updateState();
    try {
        if (!args.message().isNull() && args.message()->isArray()) {
            Type<Poco::Redis::Array>* arrayType = dynamic_cast<Type<Poco::Redis::Array>*>(args.message().get());
            Poco::Redis::Array& array = arrayType->value();

            BulkString type = array.get<BulkString>(0);
            BulkString channel = array.get<BulkString>(1);
            if (type.value().compare("pmessage") == 0) {
                BulkString messages = array.get<BulkString>(3);
                MessageSubscriberArgs msg(channel.value(), messages.value());
                notify_.notify(this, msg);
            }
            else if (type.value().compare("message") == 0) {
                // TODO
            }
            else if (type.value().compare("psubscribe") == 0) {
                LOG_INFO("psubscribe success chanel [ %s ]", channel.value().c_str());
            }
            else if (type.value().compare("subscribe") == 0) {
                // TODO
            }
            else if (type.value().compare("pong") == 0 || type.value().compare("PONG") == 0) {
                // LOG_INFO("psubscribe success chanel [%s]", type.value().c_str());
            }
        }
        else {
            // LOG_INFO("%s", args.message()->toString().c_str());
        }
    }
    catch (Poco::ReadFileException ex) {
        LOG_ERROR("%s", ex.what());
    }
}

void RedisSubscriber::onRedisReaderErrorEvent(const void* pSender, RedisEventArgs& args)
{
    LOG_ERROR("%s", args.exception()->displayText().c_str());
}

void RedisSubscriber::pSubscribe(std::vector<std::string> channels)
{
    Poco::FastMutex::ScopedLock lock(_fastMutex);
    patternChannels_.swap(channels);
    if (patternChannels_.size() > 0) {
        Poco::Redis::Array subscribe;
        subscribe.add("PSUBSCRIBE").add(patternChannels_);
        client_.execute<void>(subscribe);
        client_.flush();
        start();
    }
}

void RedisSubscriber::subscribe(std::vector<std::string> channel)
{
    Poco::FastMutex::ScopedLock lock(_fastMutex);
    patternChannels_.swap(channel);
    if (patternChannels_.size() > 0) {
        Poco::Redis::Array subscribe;
        subscribe.add("PSUBSCRIBE").add(patternChannels_);
        client_.execute<void>(subscribe);
        client_.flush();
        start();
    }
}