#include "core_agent/core_agent.h"

#include <core_agent/mqtt/mqtt_poolable_connection_factory.h>
#include <core_agent/mqtt/mqtt_publisher.h>

namespace NaviFra {

MQTTPublisher::MQTTPublisher()
{
    poolCapacity_ = 5;
    poolPeakCapacity_ = 7;
}

MQTTPublisher::~MQTTPublisher()
{
    disconnect();
}

bool MQTTPublisher::connect()
{
    return true;
}

void MQTTPublisher::disconnect()
{
}

void MQTTPublisher::reconnect()
{
}

bool MQTTPublisher::initialize()
{
    std::string hostname = Config::instance().getString("mqtt_host", "localhost");
    std::string port = Config::instance().getString("mqtt_port", "1883");
    connectionFactory_ = std::make_unique<Poco::MQTTPoolableObjectFactory>(hostname, port);
    connectionPool_ = std::make_unique<MQTTConnectionPool>(*connectionFactory_, poolCapacity_, poolPeakCapacity_);

    return true;
}

bool MQTTPublisher::isConnected()
{
    Poco::MQTTPooledConnection client(*connectionPool_);
    if (!((Poco::SharedPtr<mqtt::async_client>)client)->is_connected())
        return false;

    return true;
}

void MQTTPublisher::publish(const std::string& topic, const std::string& message)
{
    try {
        std::string cleaned_topic = topic;
        bool mqtt_transport = Config::instance().getBool("mqtt_transport");
        if (!mqtt_transport) {
            std::transform(
                cleaned_topic.begin(), cleaned_topic.end(), cleaned_topic.begin(), [](char c) { return (c == ':' || c == '.') ? '/' : c; });
        }

        mqtt::message_ptr pubmsg = mqtt::make_message(cleaned_topic, message);
        pubmsg->set_qos(1);

        Poco::MQTTPooledConnection client(*connectionPool_);
        ((Poco::SharedPtr<mqtt::async_client>)client)->publish(pubmsg);
    }
    catch (const mqtt::exception& exc) {
        NLOG(error) << "Error publishing message: " << exc.what() << std::endl;
    }
}
}  // namespace NaviFra
