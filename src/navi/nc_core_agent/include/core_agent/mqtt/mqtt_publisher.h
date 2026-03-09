#ifndef NAVIFRA_MQTT_PUBLISHER_H
#define NAVIFRA_MQTT_PUBLISHER_H

#include <Poco/Mutex.h>
#include <core_agent/message/message_publisher.h>
#include <core_agent/mqtt/mqtt_poolable_connection_factory.h>
#include <mqtt/async_client.h>

#include <memory>
#include <string>
#include <vector>

namespace NaviFra {

using MQTTPoolableObjectFactoryPtr = std::unique_ptr<Poco::MQTTPoolableObjectFactory>;
using MQTTConnectionPool = Poco::ObjectPool<mqtt::async_client, Poco::SharedPtr<mqtt::async_client>, Poco::MQTTPoolableObjectFactory>;
using MQTTConnectionPoolPtr = std::unique_ptr<MQTTConnectionPool>;

class MQTTPublisher : public MessagePublisher {
public:
    MQTTPublisher();
    ~MQTTPublisher();

    using Ptr = Poco::SharedPtr<MQTTPublisher>;

private:
    Poco::FastMutex _fastMutex;

    Poco::SharedPtr<mqtt::async_client> client_;
    mqtt::connect_options connOpts_;

    size_t poolCapacity_;
    size_t poolPeakCapacity_;

    std::unique_ptr<Poco::MQTTPoolableObjectFactory> connectionFactory_;
    std::unique_ptr<MQTTConnectionPool> connectionPool_;

private:
    bool connect();
    void disconnect();

public:
    virtual bool initialize() override;
    virtual bool isConnected() override;
    virtual void reconnect() override;

    virtual void publish(const std::string& topic, const std::string& message) override;
};
}  // namespace NaviFra

#endif  // NAVIFRA_MQTT_PUBLISHER_H
