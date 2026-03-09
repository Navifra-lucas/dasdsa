#ifndef NAVIFRA_MQTT_SUBSCRIBER_H
#define NAVIFRA_MQTT_SUBSCRIBER_H

#include <Poco/Activity.h>
#include <core_agent/message/message_subscriber.h>
#include <mqtt/async_client.h>

#include <string>
#include <vector>

namespace NaviFra {
class MQTTSubscriber
    : public MessageSubscriber
    , public virtual mqtt::callback
    , public virtual mqtt::iaction_listener {
public:
    MQTTSubscriber();
    ~MQTTSubscriber();

    using Ptr = std::shared_ptr<MQTTSubscriber>;

public:
    virtual void subscribe(std::vector<std::string> topics) override;
    virtual bool initialize() override;
    virtual bool isConnected() override;
    virtual void reconnect() override;

public:
    bool initialize(const std::string& address, const std::string& clientId);
    void disconnect();
    void pSubscribe(std::vector<std::string> topics);

    // Callback for when a message arrives.
    void message_arrived(mqtt::const_message_ptr msg) override;

    void delivery_complete(mqtt::delivery_token_ptr token) override {}

    // Re-connection failure
    void on_failure(const mqtt::token& tok) override;
    // (Re)connection success
    // Either this or connected() can be used for callbacks.
    void on_success(const mqtt::token& tok) override;

    // (Re)connection success
    void connected(const std::string& cause) override;
    // Callback for when the connection is lost.
    // This will initiate the attempt to manually reconnect.
    void connection_lost(const std::string& cause) override;

    void setClient(std::shared_ptr<mqtt::async_client> client) { client_ = client; }

protected:
    bool connect();

private:
    std::string address_;
    std::string clientId_;

    std::shared_ptr<mqtt::async_client> client_;
    mqtt::connect_options connOpts_;

    std::vector<std::string> topics_;
    Poco::FastMutex _fastMutex;
};

}  // namespace NaviFra

#endif  // NAVIFRA_MQTT_SUBSCRIBER_H
