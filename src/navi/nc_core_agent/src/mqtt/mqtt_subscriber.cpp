#include "core_agent/core_agent.h"

#include <core_agent/mqtt/mqtt_subscriber.h>

namespace NaviFra {

MQTTSubscriber::MQTTSubscriber()
{
    connOpts_.set_keep_alive_interval(20);
    connOpts_.set_clean_session(true);
    connOpts_.set_automatic_reconnect(true);

    std::string hostname = Config::instance().getString("mqtt_host", "localhost");
    std::string port = Config::instance().getString("mqtt_port", "1883");

    std::string serverURL = Poco::format("mqtt://%s:%s", hostname, port);

    client_.reset(new mqtt::async_client(serverURL, Poco::UUIDGenerator::defaultGenerator().create().toString()));
}

MQTTSubscriber::~MQTTSubscriber()
{
    disconnect();
}

bool MQTTSubscriber::initialize()
{
    return connect();
}

bool MQTTSubscriber::initialize(const std::string& address, const std::string& clientId)
{
    address_ = address;
    clientId_ = clientId;
    return connect();
}

bool MQTTSubscriber::connect()
{
    try {
        NLOG(info) << "Connecting to the MQTT server...";
        client_->set_callback(*this);
        client_->connect(connOpts_, nullptr, *this);
        return true;
    }
    catch (const mqtt::exception& exc) {
        NLOG(error) << "Error: " << exc.what();
        return false;
    }
}

void MQTTSubscriber::disconnect()
{
    try {
        NLOG(info) << "Disconnecting from the MQTT server...";
        client_->disconnect()->wait();
    }
    catch (const mqtt::exception& exc) {
        NLOG(error) << "Error: " << exc.what();
    }
}

void MQTTSubscriber::reconnect()
{
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    try {
        auto token = client_->reconnect();

        if (token->wait_for(std::chrono::seconds(60))) {
            // 연결 성공
            NLOG(info) << "Reconnected successfully!";
        }
        else {
            // 시간 초과 또는 실패
            NLOG(info) << "Reconnect failed or timed out!";
        }
    }
    catch (const mqtt::exception& exc) {
        NLOG(error) << "Error: " << exc.what();
        exit(1);
    }
}

// (Re)connection success
void MQTTSubscriber::connected(const std::string& cause)
{
    NLOG(info) << "Connection success";

    for (const auto& topic : topics_) {
        client_->subscribe(topic, 1, nullptr, *this);
    }
}

void MQTTSubscriber::subscribe(std::vector<std::string> topics)
{
    bool mqtt_transport = Config::instance().getBool("mqtt_transport");

    try {
        for (auto& topic : topics) {
            std::string cleaned_topic = topic;
            if (!mqtt_transport) {
                std::transform(cleaned_topic.begin(), cleaned_topic.end(), cleaned_topic.begin(), [](char c) {
                    if (c == ':' || c == '.')
                        return '/';
                    if (c == '*')
                        return '#';
                    return c;
                });
            }
            topics_.push_back(cleaned_topic);
        }
    }
    catch (const mqtt::exception& exc) {
        NLOG(error) << "Error subscribing to topic: " << exc.what();
    }
}

bool MQTTSubscriber::isConnected()
{
    return client_->is_connected();
}

void MQTTSubscriber::message_arrived(mqtt::const_message_ptr msg)
{
    // NLOG(info) << msg->get_topic() << ": " << msg->to_string();
    MessageSubscriberArgs notifyMSG(msg->get_topic(), msg->to_string(), "mqtt");
    if (notify_.empty() != true) {
        // NLOG(info) << "1 Notifying subscribers for topic: " << msg->get_topic();
        notify_.notifyAsync(this, notifyMSG);
        // NLOG(info) << "2 Notifying subscribers for topic: " << msg->get_topic();
    }
    else {
        NLOG(warning) << "No subscribers for topic: " << msg->get_topic();
    }
}

void MQTTSubscriber::connection_lost(const std::string& cause)
{
    NLOG(error) << "Connection lost. Cause: " << cause;
}

void MQTTSubscriber::on_failure(const mqtt::token& tok)
{
    // NLOG(info) << name_ << " failure";
    if (tok.get_message_id() != 0)
        NLOG(info) << " for token: [" << tok.get_message_id() << "]";
}

void MQTTSubscriber::on_success(const mqtt::token& tok)
{
    auto top = tok.get_topics();
    if (top && !top->empty())
        NLOG(info) << "psubscribe success [ " << (*top)[0] << " ]";
}

}  // namespace NaviFra
