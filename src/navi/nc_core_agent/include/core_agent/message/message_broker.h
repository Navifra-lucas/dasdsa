#ifndef NAVIFRA_MESSAGE_BROKER_H
#define NAVIFRA_MESSAGE_BROKER_H

#include <core_agent/manager/action_manager.h>
#include <core_agent/message/message_publisher.h>
#include <core_agent/message/message_subscriber.h>

#include <iostream>
#include <memory>
#include <unordered_map>

namespace NaviFra {
class MessageBroker {
public:
    static MessageBroker& instance()
    {
        static Poco::SingletonHolder<MessageBroker> sh;
        return *sh.get();
    }

    // Publisher 추가 (Redis/MQTT 추가 가능)
    void addPublisher(const std::string& source, MessagePublisher::Ptr publisher) { publishers[source] = publisher; }

    // Subscriber 추가 (Redis/MQTT 추가 가능)
    void addSubscriber(const std::string& source, MessageSubscriber::Ptr subscriber) { subscribers[source] = subscriber; }

    MessagePublisher::Ptr getPublisher(std::string source)
    {
        if (publishers.find(source) != publishers.end()) {  // Publisher가 있는 경우 실행
            return publishers[source];
        }
        else {
            return nullptr;
        }
    }

    // 메시지 수신 시 처리 (각 `source`별로 개별 처리)
    void receiveMessage(const std::string& source, Poco::JSON::Object::Ptr obj) { ActionManager::instance().onAction(source, obj); }

    void publish(const std::string& channel, const std::string& message)
    {
        // Poco::FastMutex::ScopedLock lock(fastMutex_);
        for (auto& pub : publishers) {
            pub.second->publish(channel, message);
        }
    }

    // 메시지 전송 (source에 맞는 Publisher로 전송)
    void publish(const std::string& source, const std::string& channel, const std::string& message)
    {
        if (publishers.find(source) != publishers.end()) {  // Publisher가 있는 경우 실행
            publishers[source]->publish(channel, message);
        }
        else {
            throw "[MessageBroker] No publisher found for source: ";
        }
    }

private:
    std::unordered_map<std::string, NaviFra::MessagePublisher::Ptr> publishers;
    std::unordered_map<std::string, NaviFra::MessageSubscriber::Ptr> subscribers;  // 여러 Subscriber 관리
};
}  // namespace NaviFra

#endif  // MESSAGE_BROKER_H
