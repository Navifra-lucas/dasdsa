#include "nc_wia_agent/initializer/mqtt_initializer.h"

#include "nc_wia_agent/data/robot_basic_status.h"

#include <Poco/Delegate.h>
#include <Poco/JSON/Object.h>
#include <Poco/JSON/Parser.h>
#include <Poco/UUIDGenerator.h>
#include <core_agent/data/memory_repository.h>
#include <core_agent/manager/initializer_manager.h>
#include <core_agent/message/message_broker.h>
#include <core_agent/mqtt/mqtt_publisher.h>
#include <core_agent/mqtt/mqtt_subscriber.h>

namespace NaviFra {
void handleMessage(const void*, MessageSubscriberArgs& args)
{
    try {
        const std::string& channel = args.channel();
        const std::string& message = args.message();

        LOG_INFO("Received message on [ %s ]: %s", channel.c_str(), message.c_str());

        Poco::JSON::Parser parser;
        Poco::Dynamic::Var result = parser.parse(message);
        Poco::JSON::Object::Ptr obj = result.extract<Poco::JSON::Object::Ptr>();
        // HPLC -> 변환
        if (channel.find("H_ACS.PLC-SC_OFFSET") != std::string::npos) {
            if (obj->has("subject")) {
                std::string subject = obj->getValue<std::string>("subject");
                LOG_INFO("PLC subject: %s", subject.c_str());

                if (subject == "PLC_DATA_STATE_ALL") {
                    Poco::JSON::Object::Ptr payload = obj->getObject("payload");
                    payload->set("action", "hplc_offset");
                    ActionManager::instance().onAction(args.source(), std::move(payload));
                    // 예시: 메모리에 저장하거나 로직 실행
                }
            }
            return;
        }

        // HPLC -> 변환
        if (channel.find("H_ACS.PLC-SC_LoadUnload") != std::string::npos) {
            if (obj->has("subject")) {
                std::string subject = obj->getValue<std::string>("subject");
                LOG_INFO("PLC subject: %s", subject.c_str());

                if (subject == "PLC_DATA_STATE_ALL") {
                    Poco::JSON::Object::Ptr payload = obj->getObject("payload");
                    payload->set("action", "hplc_scenario");
                    ActionManager::instance().onAction(args.source(), std::move(payload));
                    // 예시: 메모리에 저장하거나 로직 실행
                }
            }
            return;
        }

        // Cmd → action 변환
        if (obj->has("Cmd") && !obj->has("action")) {
            obj->set("action", obj->get("Cmd"));
        }

        // action 있는 경우만 실행
        if (obj->has("action")) {
            std::string action = obj->getValue<std::string>("action");
            LOG_INFO("Parsed action: %s", action.c_str());
            ActionManager::instance().onAction(args.source(), std::move(obj));
            return;
        }

        // 기타 처리 안 됨
        LOG_WARNING("Unhandled message: %s", channel.c_str());
    }
    catch (const Poco::Exception& ex) {
        LOG_ERROR("handleMessage error: %s", ex.displayText().c_str());
    }
    catch (const std::exception& ex) {
        LOG_ERROR("handleMessage std error: %s", ex.what());
    }
}

void MQTTInitializer::initialize()
{
    ros::NodeHandle nh;

    MessagePublisherFactory pubfactory;
    auto publisher = pubfactory.createMessagePublisher<MQTTPublisher>();

    if (publisher->initialize()) {
        LOG_INFO("MQTTPublisher connected");
        MessageBroker::instance().addPublisher("mqtt", publisher);
    }

    MessageSubscriberFactory subfactory;
    auto subscriber = subfactory.createMessageSubscriber<MQTTSubscriber>();

    std::string robotId = InMemoryRepository::instance().get<RobotBasicStatus>(RobotBasicStatus::KEY)->getRobotID();

    if (subscriber->initialize()) {
        LOG_INFO("MQTTSubscriber connected");

        subscriber->notify_ += Poco::delegate(&handleMessage);

        subscriber->subscribe({"ACS." + robotId, "ACSLD." + robotId, "H_ACS.PLC-SC_OFFSET", "H_ACS.PLC-SC_LoadUnload"});

        MessageBroker::instance().addSubscriber("mqtt", subscriber);
    }
    else {
        LOG_ERROR("MQTTInitializer: Failed to initialize subscriber");
    }

    mqtt_check_timer_ = nh.createTimer(ros::Duration(0.5), [subscriber](const ros::TimerEvent&) {
        auto robotStatus = InMemoryRepository::instance().get<RobotBasicStatus>(RobotBasicStatus::KEY);
        if (subscriber->isConnected()) {
            robotStatus->setMqttConnect(true);
        }
        else {
            robotStatus->setMqttConnect(false);
        }
    });
}  // namespace NaviFra

}  // namespace NaviFra
