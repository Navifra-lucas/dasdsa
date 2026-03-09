#ifndef NAVIFRA_MESSAGE_HANDLER_MANAGER_H
#define NAVIFRA_MESSAGE_HANDLER_MANAGER_H

#include <Poco/JSON/Object.h>
#include <Poco/SingletonHolder.h>
#include <core_agent/message/message_handler.h>

#include <future>
#include <mutex>
#include <stdexcept>
#include <string>
#include <unordered_map>

namespace NaviFra {
class MessageHandlerManager {
public:
    static MessageHandlerManager& instance()
    {
        static Poco::SingletonHolder<MessageHandlerManager> sh;
        return *sh.get();
    }

    void registerHandler(const std::string& messageType, std::shared_ptr<IMessageHandler> handler)
    {
        std::lock_guard<std::mutex> lock(handlerMutex);
        handlers[messageType] = handler;
    }

    void handleMessage(const std::string& messageType, Poco::JSON::Object::Ptr message)
    {
        auto it = handlers.find(messageType);
        if (it != handlers.end()) {
            // 비동기로 메시지를 처리
            auto handler = it->second;
            auto future = std::async(std::launch::async, [handler, message]() { handler->handleMessage(message); });
        }
        else {
            throw std::runtime_error("No handler registered for message type: " + messageType);
        }
    }

    bool has(const std::string& messageType)
    {
        auto it = handlers.find(messageType);
        if (it != handlers.end()) {
            return true;
        }

        return false;
    }

private:
    std::unordered_map<std::string, std::shared_ptr<IMessageHandler>> handlers;
    std::mutex handlerMutex;
};
}  // namespace NaviFra

#endif  // NAVIFRA_MESSAGE_HANDLER_MANAGER_H