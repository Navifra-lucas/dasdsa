#include "core_agent/manager/message_handler_manager.h"

#include <Poco/JSON/Object.h>
#include <gtest/gtest.h>

#include <chrono>
#include <iostream>
#include <memory>
#include <sstream>
#include <thread>

using namespace std;
using namespace Poco::JSON;
using namespace NaviFra;

class Dependency {
public:
    void performTask() const {}
};

class TestMessageHandler : public IMessageHandler {
public:
    TestMessageHandler(shared_ptr<Dependency> dependency)
        : dependency_(dependency)
    {
    }

    void handleMessage(Object::Ptr message) override
    {
        stringstream ss;
        message->stringify(ss);
        dependency_->performTask();
    }

private:
    shared_ptr<Dependency> dependency_;
};

TEST(MessageHandlerManagerTest, RegisterAndHandleMessage)
{
    auto& manager = MessageHandlerManager::instance();
    auto dependency = make_shared<Dependency>();
    auto handler = make_shared<TestMessageHandler>(dependency);

    manager.registerHandler("test", handler);

    ASSERT_TRUE(manager.has("test"));

    Object::Ptr message = new Object;
    message->set("key", "value");

    testing::internal::CaptureStdout();
    manager.handleMessage("test", message);
    this_thread::sleep_for(chrono::milliseconds(100));  // Give some time for async processing
    string output = testing::internal::GetCapturedStdout();

    ASSERT_EQ(output, "TestMessageHandler: {\"key\":\"value\"}\nDependency task performed.\n");
}

TEST(MessageHandlerManagerTest, HandleMessageWithoutHandler)
{
    auto& manager = MessageHandlerManager::instance();

    Object::Ptr message = new Object;
    message->set("key", "value");

    ASSERT_THROW(manager.handleMessage("nonexistent", message), runtime_error);
}

TEST(MessageHandlerManagerTest, ConcurrentHandling)
{
    auto& manager = MessageHandlerManager::instance();
    auto dependency = make_shared<Dependency>();
    auto handler = make_shared<TestMessageHandler>(dependency);

    manager.registerHandler("test", handler);

    Object::Ptr message1 = new Object;
    message1->set("key", "value1");
    Object::Ptr message2 = new Object;
    message2->set("key", "value2");

    testing::internal::CaptureStdout();

    thread t1([&manager, message1]() { manager.handleMessage("test", message1); });
    thread t2([&manager, message2]() { manager.handleMessage("test", message2); });

    t1.join();
    t2.join();

    this_thread::sleep_for(chrono::milliseconds(200));  // Give some time for async processing
    string output = testing::internal::GetCapturedStdout();

    ASSERT_TRUE(output.find("TestMessageHandler: {\"key\":\"value1\"}") != string::npos);
    ASSERT_TRUE(output.find("TestMessageHandler: {\"key\":\"value2\"}") != string::npos);
    ASSERT_TRUE(output.find("Dependency task performed.") != string::npos);
}
