#include "core_agent/data/memory_repository.h"
#include "core_agent/data/robot_status.h"
#include "core_agent/manager/alarm_manager.h"
#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include "mocks/mock_redis_publisher.h"

#include <Poco/JSON/Object.h>
#include <Poco/UUIDGenerator.h>

#include <thread>

using namespace NaviFra;
using ::testing::_;
using ::testing::Return;

class AlarmManagerTest : public ::testing::Test {
protected:
    void SetUp() override
    {
        // MockRedisPublisher 인스턴스를 AlarmManager에 설정합니다.
        mockPublisher = std::make_shared<MockRedisPublisher>();
        alarmManager.setCommander(mockPublisher);
        InMemoryRepository::instance().get<RobotStatus>(RobotStatus::KEY)->setID("test");
        // alarmManager.testLoadState();

        // ON_CALL(*mockPublisher, publish(_, _)).WillByDefault(testing::Invoke([](const std::string& channel, const std::string& message) {
        //    NLOG(info) << "Publish called with channel: " << channel << ", message: " << message;
        //}));
    }

    AlarmManager& alarmManager = AlarmManager::instance();
    std::shared_ptr<MockRedisPublisher> mockPublisher;
};

TEST_F(AlarmManagerTest, SetAndGetAlarm)
{
    uint32_t alarmId = 1;
    std::string description = "Test alarm";

    // EXPECT_CALL(*mockPublisher, publish(_, _)).Times(2);
    EXPECT_CALL(*mockPublisher, hset(_, _, _)).Times(1);
    EXPECT_CALL(*mockPublisher, hdel(_, _)).Times(1);
    EXPECT_CALL(*mockPublisher, getKeys(_)).WillRepeatedly(Return(std::vector<std::string>{}));

    // 알람을 설정합니다.
    alarmManager.setAlarm(alarmId, description);

    // 알람을 가져옵니다.
    Poco::JSON::Object::Ptr allAlarms = alarmManager.getAllAlarms();
    ASSERT_EQ(allAlarms->size(), 1);

    Poco::JSON::Object::Ptr alarm = allAlarms->getObject(allAlarms->begin()->first);
    ASSERT_EQ(alarm->getValue<uint32_t>("alarmId"), alarmId);
    ASSERT_EQ(alarm->getValue<std::string>("description"), description);
    ASSERT_EQ(alarm->getValue<std::string>("status"), "active");

    alarmManager.clearAllAlarms();
}

TEST_F(AlarmManagerTest, DuplicateAlarmId)
{
    uint32_t alarmId = 1;
    std::string description1 = "Test alarm 1";
    std::string description2 = "Test alarm 2";

    // EXPECT_CALL(*mockPublisher, publish(_, _)).Times(2);
    EXPECT_CALL(*mockPublisher, hset(_, _, _)).Times(1);
    EXPECT_CALL(*mockPublisher, hdel(_, _)).Times(1);
    EXPECT_CALL(*mockPublisher, getKeys(_)).WillRepeatedly(Return(std::vector<std::string>{}));

    // 첫 번째 알람을 설정합니다.
    alarmManager.setAlarm(alarmId, description1);

    // 두 번째 알람을 설정하려고 하지만 같은 AlarmID로 인해 설정되지 않아야 합니다.
    alarmManager.setAlarm(alarmId, description2);

    // 모든 알람을 가져옵니다.
    Poco::JSON::Object::Ptr allAlarms = alarmManager.getAllAlarms();
    ASSERT_EQ(allAlarms->size(), 1);

    Poco::JSON::Object::Ptr alarm = allAlarms->getObject(allAlarms->begin()->first);
    ASSERT_EQ(alarm->getValue<uint32_t>("alarmId"), alarmId);
    ASSERT_EQ(alarm->getValue<std::string>("description"), description1);
    ASSERT_EQ(alarm->getValue<std::string>("status"), "active");

    alarmManager.clearAllAlarms();
}

TEST_F(AlarmManagerTest, ClearAlarm)
{
    uint32_t alarmId = 2;
    std::string description = "Test alarm to clear";

    // EXPECT_CALL(*mockPublisher, publish(_, _)).Times(2);
    EXPECT_CALL(*mockPublisher, hset(_, _, _)).Times(1);
    EXPECT_CALL(*mockPublisher, hdel(_, _)).Times(1);
    EXPECT_CALL(*mockPublisher, getKeys(_)).WillRepeatedly(Return(std::vector<std::string>{}));

    // 알람을 설정합니다.
    alarmManager.setAlarm(alarmId, description);

    // 알람 UUID를 가져옵니다.
    Poco::JSON::Object::Ptr allAlarms = alarmManager.getAllAlarms();
    ASSERT_EQ(allAlarms->size(), 1);
    std::string uuid = allAlarms->begin()->first;

    // 알람을 해제합니다.
    alarmManager.clearAlarm(uuid);

    allAlarms = alarmManager.getAllAlarms();
    ASSERT_EQ(allAlarms->size(), 0);
}

TEST_F(AlarmManagerTest, ClearAllAlarms)
{
    uint32_t alarmId1 = 3;
    uint32_t alarmId2 = 4;
    std::string description1 = "Test alarm 1";
    std::string description2 = "Test alarm 2";

    // EXPECT_CALL(*mockPublisher, publish(_, _)).Times(4);
    EXPECT_CALL(*mockPublisher, hset(_, _, _)).Times(2);
    EXPECT_CALL(*mockPublisher, hdel(_, _)).Times(2);
    EXPECT_CALL(*mockPublisher, getKeys(_)).WillRepeatedly(Return(std::vector<std::string>{}));

    // 알람을 설정합니다.
    alarmManager.setAlarm(alarmId1, description1);
    alarmManager.setAlarm(alarmId2, description2);

    // 모든 알람을 해제합니다.
    alarmManager.clearAllAlarms();

    // 모든 알람을 가져옵니다.
    Poco::JSON::Object::Ptr allAlarms = alarmManager.getAllAlarms();
    ASSERT_EQ(allAlarms->size(), 0);
}

TEST_F(AlarmManagerTest, LoadState)
{
    uint32_t alarmId1 = 5;
    std::string description1 = "Loaded alarm 1";

    Poco::UUID uuid = Poco::UUIDGenerator::defaultGenerator().create();
    std::string uuidStr = uuid.toString();

    std::time_t now = std::time(nullptr);
    std::string timestamp = std::ctime(&now);
    timestamp.pop_back();  // Remove newline character

    Poco::JSON::Object::Ptr alarm = new Poco::JSON::Object;
    alarm->set("uuid", uuidStr);
    alarm->set("timestamp", timestamp);
    alarm->set("alarmId", alarmId1);
    alarm->set("description", description1);
    alarm->set("status", "active");

    std::ostringstream oss;
    alarm->stringify(oss);

    std::vector<std::string> keys = {"alarm:test:" + uuidStr};
    // EXPECT_CALL(*mockPublisher, publish(_, _)).Times(1);
    EXPECT_CALL(*mockPublisher, hdel(_, _)).Times(1);
    EXPECT_CALL(*mockPublisher, getKeys(_)).WillRepeatedly(Return(keys));
    EXPECT_CALL(*mockPublisher, hget(_, keys[0])).WillRepeatedly(Return(oss.str()));

    // Load state
    alarmManager.loadState();

    // 모든 알람을 가져옵니다.
    Poco::JSON::Object::Ptr allAlarms = alarmManager.getAllAlarms();
    ASSERT_EQ(allAlarms->size(), 1);

    Poco::JSON::Object::Ptr loadedAlarm = allAlarms->getObject(allAlarms->begin()->first);
    ASSERT_EQ(loadedAlarm->getValue<uint32_t>("alarmId"), alarmId1);
    ASSERT_EQ(loadedAlarm->getValue<std::string>("description"), description1);
    ASSERT_EQ(loadedAlarm->getValue<std::string>("status"), "active");

    alarmManager.clearAllAlarms();
}

TEST_F(AlarmManagerTest, ConcurrentSetAndClearAlarm)
{
    const int numThreads = 10;
    const uint32_t alarmId = 7;

    // EXPECT_CALL(*mockPublisher, publish(_, _)).WillRepeatedly(testing::Invoke([](const std::string& channel, const std::string& message)
    // {
    //   NLOG(info) << "Publish called with channel: " << channel << ", message: " << message;
    //}));
    EXPECT_CALL(*mockPublisher, hset(_, _, _)).WillRepeatedly(Return());
    EXPECT_CALL(*mockPublisher, hdel(_, _)).WillRepeatedly(Return());
    EXPECT_CALL(*mockPublisher, getKeys(_)).WillRepeatedly(Return(std::vector<std::string>{}));

    std::vector<std::thread> threads;

    // 스레드를 생성하여 알람을 해제합니다.
    for (int i = 0; i < numThreads; ++i) {
        threads.emplace_back([this]() {
            // 알람을 설정합니다.
            std::string uuid = alarmManager.setAlarm(alarmId, "Concurrent alarm");
            // Poco::JSON::Object::Ptr allAlarms = alarmManager.getAllAlarms();
            // if (allAlarms->size() > 0) {
            //     std::string uuid = allAlarms->begin()->first;

            alarmManager.clearAlarm(uuid);
            // }
        });
    }

    // 모든 스레드를 종료합니다.
    for (auto& thread : threads) {
        thread.join();
    }

    // 모든 알람을 가져옵니다.
    Poco::JSON::Object::Ptr finalAlarms = alarmManager.getAllAlarms();
    ASSERT_EQ(finalAlarms->size(), 0);
}

TEST_F(AlarmManagerTest, ConcurrentClearAlarm)
{
    const int numThreads = 10;
    const uint32_t alarmId = 7;
    const std::string description = "Concurrent clear alarm";

    // EXPECT_CALL(*mockPublisher, publish(_, _)).WillRepeatedly(testing::Invoke([](const std::string& channel, const std::string& message)
    // {
    //    NLOG(info) << "Publish called with channel: " << channel << ", message: " << message;
    //}));
    EXPECT_CALL(*mockPublisher, hset(_, _, _)).WillRepeatedly(Return());
    EXPECT_CALL(*mockPublisher, hdel(_, _)).WillRepeatedly(Return());
    EXPECT_CALL(*mockPublisher, getKeys(_)).WillRepeatedly(Return(std::vector<std::string>{}));

    // 알람을 설정합니다.
    alarmManager.setAlarm(alarmId, description);

    std::vector<std::thread> threads;

    // 스레드를 생성하여 알람을 해제합니다.
    for (int i = 0; i < numThreads; ++i) {
        threads.emplace_back([this]() {
            Poco::JSON::Object::Ptr allAlarms = alarmManager.getAllAlarms();
            if (allAlarms->size() > 0) {
                std::string uuid = allAlarms->begin()->first;
                alarmManager.clearAlarm(uuid);
            }
        });
    }

    // 모든 스레드를 종료합니다.
    for (auto& thread : threads) {
        thread.join();
    }

    // 모든 알람을 가져옵니다.
    Poco::JSON::Object::Ptr finalAlarms = alarmManager.getAllAlarms();
    ASSERT_EQ(finalAlarms->size(), 0);
}