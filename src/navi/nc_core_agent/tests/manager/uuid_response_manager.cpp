#include "core_agent/manager/uuid_response_manager.h"
#include "gtest/gtest.h"
#include <chrono>
#include <thread>
#include <vector>

namespace NaviFra {
class UUIDResponseManagerTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Setup code here if needed
    }

    void TearDown() override {
        // Cleanup code here if needed
    }
};

TEST_F(UUIDResponseManagerTest, RegisterAndValidateUUID) {
    UUIDResponseManager& manager = UUIDResponseManager::instance();
    std::string uuid = "123e4567-e89b-12d3-a456-426614174000";
    manager.registerUUID(uuid, 5); // 5 seconds timeout

    // Simulate response
    std::this_thread::sleep_for(std::chrono::seconds(2));
    bool isValid = manager.validateUUID(uuid);

    EXPECT_TRUE(isValid) << "UUID should be valid within timeout period.";
}

TEST_F(UUIDResponseManagerTest, TimeoutUUID) {
    UUIDResponseManager& manager = UUIDResponseManager::instance();
    std::string uuid = "123e4567-e89b-12d3-a456-426614174001";
    manager.registerUUID(uuid, 2); // 2 seconds timeout

    // Wait for timeout
    std::this_thread::sleep_for(std::chrono::seconds(3));
    bool isValid = manager.validateUUID(uuid);

    EXPECT_FALSE(isValid) << "UUID should be invalid after timeout.";
}

TEST_F(UUIDResponseManagerTest, ValidateMultipleUUIDs) {
    UUIDResponseManager& manager = UUIDResponseManager::instance();
    std::vector<std::string> uuids = {
        "123e4567-e89b-12d3-a456-426614174002",
        "123e4567-e89b-12d3-a456-426614174003",
        "123e4567-e89b-12d3-a456-426614174004"
    };

    for (const auto& uuid : uuids) {
        manager.registerUUID(uuid, 5); // 5 seconds timeout
    }

    // Simulate response for all UUIDs
    std::this_thread::sleep_for(std::chrono::seconds(2));

    for (const auto& uuid : uuids) {
        bool isValid = manager.validateUUID(uuid);
        EXPECT_TRUE(isValid) << "UUID " << uuid << " should be valid within timeout period.";
    }
}

TEST_F(UUIDResponseManagerTest, ConcurrentValidation) {
    UUIDResponseManager& manager = UUIDResponseManager::instance();
    const int numThreads = 10;
    const int timeout = 5; // 5 seconds
    std::vector<std::thread> threads;

    // Launch multiple threads
    for (int i = 0; i < numThreads; ++i) {
        std::string uuid = "uuid-" + std::to_string(i);
        manager.registerUUID(uuid, timeout);

        threads.emplace_back([&manager, uuid]() {
            std::this_thread::sleep_for(std::chrono::seconds(2));
            bool isValid = manager.validateUUID(uuid);
            EXPECT_TRUE(isValid) << "UUID " << uuid << " should be valid within timeout period.";
        });
    }

    // Join all threads
    for (auto& thread : threads) {
        thread.join();
    }
}

TEST_F(UUIDResponseManagerTest, ZeroTimeout) {
    UUIDResponseManager& manager = UUIDResponseManager::instance();
    std::string uuid = "123e4567-e89b-12d3-a456-426614174005";
    manager.registerUUID(uuid, 0); // Immediate timeout

    // Wait to ensure timeout has occurred
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    bool isValid = manager.validateUUID(uuid);

    EXPECT_FALSE(isValid) << "UUID should be invalid immediately after registration with zero timeout.";
}

TEST_F(UUIDResponseManagerTest, ValidateUUIDImmediately) {
    UUIDResponseManager& manager = UUIDResponseManager::instance();
    std::string uuid = "123e4567-e89b-12d3-a456-426614174006";
    manager.registerUUID(uuid, 10); // 10 seconds timeout

    // Validate immediately
    bool isValid = manager.validateUUID(uuid);

    EXPECT_TRUE(isValid) << "UUID should be valid immediately after registration.";
}

TEST_F(UUIDResponseManagerTest, ValidateUUIDAfterTimeout) {
    UUIDResponseManager& manager = UUIDResponseManager::instance();
    std::string uuid = "123e4567-e89b-12d3-a456-426614174007";
    manager.registerUUID(uuid, 2); // 2 seconds timeout

    // Wait for timeout
    std::this_thread::sleep_for(std::chrono::seconds(3));
    bool isValid = manager.validateUUID(uuid);

    EXPECT_FALSE(isValid) << "UUID should be invalid after timeout.";
}

TEST_F(UUIDResponseManagerTest, RegisterMultipleUUIDsAndValidate) {
    UUIDResponseManager& manager = UUIDResponseManager::instance();
    std::vector<std::string> uuids = {
        "123e4567-e89b-12d3-a456-426614174008",
        "123e4567-e89b-12d3-a456-426614174009",
        "123e4567-e89b-12d3-a456-426614174010"
    };

    for (const auto& uuid : uuids) {
        manager.registerUUID(uuid, 5); // 5 seconds timeout
    }

    // Simulate response for some UUIDs
    std::this_thread::sleep_for(std::chrono::seconds(2));

    for (size_t i = 0; i < uuids.size(); ++i) {
        bool isValid = manager.validateUUID(uuids[i]);
        EXPECT_TRUE(isValid) << "UUID " << uuids[i] << " should be valid within timeout period.";

    }
}

} // namespace NaviFra

