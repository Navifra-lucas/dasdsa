#include "core_agent/manager/publish_manager.h"

#include "gtest/gtest.h"

#include <chrono>
#include <thread>
namespace NaviFra {
class PublisherTest : public ::testing::Test {
protected:
    PublisherManager manager;

    void SetUp() override
    {
        // Setup code here
        manager.start();
    }

    void TearDown() override
    {
        // Cleanup code here
        manager.stop();
    }
};
void activateDeactivateChannel(PublisherManager& manager, int channelId, int iterations)
{
    for (int i = 0; i < iterations; ++i) {
        manager.activate(channelId);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));  // Sleep to simulate processing time
        manager.deactivate(channelId);
    }
}

TEST_F(PublisherTest, ChannelActivateDeactivate)
{
    bool executed = false;
    auto func = [&]() mutable {
        executed = true;
    };

    manager.addChannel(1, func, 100, -1);  // interval 100ms, no duration limit
    manager.activate(1);

    // Wait to ensure function is executed if the scheduling is correct
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    EXPECT_TRUE(executed) << "Function should have been executed after channel activation";

    executed = false;  // Reset and test deactivation
    manager.deactivate(1);
    std::this_thread::sleep_for(std::chrono::milliseconds(120));

    EXPECT_FALSE(executed) << "Function should not have been executed after channel deactivation";
}

TEST_F(PublisherTest, ChannelExecutionInterval)
{
    int executionCount = 0;
    auto func = [&]() {
        executionCount++;
    };

    manager.addChannel(1, func, 50, -1);  // interval 50ms
    manager.activate(1);

    std::this_thread::sleep_for(std::chrono::milliseconds(210));  // Should execute approx 4 times

    manager.deactivate(1);

    EXPECT_NEAR(executionCount, 4, 1) << "Function should execute about 4 times considering the interval";
}

TEST_F(PublisherTest, ChannelExecutionDuration)
{
    int executionCount = 0;
    auto func = [&]() mutable {
        executionCount++;
    };

    manager.addChannel(1, func, 50, 150);  // interval 50ms, duration 150ms
    manager.activate(1);

    std::this_thread::sleep_for(std::chrono::milliseconds(150));  // Should execute 3 times

    EXPECT_EQ(executionCount, 3) << "Function should stop executing after the specified duration";
}

TEST_F(PublisherTest, ConcurrentActivationDeactivation)
{
    int channelCount = 5;
    int iterationsPerThread = 10;

    // Prepare channels
    for (int i = 1; i <= channelCount; ++i) {
        manager.addChannel(
            i, []() {}, 100, 1000);  // Dummy function, interval, and duration
    }

    std::vector<std::thread> threads;
    for (int i = 1; i <= channelCount; ++i) {
        // Spawn threads that will activate and deactivate each channel
        threads.emplace_back(activateDeactivateChannel, std::ref(manager), i, iterationsPerThread);
    }

    // Join all threads
    for (auto& thread : threads) {
        thread.join();
    }

    // Assert conditions or perform checks to ensure correct state
    // For example, check if all channels are inactive:
    for (int i = 1; i <= channelCount; ++i) {
        EXPECT_FALSE(manager.isActive(i)) << "Channel " << i << " should be deactivated at the end of the test.";
    }
}

TEST_F(PublisherTest, ZeroIntervalExecution)
{
    int executionCount = 0;
    auto func = [&]() {
        executionCount++;
    };

    manager.addChannel(1, func, 0, 1000);  // interval 0ms, duration 1000ms
    manager.activate(1);

    std::this_thread::sleep_for(std::chrono::milliseconds(100));  // Short wait

    manager.deactivate(1);
    EXPECT_GT(executionCount, 50) << "Function should execute many times rapidly with zero interval.";
}

}  // namespace NaviFra