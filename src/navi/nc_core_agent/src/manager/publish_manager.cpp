#include "core_agent/core_agent.h"

#include <core_agent/manager/publish_manager.h>

using namespace NaviFra;

PublishChannel::PublishChannel(std::function<void()> func, int intervalMilliseconds, int durationMilliseconds)
    : func_(func)
    , interval_(intervalMilliseconds)
    , duration_(durationMilliseconds)
    , nextExecutionTime_(std::chrono::steady_clock::now())
    , active_(true)
{
}

void PublishChannel::execute()
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (active_) {
        func_();
        nextExecutionTime_ += std::chrono::milliseconds(interval_);
        if (duration_ > 0 && (std::chrono::steady_clock::now() - startTime_ >= std::chrono::milliseconds(duration_))) {
            active_ = false;
        }
    }
}

void PublishChannel::activate()
{
    std::lock_guard<std::mutex> lock(mutex_);
    active_ = true;
    startTime_ = std::chrono::steady_clock::now();
    nextExecutionTime_ = std::chrono::steady_clock::now();
}

void PublishChannel::deactivate()
{
    std::lock_guard<std::mutex> lock(mutex_);
    active_ = false;
}

PublisherManager::PublisherManager()
    : stopRequested_(false)
{
}

PublisherManager::~PublisherManager()
{
    stop();
}

void PublisherManager::addChannel(int id, std::function<void()> func, int intervalMilliseconds, int durationMilliseconds)
{
    auto channel = std::make_shared<PublishChannel>(func, intervalMilliseconds, durationMilliseconds);
    channels_[id] = channel;
}

void PublisherManager::setChannelInterval(int channelId, int intervalMilliseconds)
{
    if (channels_.count(channelId)) {
        std::lock_guard<std::mutex> lock(channels_[channelId]->getMutex());
        channels_[channelId]->setInterval(intervalMilliseconds);
    }
}

void PublisherManager::activate(int channelId)
{
    if (channels_.count(channelId)) {
        channels_[channelId]->activate();
    }
}

bool PublisherManager::isActive(int channelId)
{
    return channels_[channelId]->isActive();
}

void PublisherManager::deactivate(int channelId)
{
    if (channels_.count(channelId)) {
        channels_[channelId]->deactivate();
    }
}

void PublisherManager::start()
{
    thread_ = std::thread(&PublisherManager::run, this);
}

void PublisherManager::run()
{
    while (!stopRequested_) {
        auto now = std::chrono::steady_clock::now();
        for (auto& pair : channels_) {
            auto& channel = pair.second;

            //세마포어 적용 해서 async 갯수 제안을 해야 되나 일단은.....세마포어는 c++20 이라 직접 구현 해...
            if (channel->getNextExecutionTime() <= now && channel->isActive()) {
                (void)std::async(std::launch::async, &PublishChannel::execute, channel);
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}

void PublisherManager::stop()
{
    for (const auto& channel : channels_) {
        channel.second->deactivate();
    };

    stopRequested_ = true;
    if (thread_.joinable()) {
        thread_.join();
    }
}