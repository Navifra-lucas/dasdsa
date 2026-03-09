#ifndef NAVIFRA_PUBLISHER_POLLER_H
#define NAVIFRA_PUBLISHER_POLLER_H

#include <Poco/SingletonHolder.h>

#include <atomic>
#include <chrono>
#include <functional>
#include <future>
#include <map>
#include <memory>
#include <mutex>
#include <thread>

namespace NaviFra {

class PublishChannel {
public:
    PublishChannel(std::function<void()> func, int intervalMilliseconds, int durationMilliseconds);

    void execute();
    void activate();
    void deactivate();
    bool isActive() const { return active_; }
    auto getNextExecutionTime() const { return nextExecutionTime_; }
    void setInterval(int intervalMilliseconds) { interval_ = intervalMilliseconds; }
    std::mutex& getMutex() { return mutex_; }

private:
    std::function<void()> func_;
    int interval_;
    int duration_;
    std::chrono::steady_clock::time_point nextExecutionTime_;
    std::chrono::steady_clock::time_point startTime_;
    bool active_;
    std::mutex mutex_;
};

class PublisherManager {
public:
    PublisherManager();
    ~PublisherManager();

    static PublisherManager& instance()
    {
        static Poco::SingletonHolder<PublisherManager> sh;
        return *sh.get();
    }

    void addChannel(int id, std::function<void()> func, int intervalMilliseconds, int durationMilliseconds);
    void setChannelInterval(int channelId, int intervalMilliseconds);
    void activate(int channelId);
    bool isActive(int channelId);
    void deactivate(int channelId);
    void start();
    void run();
    void stop();

private:
    std::map<int, std::shared_ptr<PublishChannel>> channels_;
    std::thread thread_;
    std::atomic<bool> stopRequested_;
};

}  // namespace NaviFra

#endif  // NAVIFRA_PUBLISHER_POLLER_H
