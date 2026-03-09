#ifndef NAVIFRA_NAVICAN_EXCHANGE_HPP
#define NAVIFRA_NAVICAN_EXCHANGE_HPP

#include <boost/lockfree/queue.hpp>
#include <boost/optional.hpp>
#include <boost/thread.hpp>

namespace NaviFra {

struct COData {
public:
    uint16_t index_;
    uint8_t subindex_;
    uint32_t data_;
};

struct COEmcy {
public:
    uint16_t eec;
    uint8_t er;
    uint8_t msef[5];
};

/**
 * @brief Thread Safe Queue for CANOpen Data Exchange
 * @tparam T Type of the data to be exchanged
 * @details This class is a wrapper around boost::lockfree::queue to provide thread safe data
 * exchange between threads using the queue.
 */
template <typename T>
class SafeQueue {
private:
    std::size_t capacity_;
    std::unique_ptr<boost::lockfree::queue<T>> queue_;

public:
    /**
     * @brief Constructor for the SafeQueue
     * @param capacity Capacity of the queue
     */
    explicit SafeQueue(std::size_t capacity = 10)
        : capacity_(capacity)
        , queue_(new boost::lockfree::queue<T>(capacity_))
    {
    }

    /**
     * @brief Push a value to the queue
     * @param value Value to be pushed
     */
    void push(T value) { queue_->push(std::move(value)); }

    /**
     * @brief Try to pop a value from the queue
     * @return Value if available, boost::none otherwise
     */
    std::optional<T> try_pop()
    {
        T value;
        if (queue_->pop(value))
            return std::optional<T>(std::move(value));
        return std::optional<T>();
    }

    /**
     * @brief Try to pop a value from the queue
     * @param value Value to be returned
     */
    bool try_pop(T& value)
    {
        if (queue_->pop(value))
            return true;
        return false;
    }

    /**
     * @brief Wait for a value to be available in the queue
     * @return Value if available, boost::none otherwise
     */
    boost::optional<T> wait_and_pop()
    {
        T value;
        while (!queue_->pop(value))
            boost::this_thread::yield();
        return value;
    }

    /**
     * @brief Wait for a value to be available in the queue for a given timeout
     * @param value Value to be returned
     */
    void wait_and_pop(T& value)
    {
        while (!queue_->pop(value))
            boost::this_thread::yield();
    }

    /**
     * @brief Wait for a value to be available in the queue for a given timeout
     * @param timeout Timeout in milliseconds
     * @return Value if available, boost::none otherwise
     */
    boost::optional<T> wait_and_pop_for(const std::chrono::milliseconds& timeout)
    {
        T value;
        auto start_time = std::chrono::steady_clock::now();
        while (!queue_->pop(value)) {
            if (timeout != std::chrono::milliseconds::zero() && std::chrono::steady_clock::now() - start_time >= timeout)
                return boost::none;
            boost::this_thread::yield();
        }
        return value;
    }

    /**
     * @brief Wait for a value to be available in the queue for a given timeout
     * @param timeout Timeout in milliseconds
     * @param value Value to be returned
     */
    bool wait_and_pop_for(const std::chrono::milliseconds& timeout, T& value)
    {
        auto start_time = std::chrono::steady_clock::now();
        while (!queue_->pop(value)) {
            if (timeout != std::chrono::milliseconds::zero() && std::chrono::steady_clock::now() - start_time >= timeout)
                return false;
            boost::this_thread::yield();
        }
        return true;
    }

    bool empty() const { return queue_->empty(); }
};
}  // namespace NaviFra

#endif  // EXCHANGE_HPP
