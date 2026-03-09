#pragma once

#include <algorithm>
#include <functional>
#include <thread>
#include <vector>

/**
 * @brief Callback slot to hold and trigger multiple callbacks
 */
template <typename Func>
class CallbackSlot {
public:
    CallbackSlot() {}
    ~CallbackSlot()
    {
        for (auto &callback : callbacks) {
            callback = nullptr;
        }
    }

    /**
     * @brief Add a new callback
     * @param callback    Callback to be registered
     * @return int        Callback ID
     */
    int add(const std::function<Func> &callback)
    {
        callbacks.push_back(callback);
        return callbacks.size() - 1;
    }

    /**
     * @brief Remove a callback
     * @param callback_id  Callback ID
     */
    void remove(int callback_id) { callbacks[callback_id] = nullptr; }

    /**
     * @brief Check if the slot has a valid callback
     * @return true   The slot has at least one valid callback
     * @return false  No valid callbacks
     */
    operator bool() const
    {
        return !callbacks.empty() &&
            std::any_of(
                callbacks.begin(), callbacks.end(),
                [](const std::function<Func> &f) { return f; });
    }

    /**
     * @brief Call all the registered callbacks
     * @param args  Arguments for the callbacks
     */
    template <class... Args>
    void call(Args &&... args) const
    {
        if (callbacks.empty()) {
            return;
        }

        for (const auto &callback : callbacks) {
            if (callback) {
                callback(std::forward<Args>(args)...);
            }
        }
    }

    /**
     * @brief Call all the registered callbacks
     * @param args  Arguments for the callbacks
     */
    template <class... Args>
    void operator()(Args &&... args) const
    {
        return call(std::forward<Args>(args)...);
    }
    // --- 비동기 호출 : fire-and-forget 스타일 ---
    template <class... Args>
    void async_call(Args... args) const
    {
        for (const auto &cb : callbacks) {
            if (!cb)
                continue;
            std::thread t(cb, args...);
            t.detach();  // join은 안 하고 OS에 맡김
        }
    }

private:
    std::vector<std::function<Func>> callbacks;
};
