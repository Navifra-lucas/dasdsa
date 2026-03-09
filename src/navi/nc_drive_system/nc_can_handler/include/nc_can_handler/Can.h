#pragma once

#include "nc_can_handler/CanTypeInference.h"

#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <atomic>
#include <cstring>
#include <iomanip>
#include <memory>
#include <thread>
#include <unordered_map>
#include <vector>

namespace NaviFra::CanHandler {

class Can {
    using CallbackMap = std::unordered_map<uint32_t, std::unique_ptr<CallbackWrapperBase>>;

    int socket_fd_{-1};
    std::atomic<bool> running_{false};
    std::unique_ptr<std::thread> read_thread_;
    CallbackMap callbacks_;
    std::string ifname_;

public:
    explicit Can(std::string ifname = "can0")
        : ifname_(std::move(ifname))
    {
    }
    ~Can() { stop(); }

    template <typename F>
    void registerCallback(uint32_t can_id, F&& callback, EndianType endian = EndianType::LittleEndian)
    {
        callbacks_[can_id] = std::make_unique<AutoCallbackWrapper<std::decay_t<F>>>(std::forward<F>(callback), endian);
    }

    void unregisterCallback(uint32_t can_id) { callbacks_.erase(can_id); }

    bool sendMessage(uint32_t can_id, const std::vector<uint8_t>& data);

    template <typename... Args>
    bool sendMessage(uint32_t can_id, Args... args)
    {
        return sendMessageWithEndian(can_id, EndianType::LittleEndian, args...);
    }

    template <typename... Args>
    bool sendMessageWithEndian(uint32_t can_id, EndianType endian, Args... args)
    {
        static_assert((sizeof(Args) + ...) <= 8, "CAN data exceeds 8 bytes");

        can_frame frame{};
        frame.can_id = can_id;
        frame.can_dlc = sizeof...(Args);

        size_t offset = 0;
        (packData(frame.data, offset, endian, args), ...);

        return sendFrame(frame);
    }

    bool initialize();
    bool start();
    void stop();
    void listCallbacks() const;

private:
    bool sendFrame(const can_frame& frame);
    void readLoop();

    template <typename T>
    void packData(uint8_t* data, size_t& offset, EndianType endian, T value)
    {
        if constexpr (sizeof(T) == 1) {
            data[offset++] = static_cast<uint8_t>(value);
        }
        else if constexpr (std::is_same_v<T, float>) {
            uint32_t raw;
            std::memcpy(&raw, &value, sizeof(float));
            packBytes(data, offset, endian, raw);
        }
        else {
            packBytes(data, offset, endian, value);
        }
    }

    template <typename T>
    void packBytes(uint8_t* data, size_t& offset, EndianType endian, T value)
    {
        if constexpr (sizeof(T) == 2) {
            // 2바이트 최적화
            if (endian == EndianType::BigEndian) {
                data[offset++] = static_cast<uint8_t>(value >> 8);
                data[offset++] = static_cast<uint8_t>(value);
            }
            else {
                data[offset++] = static_cast<uint8_t>(value);
                data[offset++] = static_cast<uint8_t>(value >> 8);
            }
        }
        else if constexpr (sizeof(T) == 4) {
            // 4바이트 최적화
            if (endian == EndianType::BigEndian) {
                data[offset++] = static_cast<uint8_t>(value >> 24);
                data[offset++] = static_cast<uint8_t>(value >> 16);
                data[offset++] = static_cast<uint8_t>(value >> 8);
                data[offset++] = static_cast<uint8_t>(value);
            }
            else {
                data[offset++] = static_cast<uint8_t>(value);
                data[offset++] = static_cast<uint8_t>(value >> 8);
                data[offset++] = static_cast<uint8_t>(value >> 16);
                data[offset++] = static_cast<uint8_t>(value >> 24);
            }
        }
        else {
            // 일반적인 경우 (8바이트 등)
            for (size_t i = 0; i < sizeof(T); ++i) {
                size_t shift = (endian == EndianType::BigEndian) ? (sizeof(T) - 1 - i) * 8 : i * 8;
                data[offset++] = static_cast<uint8_t>(value >> shift);
            }
        }
    }
};

}  // namespace NaviFra::CanHandler