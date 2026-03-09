
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstring>
#include <functional>
#include <future>
#include <iomanip>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <tuple>
#include <type_traits>
#include <unordered_map>
#include <vector>

// Linux CAN headers
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <csignal>

// ========================================
// CANopen Protocol Definitions
// ========================================

namespace canopen {
// CANopen Function Codes
enum class FunctionCode : uint16_t
{
    NMT = 0x000,  // Network Management
    SYNC = 0x080,  // Synchronization
    EMCY = 0x080,  // Emergency (node-specific)
    TPDO1 = 0x180,  // Transmit PDO 1
    RPDO1 = 0x200,  // Receive PDO 1
    TPDO2 = 0x280,  // Transmit PDO 2
    RPDO2 = 0x300,  // Receive PDO 2
    TPDO3 = 0x380,  // Transmit PDO 3
    RPDO3 = 0x400,  // Receive PDO 3
    TPDO4 = 0x480,  // Transmit PDO 4
    RPDO4 = 0x500,  // Receive PDO 4
    TSDO = 0x580,  // Transmit SDO
    RSDO = 0x600,  // Receive SDO
    HEARTBEAT = 0x700  // Heartbeat/Node Guarding
};

// SDO Command Specifiers
enum class SDOCommand : uint8_t
{
    DOWNLOAD_INIT = 0x20,  // Download (write) initiate
    DOWNLOAD_SEGMENT = 0x00,  // Download segment
    UPLOAD_INIT = 0x40,  // Upload (read) initiate
    UPLOAD_SEGMENT = 0x60,  // Upload segment
    ABORT = 0x80  // Abort transfer
};

// SDO Response Specifiers
enum class SDOResponse : uint8_t
{
    DOWNLOAD_INIT_RESP = 0x60,  // Download initiate response
    DOWNLOAD_SEGMENT_RESP = 0x20,  // Download segment response
    UPLOAD_INIT_RESP = 0x40,  // Upload initiate response (with data)
    UPLOAD_SEGMENT_RESP = 0x00,  // Upload segment response
    ABORT_RESP = 0x80  // Abort response
};

// SDO Abort Codes
enum class SDOAbortCode : uint32_t
{
    TOGGLE_ERROR = 0x05030000,
    TIMEOUT = 0x05040000,
    UNKNOWN_COMMAND = 0x05040001,
    INVALID_BLOCK_SIZE = 0x05040002,
    INVALID_SEQUENCE = 0x05040003,
    CRC_ERROR = 0x05040004,
    OUT_OF_MEMORY = 0x05040005,
    UNSUPPORTED_ACCESS = 0x06010000,
    READ_ONLY = 0x06010001,
    WRITE_ONLY = 0x06010002,
    OBJECT_NOT_EXIST = 0x06020000,
    PDO_MAPPING_ERROR = 0x06040041,
    PDO_LENGTH_ERROR = 0x06040042,
    GENERAL_PARAMETER = 0x06040043,
    GENERAL_INTERNAL = 0x06040047,
    HARDWARE_ERROR = 0x06060000,
    TYPE_MISMATCH = 0x06070010,
    DATA_TOO_LONG = 0x06070012,
    DATA_TOO_SHORT = 0x06070013,
    SUBINDEX_NOT_EXIST = 0x06090011,
    VALUE_RANGE_ERROR = 0x06090030,
    VALUE_TOO_HIGH = 0x06090031,
    VALUE_TOO_LOW = 0x06090032,
    MAX_MIN_ERROR = 0x06090036,
    GENERAL_ERROR = 0x08000000,
    DATA_STORE_ERROR = 0x08000020,
    LOCAL_CONTROL_ERROR = 0x08000021,
    DEVICE_STATE_ERROR = 0x08000022
};

// NMT States
enum class NMTState : uint8_t
{
    INITIALISATION = 0x00,
    PRE_OPERATIONAL = 0x7F,
    OPERATIONAL = 0x05,
    STOPPED = 0x04,
    RESET_NODE = 0x81,
    RESET_COMMUNICATION = 0x82
};

// NMT Commands
enum class NMTCommand : uint8_t
{
    START_REMOTE_NODE = 0x01,
    STOP_REMOTE_NODE = 0x02,
    ENTER_PRE_OPERATIONAL = 0x80,
    RESET_NODE = 0x81,
    RESET_COMMUNICATION = 0x82
};

// Object Dictionary Entry
struct ODEntry {
    uint16_t index;
    uint8_t subindex;
    std::vector<uint8_t> data;
    bool writable = true;

    ODEntry(uint16_t idx, uint8_t sub, std::vector<uint8_t> d, bool w = true)
        : index(idx)
        , subindex(sub)
        , data(std::move(d))
        , writable(w)
    {
    }
};

// PDO Configuration
struct PDOConfig {
    uint32_t cob_id;
    uint8_t transmission_type = 0xFF;  // Event-driven
    uint16_t inhibit_time = 0;
    uint16_t event_timer = 0;
    std::vector<std::pair<uint16_t, uint8_t>> mapping;  // (index, subindex) pairs

    PDOConfig(uint32_t id)
        : cob_id(id)
    {
    }
};
}  // namespace canopen

// ========================================
// 엔디안 변환 유틸리티
// ========================================

namespace endian {
template <typename T>
T from_little_endian(const uint8_t* data)
{
    T result = 0;
    for (size_t i = 0; i < sizeof(T); ++i) {
        result |= static_cast<T>(data[i]) << (i * 8);
    }
    return result;
}

template <typename T>
T from_big_endian(const uint8_t* data)
{
    T result = 0;
    for (size_t i = 0; i < sizeof(T); ++i) {
        result = (result << 8) | data[i];
    }
    return result;
}

template <typename T>
void to_little_endian(T value, uint8_t* data)
{
    for (size_t i = 0; i < sizeof(T); ++i) {
        data[i] = static_cast<uint8_t>(value >> (i * 8));
    }
}

template <typename T>
void to_big_endian(T value, uint8_t* data)
{
    for (size_t i = 0; i < sizeof(T); ++i) {
        data[sizeof(T) - 1 - i] = static_cast<uint8_t>(value >> (i * 8));
    }
}
}  // namespace endian

// 파싱 전략
enum class EndianType
{
    LittleEndian,
    BigEndian
};

// ========================================
// 함수 타입 추론 시스템
// ========================================

template <typename R, typename... Args>
struct function_traits_impl {
    using args_tuple = std::tuple<Args...>;
    static constexpr size_t arity = sizeof...(Args);
};

template <typename R, typename... Args>
function_traits_impl<R, Args...> function_traits_helper(R (*)(Args...));

template <typename C, typename R, typename... Args>
function_traits_impl<R, Args...> function_traits_helper(R (C::*)(Args...) const);

template <typename C, typename R, typename... Args>
function_traits_impl<R, Args...> function_traits_helper(R (C::*)(Args...));

template <typename F>
auto function_traits_helper(F&& f) -> decltype(function_traits_helper(&std::decay_t<F>::operator()));

template <typename F>
using function_traits = decltype(function_traits_helper(std::declval<F>()));

template <typename F>
using function_args_tuple = typename function_traits<F>::args_tuple;

// ========================================
// 튜플 크기 계산
// ========================================

template <typename Tuple>
struct tuple_size_bytes;

template <typename... Types>
struct tuple_size_bytes<std::tuple<Types...>> {
    static constexpr size_t value = (sizeof(Types) + ...);
};

template <typename Tuple>
constexpr size_t tuple_size_bytes_v = tuple_size_bytes<Tuple>::value;

// ========================================
// 튜플 파서
// ========================================

template <typename Tuple, size_t Index = 0>
struct TupleParser {
    static void parse(Tuple& tuple, const uint8_t* data, size_t& offset, EndianType endian)
    {
        if constexpr (Index < std::tuple_size_v<Tuple>) {
            using ElementType = std::tuple_element_t<Index, Tuple>;

            auto& element = std::get<Index>(tuple);
            element = parseElement<ElementType>(data, offset, endian);
            offset += sizeof(ElementType);

            TupleParser<Tuple, Index + 1>::parse(tuple, data, offset, endian);
        }
    }

private:
    template <typename T>
    static T parseElement(const uint8_t* data, size_t offset, EndianType endian)
    {
        if constexpr (sizeof(T) == 1) {
            if constexpr (std::is_signed_v<T>) {
                return static_cast<T>(static_cast<int8_t>(data[offset]));
            }
            else {
                return static_cast<T>(data[offset]);
            }
        }
        else if constexpr (std::is_same_v<T, float>) {
            uint32_t raw_value;
            if (endian == EndianType::BigEndian) {
                raw_value = endian::from_big_endian<uint32_t>(data + offset);
            }
            else {
                raw_value = endian::from_little_endian<uint32_t>(data + offset);
            }
            float result;
            std::memcpy(&result, &raw_value, sizeof(float));
            return result;
        }
        else {
            if (endian == EndianType::BigEndian) {
                return endian::from_big_endian<T>(data + offset);
            }
            else {
                return endian::from_little_endian<T>(data + offset);
            }
        }
    }
};

// 튜플을 함수 인자로 전개
template <typename F, typename Tuple, size_t... I>
void apply_tuple_impl(F&& f, Tuple&& t, std::index_sequence<I...>)
{
    f(std::get<I>(t)...);
}

template <typename F, typename Tuple>
void apply_tuple(F&& f, Tuple&& t)
{
    apply_tuple_impl(std::forward<F>(f), std::forward<Tuple>(t), std::make_index_sequence<std::tuple_size_v<std::decay_t<Tuple>>>{});
}

// ========================================
// 자동 추론 콜백 클래스
// ========================================

template <typename F>
class AutoCallback {
private:
    F callback_;
    EndianType endian_type_;

    using ArgsTuple = function_args_tuple<F>;
    static constexpr size_t required_bytes = tuple_size_bytes_v<ArgsTuple>;

public:
    AutoCallback(F callback, EndianType endian = EndianType::LittleEndian)
        : callback_(std::move(callback))
        , endian_type_(endian)
    {
    }

    void operator()(const can_frame& frame)
    {
        if (frame.can_dlc < required_bytes) {
            std::cerr << "Warning: CAN frame data length (" << static_cast<int>(frame.can_dlc) << ") is less than required ("
                      << required_bytes << " bytes)" << std::endl;
            return;
        }

        ArgsTuple data_tuple;
        size_t offset = 0;
        TupleParser<ArgsTuple>::parse(data_tuple, frame.data, offset, endian_type_);

        apply_tuple(callback_, data_tuple);
    }

    static constexpr size_t getRequiredBytes() { return required_bytes; }

    std::string getTypeInfo() const { return generateTypeInfo<ArgsTuple>(); }

private:
    template <typename Tuple>
    std::string generateTypeInfo() const
    {
        return generateTypeInfoImpl<Tuple>(std::make_index_sequence<std::tuple_size_v<Tuple>>{});
    }

    template <typename Tuple, size_t... I>
    std::string generateTypeInfoImpl(std::index_sequence<I...>) const
    {
        std::string result = "auto(";
        ((result += getTypeName<std::tuple_element_t<I, Tuple>>() + (I < sizeof...(I) - 1 ? ", " : "")), ...);
        result += ")";
        return result;
    }

    template <typename T>
    std::string getTypeName() const
    {
        if constexpr (std::is_same_v<T, uint8_t>)
            return "uint8_t";
        else if constexpr (std::is_same_v<T, uint16_t>)
            return "uint16_t";
        else if constexpr (std::is_same_v<T, uint32_t>)
            return "uint32_t";
        else if constexpr (std::is_same_v<T, int8_t>)
            return "int8_t";
        else if constexpr (std::is_same_v<T, int16_t>)
            return "int16_t";
        else if constexpr (std::is_same_v<T, int32_t>)
            return "int32_t";
        else if constexpr (std::is_same_v<T, float>)
            return "float";
        else
            return "unknown";
    }
};

// ========================================
// Object Dictionary Manager
// ========================================

class ObjectDictionary {
private:
    std::unordered_map<uint32_t, canopen::ODEntry> entries_;

    uint32_t makeKey(uint16_t index, uint8_t subindex) const
    {
        return (static_cast<uint32_t>(index) << 8) | static_cast<uint32_t>(subindex);
    }

public:
    ObjectDictionary() { initializeStandardEntries(); }

    void addEntry(uint16_t index, uint8_t subindex, const std::vector<uint8_t>& data, bool writable = true)
    {
        uint32_t key = makeKey(index, subindex);
        entries_[key] = canopen::ODEntry(index, subindex, data, writable);
    }

    template <typename T>
    void addEntry(uint16_t index, uint8_t subindex, T value, bool writable = true)
    {
        std::vector<uint8_t> data(sizeof(T));
        if constexpr (sizeof(T) == 1) {
            data[0] = static_cast<uint8_t>(value);
        }
        else if constexpr (std::is_same_v<T, float>) {
            uint32_t raw_value;
            std::memcpy(&raw_value, &value, sizeof(float));
            endian::to_little_endian(raw_value, data.data());
        }
        else {
            endian::to_little_endian(value, data.data());
        }
        addEntry(index, subindex, data, writable);
    }

    bool getEntry(uint16_t index, uint8_t subindex, std::vector<uint8_t>& data) const
    {
        uint32_t key = makeKey(index, subindex);
        auto it = entries_.find(key);
        if (it != entries_.end()) {
            data = it->second.data;
            return true;
        }
        return false;
    }

    template <typename T>
    bool getEntry(uint16_t index, uint8_t subindex, T& value) const
    {
        std::vector<uint8_t> data;
        if (!getEntry(index, subindex, data) || data.size() < sizeof(T)) {
            return false;
        }

        if constexpr (sizeof(T) == 1) {
            value = static_cast<T>(data[0]);
        }
        else if constexpr (std::is_same_v<T, float>) {
            uint32_t raw_value = endian::from_little_endian<uint32_t>(data.data());
            std::memcpy(&value, &raw_value, sizeof(float));
        }
        else {
            value = endian::from_little_endian<T>(data.data());
        }
        return true;
    }

    bool setEntry(uint16_t index, uint8_t subindex, const std::vector<uint8_t>& data)
    {
        uint32_t key = makeKey(index, subindex);
        auto it = entries_.find(key);
        if (it != entries_.end() && it->second.writable) {
            it->second.data = data;
            return true;
        }
        return false;
    }

    template <typename T>
    bool setEntry(uint16_t index, uint8_t subindex, T value)
    {
        std::vector<uint8_t> data(sizeof(T));
        if constexpr (sizeof(T) == 1) {
            data[0] = static_cast<uint8_t>(value);
        }
        else if constexpr (std::is_same_v<T, float>) {
            uint32_t raw_value;
            std::memcpy(&raw_value, &value, sizeof(float));
            endian::to_little_endian(raw_value, data.data());
        }
        else {
            endian::to_little_endian(value, data.data());
        }
        return setEntry(index, subindex, data);
    }

    bool isWritable(uint16_t index, uint8_t subindex) const
    {
        uint32_t key = makeKey(index, subindex);
        auto it = entries_.find(key);
        return (it != entries_.end()) ? it->second.writable : false;
    }

    bool exists(uint16_t index, uint8_t subindex) const
    {
        uint32_t key = makeKey(index, subindex);
        return entries_.find(key) != entries_.end();
    }

    void listEntries() const
    {
        std::cout << "\n=== Object Dictionary Entries ===" << std::endl;
        for (const auto& [key, entry] : entries_) {
            std::cout << "Index: 0x" << std::hex << std::setw(4) << std::setfill('0') << entry.index << ", Sub: " << std::dec
                      << static_cast<int>(entry.subindex) << ", Size: " << entry.data.size() << " bytes"
                      << ", " << (entry.writable ? "R/W" : "R/O") << std::endl;
        }
        std::cout << "==================================" << std::endl;
    }

private:
    void initializeStandardEntries()
    {
        // Device type (0x1000)
        addEntry<uint32_t>(0x1000, 0, 0x00000000, false);

        // Error register (0x1001)
        addEntry<uint8_t>(0x1001, 0, 0x00, false);

        // Manufacturer status register (0x1002)
        addEntry<uint32_t>(0x1002, 0, 0x00000000, false);

        // Identity Object (0x1018)
        addEntry<uint8_t>(0x1018, 0, 4, false);
        addEntry<uint32_t>(0x1018, 1, 0x00000000, false);
        addEntry<uint32_t>(0x1018, 2, 0x00000000, false);
        addEntry<uint32_t>(0x1018, 3, 0x00010001, false);
        addEntry<uint32_t>(0x1018, 4, 0x00000000, false);

        // Server SDO parameters (0x1200)
        addEntry<uint8_t>(0x1200, 0, 2, false);
        addEntry<uint32_t>(0x1200, 1, 0x600, false);
        addEntry<uint32_t>(0x1200, 2, 0x580, false);

        // Application specific objects
        addEntry<uint16_t>(0x2000, 1, 250, true);
        addEntry<uint16_t>(0x2000, 2, 1013, true);
        addEntry<uint16_t>(0x2001, 1, 2000, true);
        addEntry<uint32_t>(0x2001, 2, 1500, true);
        addEntry<int16_t>(0x2002, 1, 100, true);
        addEntry<int16_t>(0x2002, 2, -50, true);
        addEntry<float>(0x2002, 3, 12.5f, true);
        addEntry<uint8_t>(0x2003, 1, 1, true);
        addEntry<uint8_t>(0x2003, 2, 2, true);
        addEntry<uint8_t>(0x2003, 3, 75, true);
    }
};

// ========================================
// SDO Manager
// ========================================

class SDOManager {
public:
    struct SDORequest {
        uint8_t node_id;
        uint16_t index;
        uint8_t subindex;
        std::vector<uint8_t> data;
        bool is_write;
        std::promise<std::vector<uint8_t>> promise;
        std::chrono::steady_clock::time_point timestamp;

        SDORequest(uint8_t nid, uint16_t idx, uint8_t sub, bool write)
            : node_id(nid)
            , index(idx)
            , subindex(sub)
            , is_write(write)
            , timestamp(std::chrono::steady_clock::now())
        {
        }
    };

private:
    std::unordered_map<uint32_t, std::unique_ptr<SDORequest>> pending_requests_;
    std::mutex requests_mutex_;
    std::function<bool(uint32_t, const std::vector<uint8_t>&)> send_callback_;
    static constexpr uint32_t SDO_TIMEOUT_MS = 1000;

public:
    SDOManager(std::function<bool(uint32_t, const std::vector<uint8_t>&)> send_cb)
        : send_callback_(std::move(send_cb))
    {
    }

    template <typename T>
    std::future<T> readObject(uint8_t node_id, uint16_t index, uint8_t subindex = 0)
    {
        auto promise = std::make_shared<std::promise<T>>();
        auto future = promise->get_future();

        auto request = std::make_unique<SDORequest>(node_id, index, subindex, false);
        uint32_t request_key = makeRequestKey(node_id, index, subindex);

        {
            std::lock_guard<std::mutex> lock(requests_mutex_);
            pending_requests_[request_key] = std::move(request);
        }

        std::vector<uint8_t> sdo_data(8, 0);
        sdo_data[0] = static_cast<uint8_t>(canopen::SDOCommand::UPLOAD_INIT);
        endian::to_little_endian(index, &sdo_data[1]);
        sdo_data[3] = subindex;

        uint32_t sdo_tx_id = static_cast<uint32_t>(canopen::FunctionCode::RSDO) + node_id;

        if (!send_callback_(sdo_tx_id, sdo_data)) {
            std::lock_guard<std::mutex> lock(requests_mutex_);
            pending_requests_.erase(request_key);
            promise->set_exception(std::make_exception_ptr(std::runtime_error("Failed to send SDO request")));
            return future;
        }

        std::thread([this, promise, request_key]() {
            try {
                auto raw_future = pending_requests_[request_key]->promise.get_future();
                if (raw_future.wait_for(std::chrono::milliseconds(SDO_TIMEOUT_MS)) == std::future_status::timeout) {
                    {
                        std::lock_guard<std::mutex> lock(requests_mutex_);
                        pending_requests_.erase(request_key);
                    }
                    promise->set_exception(std::make_exception_ptr(std::runtime_error("SDO timeout")));
                    return;
                }

                auto raw_data = raw_future.get();
                if (raw_data.size() < sizeof(T)) {
                    promise->set_exception(std::make_exception_ptr(std::runtime_error("Insufficient data for type")));
                    return;
                }

                T result;
                if constexpr (sizeof(T) == 1) {
                    result = static_cast<T>(raw_data[0]);
                }
                else if constexpr (std::is_same_v<T, float>) {
                    uint32_t raw_value = endian::from_little_endian<uint32_t>(raw_data.data());
                    std::memcpy(&result, &raw_value, sizeof(float));
                }
                else {
                    result = endian::from_little_endian<T>(raw_data.data());
                }

                promise->set_value(result);
            }
            catch (...) {
                promise->set_exception(std::current_exception());
            }
        }).detach();

        return future;
    }

    template <typename T>
    std::future<bool> writeObject(uint8_t node_id, uint16_t index, uint8_t subindex, T value)
    {
        auto promise = std::make_shared<std::promise<bool>>();
        auto future = promise->get_future();

        auto request = std::make_unique<SDORequest>(node_id, index, subindex, true);
        uint32_t request_key = makeRequestKey(node_id, index, subindex);

        std::vector<uint8_t> value_data(sizeof(T));
        if constexpr (sizeof(T) == 1) {
            value_data[0] = static_cast<uint8_t>(value);
        }
        else if constexpr (std::is_same_v<T, float>) {
            uint32_t raw_value;
            std::memcpy(&raw_value, &value, sizeof(float));
            endian::to_little_endian(raw_value, value_data.data());
        }
        else {
            endian::to_little_endian(value, value_data.data());
        }

        request->data = value_data;

        {
            std::lock_guard<std::mutex> lock(requests_mutex_);
            pending_requests_[request_key] = std::move(request);
        }

        std::vector<uint8_t> sdo_data(8, 0);
        uint8_t n = 4 - sizeof(T);
        sdo_data[0] = static_cast<uint8_t>(canopen::SDOCommand::DOWNLOAD_INIT) | (n << 2) | 0x03;
        endian::to_little_endian(index, &sdo_data[1]);
        sdo_data[3] = subindex;

        std::copy(value_data.begin(), value_data.end(), sdo_data.begin() + 4);

        uint32_t sdo_tx_id = static_cast<uint32_t>(canopen::FunctionCode::RSDO) + node_id;

        if (!send_callback_(sdo_tx_id, sdo_data)) {
            std::lock_guard<std::mutex> lock(requests_mutex_);
            pending_requests_.erase(request_key);
            promise->set_value(false);
            return future;
        }

        std::thread([this, promise, request_key]() {
            try {
                auto raw_future = pending_requests_[request_key]->promise.get_future();
                if (raw_future.wait_for(std::chrono::milliseconds(SDO_TIMEOUT_MS)) == std::future_status::timeout) {
                    {
                        std::lock_guard<std::mutex> lock(requests_mutex_);
                        pending_requests_.erase(request_key);
                    }
                    promise->set_value(false);
                    return;
                }

                raw_future.get();
                promise->set_value(true);
            }
            catch (...) {
                promise->set_value(false);
            }
        }).detach();

        return future;
    }

    void handleSDOResponse(uint8_t node_id, const can_frame& frame)
    {
        if (frame.can_dlc < 4)
            return;

        uint8_t command = frame.data[0];
        uint16_t index = endian::from_little_endian<uint16_t>(&frame.data[1]);
        uint8_t subindex = frame.data[3];

        uint32_t request_key = makeRequestKey(node_id, index, subindex);

        std::lock_guard<std::mutex> lock(requests_mutex_);
        auto it = pending_requests_.find(request_key);
        if (it == pending_requests_.end())
            return;

        auto& request = it->second;

        if ((command & 0xE0) == static_cast<uint8_t>(canopen::SDOResponse::ABORT_RESP)) {
            uint32_t abort_code = endian::from_little_endian<uint32_t>(&frame.data[4]);
            std::string error_msg = "SDO Abort: 0x" + std::to_string(abort_code);
            request->promise.set_exception(std::make_exception_ptr(std::runtime_error(error_msg)));
            pending_requests_.erase(it);
            return;
        }

        if (request->is_write) {
            if ((command & 0xE0) == static_cast<uint8_t>(canopen::SDOResponse::DOWNLOAD_INIT_RESP)) {
                std::vector<uint8_t> empty;
                request->promise.set_value(std::move(empty));
            }
        }
        else {
            if ((command & 0xE0) == static_cast<uint8_t>(canopen::SDOResponse::UPLOAD_INIT_RESP)) {
                if (command & 0x02) {
                    uint8_t n = (command >> 2) & 0x03;
                    size_t data_size = 4 - n;
                    std::vector<uint8_t> data(frame.data + 4, frame.data + 4 + data_size);
                    request->promise.set_value(std::move(data));
                }
            }
        }

        pending_requests_.erase(it);
    }

private:
    uint32_t makeRequestKey(uint8_t node_id, uint16_t index, uint8_t subindex) const
    {
        return (static_cast<uint32_t>(node_id) << 24) | (static_cast<uint32_t>(index) << 8) | static_cast<uint32_t>(subindex);
    }
};

// ========================================
// SDO Server
// ========================================

class SDOServer {
private:
    ObjectDictionary& od_;
    std::function<bool(uint32_t, const std::vector<uint8_t>&)> send_callback_;
    uint8_t node_id_;

public:
    SDOServer(ObjectDictionary& od, uint8_t node_id, std::function<bool(uint32_t, const std::vector<uint8_t>&)> send_cb)
        : od_(od)
        , node_id_(node_id)
        , send_callback_(std::move(send_cb))
    {
    }

    void handleSDORequest(const can_frame& frame)
    {
        if (frame.can_dlc < 4)
            return;

        uint8_t node_id = frame.can_id & 0x7F;
        if (node_id != node_id_)
            return;

        uint8_t command = frame.data[0];
        uint16_t index = endian::from_little_endian<uint16_t>(&frame.data[1]);
        uint8_t subindex = frame.data[3];

        std::cout << "🔄 SDO Request: cmd=0x" << std::hex << static_cast<int>(command) << ", index=0x" << index << ", subindex=" << std::dec
                  << static_cast<int>(subindex) << std::endl;

        if ((command & 0xE0) == static_cast<uint8_t>(canopen::SDOCommand::UPLOAD_INIT)) {
            handleUploadRequest(index, subindex);
        }
        else if ((command & 0xE0) == static_cast<uint8_t>(canopen::SDOCommand::DOWNLOAD_INIT)) {
            handleDownloadRequest(frame, index, subindex);
        }
    }

private:
    void handleUploadRequest(uint16_t index, uint8_t subindex)
    {
        std::vector<uint8_t> data;

        if (!od_.exists(index, subindex)) {
            sendAbortResponse(index, subindex, canopen::SDOAbortCode::OBJECT_NOT_EXIST);
            return;
        }

        if (!od_.getEntry(index, subindex, data)) {
            sendAbortResponse(index, subindex, canopen::SDOAbortCode::GENERAL_ERROR);
            return;
        }

        if (data.size() <= 4) {
            std::vector<uint8_t> response(8, 0);
            uint8_t n = 4 - data.size();
            response[0] = static_cast<uint8_t>(canopen::SDOResponse::UPLOAD_INIT_RESP) | 0x02 | (n << 2);
            endian::to_little_endian(index, &response[1]);
            response[3] = subindex;

            std::copy(data.begin(), data.end(), response.begin() + 4);

            uint32_t response_id = static_cast<uint32_t>(canopen::FunctionCode::TSDO) + node_id_;
            send_callback_(response_id, response);

            std::cout << "✅ SDO Upload response sent (expedited)" << std::endl;
        }
    }

    void handleDownloadRequest(const can_frame& frame, uint16_t index, uint8_t subindex)
    {
        if (!od_.exists(index, subindex)) {
            sendAbortResponse(index, subindex, canopen::SDOAbortCode::OBJECT_NOT_EXIST);
            return;
        }

        if (!od_.isWritable(index, subindex)) {
            sendAbortResponse(index, subindex, canopen::SDOAbortCode::READ_ONLY);
            return;
        }

        uint8_t command = frame.data[0];

        if (command & 0x02) {
            uint8_t n = (command >> 2) & 0x03;
            size_t data_size = 4 - n;

            std::vector<uint8_t> data(frame.data + 4, frame.data + 4 + data_size);

            if (od_.setEntry(index, subindex, data)) {
                std::vector<uint8_t> response(8, 0);
                response[0] = static_cast<uint8_t>(canopen::SDOResponse::DOWNLOAD_INIT_RESP);
                endian::to_little_endian(index, &response[1]);
                response[3] = subindex;

                uint32_t response_id = static_cast<uint32_t>(canopen::FunctionCode::TSDO) + node_id_;
                send_callback_(response_id, response);

                std::cout << "✅ SDO Download successful" << std::endl;
            }
            else {
                sendAbortResponse(index, subindex, canopen::SDOAbortCode::GENERAL_ERROR);
            }
        }
    }

    void sendAbortResponse(uint16_t index, uint8_t subindex, canopen::SDOAbortCode abort_code)
    {
        std::vector<uint8_t> response(8, 0);
        response[0] = static_cast<uint8_t>(canopen::SDOResponse::ABORT_RESP);
        endian::to_little_endian(index, &response[1]);
        response[3] = subindex;
        endian::to_little_endian(static_cast<uint32_t>(abort_code), &response[4]);

        uint32_t response_id = static_cast<uint32_t>(canopen::FunctionCode::TSDO) + node_id_;
        send_callback_(response_id, response);

        std::cout << "❌ SDO Abort sent: 0x" << std::hex << static_cast<uint32_t>(abort_code) << std::dec << std::endl;
    }
};

// ========================================
// PDO Manager
// ========================================

class PDOManager {
private:
    std::unordered_map<uint32_t, canopen::PDOConfig> pdo_configs_;
    std::function<bool(uint32_t, const std::vector<uint8_t>&)> send_callback_;

public:
    PDOManager(std::function<bool(uint32_t, const std::vector<uint8_t>&)> send_cb)
        : send_callback_(std::move(send_cb))
    {
    }

    void configurePDO(uint32_t cob_id, const canopen::PDOConfig& config)
    {
        pdo_configs_[cob_id] = config;
        std::cout << "PDO configured: COB-ID 0x" << std::hex << cob_id << std::dec << ", Mapping entries: " << config.mapping.size()
                  << std::endl;
    }

    template <typename... Args>
    bool transmitPDO(uint32_t cob_id, Args... args)
    {
        auto config_it = pdo_configs_.find(cob_id);
        if (config_it == pdo_configs_.end()) {
            canopen::PDOConfig default_config(cob_id);
            pdo_configs_[cob_id] = default_config;
        }

        std::vector<uint8_t> data;
        packData(data, args...);

        if (data.size() > 8) {
            std::cerr << "PDO data too large: " << data.size() << " bytes" << std::endl;
            return false;
        }

        return send_callback_(cob_id, data);
    }

    bool transmitSync()
    {
        std::vector<uint8_t> empty_data;
        return send_callback_(static_cast<uint32_t>(canopen::FunctionCode::SYNC), empty_data);
    }

    bool transmitHeartbeat(uint8_t node_id, canopen::NMTState state)
    {
        uint32_t heartbeat_id = static_cast<uint32_t>(canopen::FunctionCode::HEARTBEAT) + node_id;
        std::vector<uint8_t> data = {static_cast<uint8_t>(state)};
        return send_callback_(heartbeat_id, data);
    }

private:
    template <typename T>
    void packData(std::vector<uint8_t>& data, T value)
    {
        size_t current_pos = data.size();
        data.resize(current_pos + sizeof(T));

        if constexpr (sizeof(T) == 1) {
            data[current_pos] = static_cast<uint8_t>(value);
        }
        else if constexpr (std::is_same_v<T, float>) {
            uint32_t raw_value;
            std::memcpy(&raw_value, &value, sizeof(float));
            endian::to_little_endian(raw_value, &data[current_pos]);
        }
        else {
            endian::to_little_endian(value, &data[current_pos]);
        }
    }

    template <typename T, typename... Rest>
    void packData(std::vector<uint8_t>& data, T first, Rest... rest)
    {
        packData(data, first);
        packData(data, rest...);
    }
};

// ========================================
// 콜백 래퍼 클래스들
// ========================================

class CallbackWrapperBase {
public:
    virtual ~CallbackWrapperBase() = default;
    virtual void operator()(const can_frame& frame) = 0;
    virtual std::string getTypeInfo() const = 0;
    virtual size_t getRequiredBytes() const = 0;
};

template <typename F>
class AutoCallbackWrapper : public CallbackWrapperBase {
private:
    AutoCallback<F> callback_;

public:
    AutoCallbackWrapper(F callback, EndianType endian = EndianType::LittleEndian)
        : callback_(std::move(callback), endian)
    {
    }

    void operator()(const can_frame& frame) override { callback_(frame); }

    std::string getTypeInfo() const override { return callback_.getTypeInfo(); }

    size_t getRequiredBytes() const override { return AutoCallback<F>::getRequiredBytes(); }
};

// ========================================
// Enhanced SocketCAN Reader
// ========================================

class SocketCANReader {
public:
    using CallbackMap = std::unordered_map<uint32_t, std::unique_ptr<CallbackWrapperBase>>;

protected:
    int socket_fd_{-1};
    std::atomic<bool> running_{false};
    std::unique_ptr<std::thread> read_thread_;
    CallbackMap callbacks_;
    std::string interface_name_;

    std::unique_ptr<SDOManager> sdo_manager_;
    std::unique_ptr<PDOManager> pdo_manager_;
    std::unique_ptr<ObjectDictionary> object_dictionary_;
    std::unique_ptr<SDOServer> sdo_server_;

    uint8_t node_id_ = 1;
    canopen::NMTState nmt_state_ = canopen::NMTState::PRE_OPERATIONAL;

public:
    explicit SocketCANReader(const std::string& interface = "can0", uint8_t node_id = 1)
        : interface_name_(interface)
        , node_id_(node_id)
    {
        auto send_callback = [this](uint32_t can_id, const std::vector<uint8_t>& data) {
            return this->sendMessage(can_id, data);
        };

        sdo_manager_ = std::make_unique<SDOManager>(send_callback);
        pdo_manager_ = std::make_unique<PDOManager>(send_callback);
        object_dictionary_ = std::make_unique<ObjectDictionary>();
        sdo_server_ = std::make_unique<SDOServer>(*object_dictionary_, node_id, send_callback);
    }

    virtual ~SocketCANReader() { stop(); }

    SocketCANReader(const SocketCANReader&) = delete;
    SocketCANReader& operator=(const SocketCANReader&) = delete;

    bool initialize()
    {
        socket_fd_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
        if (socket_fd_ < 0) {
            std::cerr << "Error creating socket: " << strerror(errno) << std::endl;
            return false;
        }

        struct ifreq ifr;
        std::strcpy(ifr.ifr_name, interface_name_.c_str());
        if (ioctl(socket_fd_, SIOCGIFINDEX, &ifr) < 0) {
            std::cerr << "Error getting interface index: " << strerror(errno) << std::endl;
            close(socket_fd_);
            return false;
        }

        struct sockaddr_can addr {
        };
        addr.can_family = AF_CAN;
        addr.can_ifindex = ifr.ifr_ifindex;

        if (bind(socket_fd_, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) < 0) {
            std::cerr << "Error binding socket: " << strerror(errno) << std::endl;
            close(socket_fd_);
            return false;
        }

        std::cout << "CAN socket initialized on interface: " << interface_name_ << " (Node ID: " << static_cast<int>(node_id_) << ")"
                  << std::endl;
        return true;
    }

    template <typename F>
    void registerCallback(uint32_t can_id, F&& callback, EndianType endian = EndianType::LittleEndian)
    {
        auto wrapper = std::make_unique<AutoCallbackWrapper<std::decay_t<F>>>(std::forward<F>(callback), endian);

        std::cout << "Registering auto callback for CAN ID 0x" << std::hex << std::setw(3) << std::setfill('0') << can_id << std::dec
                  << " -> " << wrapper->getTypeInfo() << " (" << wrapper->getRequiredBytes() << " bytes)" << std::endl;

        callbacks_[can_id] = std::move(wrapper);
    }

    template <typename F>
    void registerPDOCallback(uint32_t cob_id, F&& callback, EndianType endian = EndianType::LittleEndian)
    {
        registerCallback(cob_id, std::forward<F>(callback), endian);
    }

    void configurePDO(uint32_t cob_id, const canopen::PDOConfig& config) { pdo_manager_->configurePDO(cob_id, config); }

    template <typename... Args>
    bool transmitPDO(uint32_t cob_id, Args... args)
    {
        return pdo_manager_->transmitPDO(cob_id, args...);
    }

    template <typename T>
    std::future<T> readSDO(uint8_t node_id, uint16_t index, uint8_t subindex = 0)
    {
        return sdo_manager_->readObject<T>(node_id, index, subindex);
    }

    template <typename T>
    std::future<bool> writeSDO(uint8_t node_id, uint16_t index, uint8_t subindex, T value)
    {
        return sdo_manager_->writeObject(node_id, index, subindex, value);
    }

    bool sendNMTCommand(canopen::NMTCommand command, uint8_t target_node = 0)
    {
        std::vector<uint8_t> data = {static_cast<uint8_t>(command), target_node};
        return sendMessage(static_cast<uint32_t>(canopen::FunctionCode::NMT), data);
    }

    bool transmitSync() { return pdo_manager_->transmitSync(); }

    bool transmitHeartbeat() { return pdo_manager_->transmitHeartbeat(node_id_, nmt_state_); }

    void setNMTState(canopen::NMTState state)
    {
        nmt_state_ = state;
        std::cout << "NMT State changed to: " << static_cast<int>(state) << std::endl;
    }

    canopen::NMTState getNMTState() const { return nmt_state_; }

    uint8_t getNodeId() const { return node_id_; }

    template <typename T>
    bool getODValue(uint16_t index, uint8_t subindex, T& value) const
    {
        return object_dictionary_->getEntry(index, subindex, value);
    }

    template <typename T>
    bool setODValue(uint16_t index, uint8_t subindex, T value)
    {
        return object_dictionary_->setEntry(index, subindex, value);
    }

    void addODEntry(uint16_t index, uint8_t subindex, const std::vector<uint8_t>& data, bool writable = true)
    {
        object_dictionary_->addEntry(index, subindex, data, writable);
    }

    template <typename T>
    void addODEntry(uint16_t index, uint8_t subindex, T value, bool writable = true)
    {
        object_dictionary_->addEntry(index, subindex, value, writable);
    }

    void listODEntries() const { object_dictionary_->listEntries(); }

    void unregisterCallback(uint32_t can_id) { callbacks_.erase(can_id); }

    void listCallbacks() const
    {
        std::cout << "\n=== Registered Auto Callbacks ===" << std::endl;
        if (callbacks_.empty()) {
            std::cout << "No callbacks registered." << std::endl;
        }
        else {
            for (const auto& [can_id, wrapper] : callbacks_) {
                std::cout << "CAN ID 0x" << std::hex << std::setw(3) << std::setfill('0') << can_id << std::dec << ": "
                          << wrapper->getTypeInfo() << " (" << wrapper->getRequiredBytes() << " bytes)" << std::endl;
            }
        }
        std::cout << "==================================" << std::endl << std::endl;
    }

    bool start()
    {
        if (socket_fd_ < 0 || running_.load()) {
            return false;
        }

        running_.store(true);
        read_thread_ = std::make_unique<std::thread>(&SocketCANReader::readLoop, this);
        std::cout << "CAN reader started..." << std::endl;
        return true;
    }

    void stop()
    {
        if (running_.load()) {
            running_.store(false);
            if (read_thread_ && read_thread_->joinable()) {
                read_thread_->join();
            }
            read_thread_.reset();
        }

        if (socket_fd_ >= 0) {
            close(socket_fd_);
            socket_fd_ = -1;
        }
        std::cout << "CAN reader stopped." << std::endl;
    }

    bool sendMessage(uint32_t can_id, const std::vector<uint8_t>& data)
    {
        if (socket_fd_ < 0 || data.size() > 8) {
            return false;
        }

        can_frame frame{};
        frame.can_id = can_id;
        frame.can_dlc = static_cast<uint8_t>(data.size());
        std::copy(data.begin(), data.end(), frame.data);

        ssize_t bytes_sent = write(socket_fd_, &frame, sizeof(can_frame));
        if (bytes_sent == sizeof(can_frame)) {
            std::cout << "CAN message sent: ID=0x" << std::hex << std::setw(3) << std::setfill('0') << can_id << std::dec
                      << ", DLC=" << static_cast<int>(frame.can_dlc) << std::endl;
            return true;
        }
        return false;
    }

private:
    void readLoop()
    {
        std::cout << "CAN read thread started..." << std::endl;

        while (running_.load()) {
            can_frame frame{};
            ssize_t bytes_read = read(socket_fd_, &frame, sizeof(can_frame));

            if (bytes_read == sizeof(can_frame)) {
                handleCANopenMessage(frame);

                auto it = callbacks_.find(frame.can_id);
                if (it != callbacks_.end()) {
                    try {
                        (*it->second)(frame);
                    }
                    catch (const std::exception& e) {
                        std::cerr << "Error in callback for CAN ID 0x" << std::hex << std::setw(3) << std::setfill('0') << frame.can_id
                                  << std::dec << ": " << e.what() << std::endl;
                    }
                }
                else if (!isCANopenMessage(frame.can_id)) {
                    std::cout << "Unhandled CAN message: ID=0x" << std::hex << std::setw(3) << std::setfill('0') << frame.can_id << std::dec
                              << ", DLC=" << static_cast<int>(frame.can_dlc) << std::endl;
                }
            }
        }

        std::cout << "CAN read thread terminated." << std::endl;
    }

    bool isCANopenMessage(uint32_t can_id) const
    {
        return (can_id == 0x000) ||  // NMT
            (can_id == 0x080) ||  // SYNC
            ((can_id & 0x780) == 0x580) ||  // SDO TX
            ((can_id & 0x780) == 0x600) ||  // SDO RX
            ((can_id & 0x780) == 0x180) ||  // PDO1 TX
            ((can_id & 0x780) == 0x200) ||  // PDO1 RX
            ((can_id & 0x780) == 0x280) ||  // PDO2 TX
            ((can_id & 0x780) == 0x300) ||  // PDO2 RX
            ((can_id & 0x780) == 0x380) ||  // PDO3 TX
            ((can_id & 0x780) == 0x400) ||  // PDO3 RX
            ((can_id & 0x780) == 0x480) ||  // PDO4 TX
            ((can_id & 0x780) == 0x500) ||  // PDO4 RX
            ((can_id & 0x780) == 0x700);  // Heartbeat
    }

    void handleCANopenMessage(const can_frame& frame)
    {
        uint32_t can_id = frame.can_id;

        if (can_id == 0x000) {
            handleNMTMessage(frame);
            return;
        }

        if (can_id == 0x080) {
            handleSyncMessage(frame);
            return;
        }

        if ((can_id & 0x780) == 0x580) {
            uint8_t node_id = can_id & 0x7F;
            sdo_manager_->handleSDOResponse(node_id, frame);
            return;
        }

        if ((can_id & 0x780) == 0x600) {
            sdo_server_->handleSDORequest(frame);
            return;
        }

        if ((can_id & 0x780) == 0x700) {
            handleHeartbeatMessage(frame);
            return;
        }
    }

    void handleNMTMessage(const can_frame& frame)
    {
        if (frame.can_dlc < 2)
            return;

        canopen::NMTCommand command = static_cast<canopen::NMTCommand>(frame.data[0]);
        uint8_t target_node = frame.data[1];

        if (target_node == node_id_ || target_node == 0) {
            switch (command) {
                case canopen::NMTCommand::START_REMOTE_NODE:
                    setNMTState(canopen::NMTState::OPERATIONAL);
                    break;
                case canopen::NMTCommand::STOP_REMOTE_NODE:
                    setNMTState(canopen::NMTState::STOPPED);
                    break;
                case canopen::NMTCommand::ENTER_PRE_OPERATIONAL:
                    setNMTState(canopen::NMTState::PRE_OPERATIONAL);
                    break;
                case canopen::NMTCommand::RESET_NODE:
                    setNMTState(canopen::NMTState::INITIALISATION);
                    std::cout << "Node reset requested" << std::endl;
                    break;
                case canopen::NMTCommand::RESET_COMMUNICATION:
                    setNMTState(canopen::NMTState::INITIALISATION);
                    std::cout << "Communication reset requested" << std::endl;
                    break;
            }
        }

        std::cout << "NMT Command: " << static_cast<int>(command) << " for node " << static_cast<int>(target_node) << std::endl;
    }

    void handleSyncMessage(const can_frame& frame) { std::cout << "SYNC received" << std::endl; }

    void handleHeartbeatMessage(const can_frame& frame)
    {
        if (frame.can_dlc < 1)
            return;

        uint8_t node_id = frame.can_id & 0x7F;
        canopen::NMTState state = static_cast<canopen::NMTState>(frame.data[0]);

        std::cout << "Heartbeat from node " << static_cast<int>(node_id) << ", state: " << static_cast<int>(state) << std::endl;
    }
};

// ========================================
// Full CANopen Application
// ========================================

class FullCANopenApplication {
private:
    std::unique_ptr<SocketCANReader> can_reader_;
    std::atomic<bool> running_{true};
    std::unique_ptr<std::thread> heartbeat_thread_;
    std::unique_ptr<std::thread> pdo_timer_thread_;

public:
    explicit FullCANopenApplication(const std::string& interface = "can0", uint8_t node_id = 1)
        : can_reader_(std::make_unique<SocketCANReader>(interface, node_id))
    {
    }

    ~FullCANopenApplication() { shutdown(); }

    bool initialize()
    {
        if (!can_reader_->initialize()) {
            return false;
        }
        setupCANopenCallbacks();
        setupCustomODEntries();
        return true;
    }

    void run()
    {
        if (!can_reader_->start()) {
            std::cerr << "Failed to start CAN reader" << std::endl;
            return;
        }

        can_reader_->listCallbacks();
        can_reader_->listODEntries();

        can_reader_->setNMTState(canopen::NMTState::PRE_OPERATIONAL);

        startPeriodicTasks();
        sendInitialMessages();
        handleUserInput();
    }

    void shutdown()
    {
        running_.store(false);
        if (heartbeat_thread_ && heartbeat_thread_->joinable()) {
            heartbeat_thread_->join();
        }
        if (pdo_timer_thread_ && pdo_timer_thread_->joinable()) {
            pdo_timer_thread_->join();
        }
        can_reader_->stop();
    }

private:
    void setupCANopenCallbacks()
    {
        std::cout << "🔧 Setting up full CANopen callbacks..." << std::endl;

        canopen::PDOConfig pdo1_config(0x181);
        pdo1_config.transmission_type = 1;
        pdo1_config.mapping = {{0x2000, 1}, {0x2000, 2}};
        can_reader_->configurePDO(0x181, pdo1_config);

        can_reader_->registerPDOCallback(0x181, [this](uint16_t temperature, uint16_t pressure) {
            std::cout << "🌡️  PDO1 RX - Climate: " << temperature / 10.0 << "°C, " << pressure << " hPa" << std::endl;
            can_reader_->setODValue<uint16_t>(0x2000, 1, temperature);
            can_reader_->setODValue<uint16_t>(0x2000, 2, pressure);
        });

        canopen::PDOConfig pdo2_config(0x281);
        pdo2_config.transmission_type = 0xFF;
        pdo2_config.mapping = {{0x2001, 1}, {0x2001, 2}};
        can_reader_->configurePDO(0x281, pdo2_config);

        can_reader_->registerPDOCallback(0x281, [this](uint16_t rpm, uint32_t current) {
            std::cout << "⚡ PDO2 RX - Engine: RPM=" << rpm << ", Current=" << current << " mA" << std::endl;
            can_reader_->setODValue<uint16_t>(0x2001, 1, rpm);
            can_reader_->setODValue<uint32_t>(0x2001, 2, current);
        });

        canopen::PDOConfig pdo3_config(0x381);
        pdo3_config.transmission_type = 2;
        pdo3_config.mapping = {{0x2002, canopen::PDOConfig pdo3_config(0x381);
        pdo3_config.transmission_type = 2;
        pdo3_config.mapping = {{0x2002, 1}, {0x2002, 2}, {0x2002, 3}};
        can_reader_->configurePDO(0x381, pdo3_config);

        can_reader_->registerPDOCallback(0x381, [this](int16_t x, int16_t y, float velocity) {
            std::cout << "📍 PDO3 RX - Position: X=" << x << ", Y=" << y << ", V=" << velocity << " m/s" << std::endl;
            can_reader_->setODValue<int16_t>(0x2002, 1, x);
            can_reader_->setODValue<int16_t>(0x2002, 2, y);
            can_reader_->setODValue<float>(0x2002, 3, velocity);
        });

        canopen::PDOConfig pdo4_config(0x481);
        pdo4_config.transmission_type = 0xFE;
        pdo4_config.inhibit_time = 100;
        pdo4_config.mapping = {{0x2003, 1}, {0x2003, 2}, {0x2003, 3}};
        can_reader_->configurePDO(0x481, pdo4_config);

        can_reader_->registerPDOCallback(0x481, [this](uint8_t status, uint8_t mode, uint8_t level) {
            std::cout << "⚙️  PDO4 RX - System: Status=" << (int)status << ", Mode=" << (int)mode << ", Level=" << (int)level << "%"
                      << std::endl;
            can_reader_->setODValue<uint8_t>(0x2003, 1, status);
            can_reader_->setODValue<uint8_t>(0x2003, 2, mode);
            can_reader_->setODValue<uint8_t>(0x2003, 3, level);
        });

        // Emergency callback
        can_reader_->registerCallback(
            0x81,
            [](uint16_t error_code, uint8_t error_register, uint8_t add_info1, uint8_t add_info2, uint8_t add_info3, uint8_t add_info4,
               uint8_t add_info5) {
                std::cout << "🚨 EMERGENCY: Code=0x" << std::hex << error_code << std::dec << ", Register=0x" << std::hex
                          << (int)error_register << std::dec << std::endl;
            });

        // Legacy auto-deduced callbacks
        can_reader_->registerCallback(0x100, [](uint16_t temperature, uint16_t pressure) {
            std::cout << "🌡️  Raw Climate: " << temperature / 10.0 << "°C, " << pressure << " hPa" << std::endl;
        });

        can_reader_->registerCallback(0x200, [](uint32_t count, uint32_t timestamp) {
            std::cout << "🕰️  Counter: " << count << ", Timestamp=" << timestamp << " ms" << std::endl;
        });

        std::cout << "✅ CANopen callbacks configured" << std::endl;
    }

    void setupCustomODEntries()
    {
        std::cout << "📚 Setting up custom Object Dictionary entries..." << std::endl;

        can_reader_->addODEntry<uint32_t>(0x2100, 0, 0x12345678, true);
        can_reader_->addODEntry<float>(0x2101, 0, 3.14159f, true);
        can_reader_->addODEntry<uint16_t>(0x2102, 0, 42, true);

        std::vector<uint8_t> device_name = {'C', 'A', 'N', 'o', 'p', 'e', 'n', ' ', 'D', 'e', 'v', 'i', 'c', 'e', '\0'};
        can_reader_->addODEntry(0x2200, 0, device_name, false);

        can_reader_->addODEntry<uint32_t>(0x2201, 0, 0x00010001, false);
        can_reader_->addODEntry<uint32_t>(0x2202, 0, 0x20241212, false);

        can_reader_->addODEntry<uint16_t>(0x2300, 1, 1000, true);
        can_reader_->addODEntry<uint8_t>(0x2300, 2, 1, true);
        can_reader_->addODEntry<float>(0x2300, 3, 1.0f, true);

        can_reader_->addODEntry<uint32_t>(0x2400, 1, 0, false);
        can_reader_->addODEntry<uint32_t>(0x2400, 2, 0, false);
        can_reader_->addODEntry<uint32_t>(0x2400, 3, 0, false);

        std::cout << "✅ Custom OD entries added" << std::endl;
    }

    void startPeriodicTasks()
    {
        std::cout << "🔄 Starting periodic tasks..." << std::endl;

        heartbeat_thread_ = std::make_unique<std::thread>([this]() {
            std::cout << "❤️  Heartbeat thread started" << std::endl;
            while (running_.load()) {
                can_reader_->transmitHeartbeat();

                uint32_t uptime;
                if (can_reader_->getODValue<uint32_t>(0x2400, 3, uptime)) {
                    can_reader_->setODValue<uint32_t>(0x2400, 3, uptime + 1);
                }

                std::this_thread::sleep_for(std::chrono::seconds(1));
            }
            std::cout << "❤️  Heartbeat thread terminated" << std::endl;
        });

        pdo_timer_thread_ = std::make_unique<std::thread>([this]() {
            std::cout << "📡 PDO timer thread started" << std::endl;
            int counter = 0;
            auto last_sync = std::chrono::steady_clock::now();

            while (running_.load()) {
                auto now = std::chrono::steady_clock::now();

                if (std::chrono::duration_cast<std::chrono::milliseconds>(now - last_sync).count() >= 100) {
                    can_reader_->transmitSync();
                    last_sync = now;
                }

                if (counter % 20 == 0) {
                    uint16_t temp = 250 + (counter % 50) - 25;
                    uint16_t pressure = 1013 + (counter % 20) - 10;

                    can_reader_->setODValue<uint16_t>(0x2000, 1, temp);
                    can_reader_->setODValue<uint16_t>(0x2000, 2, pressure);
                    can_reader_->transmitPDO(0x181, temp, pressure);
                }

                if (counter % 30 == 10) {
                    uint16_t rpm = 1500 + (counter % 1000);
                    uint32_t current = 1000 + (counter % 500);

                    can_reader_->setODValue<uint16_t>(0x2001, 1, rpm);
                    can_reader_->setODValue<uint32_t>(0x2001, 2, current);
                    can_reader_->transmitPDO(0x281, rpm, current);
                }

                uint32_t msg_count;
                if (can_reader_->getODValue<uint32_t>(0x2400, 1, msg_count)) {
                    can_reader_->setODValue<uint32_t>(0x2400, 1, msg_count + 1);
                }

                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                counter++;
            }
            std::cout << "📡 PDO timer thread terminated" << std::endl;
        });

        std::cout << "✅ Periodic tasks started" << std::endl;
    }

    void sendInitialMessages()
    {
        std::cout << "🚀 Sending initial CANopen messages..." << std::endl;

        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        std::cout << "📢 Sending boot-up message..." << std::endl;
        can_reader_->transmitHeartbeat();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        std::cout << "🔄 Entering operational state..." << std::endl;
        can_reader_->setNMTState(canopen::NMTState::OPERATIONAL);
        can_reader_->transmitHeartbeat();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        std::cout << "🔄 Sending initial SYNC..." << std::endl;
        can_reader_->transmitSync();

        std::cout << "✅ Initial messages sent" << std::endl;
    }

    void handleUserInput()
    {
        std::cout << "\n🎛️  Full CANopen System Ready!" << std::endl;
        std::cout << "Enhanced Commands:" << std::endl;
        std::cout << "  q/Q     - Quit application" << std::endl;
        std::cout << "  l/L     - List registered callbacks" << std::endl;
        std::cout << "  o/O     - Show Object Dictionary entries" << std::endl;
        std::cout << "  t/T     - Send comprehensive test messages" << std::endl;
        std::cout << "  s/S     - Send SYNC message" << std::endl;
        std::cout << "  h/H     - Send Heartbeat" << std::endl;
        std::cout << "  n/N     - NMT commands menu" << std::endl;
        std::cout << "  p/P     - PDO transmission test" << std::endl;
        std::cout << "  d/D     - SDO demonstration" << std::endl;
        std::cout << "  e/E     - Send Emergency message" << std::endl;
        std::cout << "  c/C     - Show statistics/counters" << std::endl;
        std::cout << "  1-4     - Send specific PDO messages" << std::endl;
        std::cout << "=================================================" << std::endl;

        char input;
        while (running_.load() && std::cin >> input) {
            switch (input) {
                case 'q':
                case 'Q':
                    std::cout << "🛑 Shutting down..." << std::endl;
                    return;
                case 'l':
                case 'L':
                    can_reader_->listCallbacks();
                    break;
                case 'o':
                case 'O':
                    can_reader_->listODEntries();
                    break;
                case 't':
                case 'T':
                    sendTestMessages();
                    break;
                case 's':
                case 'S':
                    can_reader_->transmitSync();
                    std::cout << "🔄 SYNC sent" << std::endl;
                    break;
                case 'h':
                case 'H':
                    can_reader_->transmitHeartbeat();
                    std::cout << "❤️  Heartbeat sent" << std::endl;
                    break;
                case 'n':
                case 'N':
                    handleNMTMenu();
                    break;
                case 'p':
                case 'P':
                    testPDOTransmission();
                    break;
                case 'd':
                case 'D':
                    demonstrateSDO();
                    break;
                case 'e':
                case 'E':
                    sendEmergencyMessage();
                    break;
                case 'c':
                case 'C':
                    showStatistics();
                    break;
                case '1':
                    can_reader_->transmitPDO(0x181, static_cast<uint16_t>(250), static_cast<uint16_t>(1013));
                    std::cout << "📡 PDO1 transmitted" << std::endl;
                    break;
                case '2':
                    can_reader_->transmitPDO(0x281, static_cast<uint16_t>(2000), static_cast<uint32_t>(1500));
                    std::cout << "📡 PDO2 transmitted" << std::endl;
                    break;
                case '3':
                    can_reader_->transmitPDO(0x381, static_cast<int16_t>(100), static_cast<int16_t>(-50), 12.5f);
                    std::cout << "📡 PDO3 transmitted" << std::endl;
                    break;
                case '4':
                    can_reader_->transmitPDO(0x481, static_cast<uint8_t>(1), static_cast<uint8_t>(2), static_cast<uint8_t>(75));
                    std::cout << "📡 PDO4 transmitted" << std::endl;
                    break;
                default:
                    std::cout << "❓ Unknown command: " << input << std::endl;
                    break;
            }
        }
    }

    void handleNMTMenu()
    {
        std::cout << "\n🎛️  NMT Commands Menu:" << std::endl;
        std::cout << "  1 - Start remote node (broadcast)" << std::endl;
        std::cout << "  2 - Stop remote node (broadcast)" << std::endl;
        std::cout << "  3 - Enter pre-operational (broadcast)" << std::endl;
        std::cout << "  4 - Reset node (broadcast)" << std::endl;
        std::cout << "  5 - Reset communication (broadcast)" << std::endl;
        std::cout << "  6 - Show current NMT state" << std::endl;
        std::cout << "Enter choice (1-6): ";

        char choice;
        if (std::cin >> choice) {
            switch (choice) {
                case '1':
                    can_reader_->sendNMTCommand(canopen::NMTCommand::START_REMOTE_NODE, 0);
                    std::cout << "▶️  START command sent (broadcast)" << std::endl;
                    break;
                case '2':
                    can_reader_->sendNMTCommand(canopen::NMTCommand::STOP_REMOTE_NODE, 0);
                    std::cout << "⏹️  STOP command sent (broadcast)" << std::endl;
                    break;
                case '3':
                    can_reader_->sendNMTCommand(canopen::NMTCommand::ENTER_PRE_OPERATIONAL, 0);
                    std::cout << "⏸️  PRE-OPERATIONAL command sent (broadcast)" << std::endl;
                    break;
                case '4':
                    can_reader_->sendNMTCommand(canopen::NMTCommand::RESET_NODE, 0);
                    std::cout << "🔄 RESET NODE command sent (broadcast)" << std::endl;
                    break;
                case '5':
                    can_reader_->sendNMTCommand(canopen::NMTCommand::RESET_COMMUNICATION, 0);
                    std::cout << "🔄 RESET COMMUNICATION command sent (broadcast)" << std::endl;
                    break;
                case '6':
                    std::cout << "Current NMT State: " << static_cast<int>(can_reader_->getNMTState()) << " (";
                    switch (can_reader_->getNMTState()) {
                        case canopen::NMTState::INITIALISATION:
                            std::cout << "INITIALISATION";
                            break;
                        case canopen::NMTState::PRE_OPERATIONAL:
                            std::cout << "PRE-OPERATIONAL";
                            break;
                        case canopen::NMTState::OPERATIONAL:
                            std::cout << "OPERATIONAL";
                            break;
                        case canopen::NMTState::STOPPED:
                            std::cout << "STOPPED";
                            break;
                        default:
                            std::cout << "UNKNOWN";
                            break;
                    }
                    std::cout << ")" << std::endl;
                    break;
                default:
                    std::cout << "❌ Invalid choice" << std::endl;
                    break;
            }
        }
    }

    void testPDOTransmission()
    {
        std::cout << "🔄 Testing comprehensive PDO transmission..." << std::endl;

        can_reader_->setODValue<uint16_t>(0x2000, 1, 235);
        can_reader_->setODValue<uint16_t>(0x2000, 2, 1008);
        can_reader_->setODValue<uint16_t>(0x2001, 1, 1800);
        can_reader_->setODValue<uint32_t>(0x2001, 2, 1200);

        uint16_t temp, pressure, rpm;
        uint32_t current;

        if (can_reader_->getODValue<uint16_t>(0x2000, 1, temp) && can_reader_->getODValue<uint16_t>(0x2000, 2, pressure)) {
            can_reader_->transmitPDO(0x181, temp, pressure);
            std::cout << "📡 PDO1 transmitted: Temp=" << temp / 10.0 << "°C, Pressure=" << pressure << "hPa" << std::endl;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(200));

        if (can_reader_->getODValue<uint16_t>(0x2001, 1, rpm) && can_reader_->getODValue<uint32_t>(0x2001, 2, current)) {
            can_reader_->transmitPDO(0x281, rpm, current);
            std::cout << "📡 PDO2 transmitted: RPM=" << rpm << ", Current=" << current << "mA" << std::endl;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(200));

        can_reader_->transmitPDO(0x381, static_cast<int16_t>(150), static_cast<int16_t>(-75), 8.5f);
        std::cout << "📡 PDO3 transmitted: Position data" << std::endl;

        std::this_thread::sleep_for(std::chrono::milliseconds(200));

        can_reader_->transmitPDO(0x481, static_cast<uint8_t>(2), static_cast<uint8_t>(1), static_cast<uint8_t>(85));
        std::cout << "📡 PDO4 transmitted: System status" << std::endl;

        std::cout << "✅ PDO transmission test completed" << std::endl;
    }

    void demonstrateSDO()
    {
        std::cout << "🔍 SDO Demonstration - Object Dictionary Access..." << std::endl;

        std::cout << "📖 Test 1: Reading temperature value (0x2000:1)..." << std::endl;
        auto read_future1 = can_reader_->readSDO<uint16_t>(can_reader_->getNodeId(), 0x2000, 1);

        std::thread([read_future1 = std::move(read_future1)]() mutable {
            try {
                auto result = read_future1.get();
                std::cout << "✅ Temperature read: " << result / 10.0 << "°C" << std::endl;
            }
            catch (const std::exception& e) {
                std::cout << "❌ Temperature read failed: " << e.what() << std::endl;
            }
        }).detach();

        std::this_thread::sleep_for(std::chrono::milliseconds(200));

        std::cout << "✏️  Test 2: Writing pressure value (0x2000:2)..." << std::endl;
        auto write_future = can_reader_->writeSDO<uint16_t>(can_reader_->getNodeId(), 0x2000, 2, 1020);

        std::thread([write_future = std::move(write_future)]() mutable {
            try {
                bool result = write_future.get();
                std::cout << (result ? "✅ Pressure write successful: 1020 hPa" : "❌ Pressure write failed") << std::endl;
            }
            catch (const std::exception& e) {
                std::cout << "❌ Pressure write failed: " << e.what() << std::endl;
            }
        }).detach();

        std::this_thread::sleep_for(std::chrono::milliseconds(200));

        std::cout << "📖 Test 3: Reading device type (0x1000:0)..." << std::endl;
        auto read_future2 = can_reader_->readSDO<uint32_t>(can_reader_->getNodeId(), 0x1000, 0);

        std::thread([read_future2 = std::move(read_future2)]() mutable {
            try {
                auto result = read_future2.get();
                std::cout << "✅ Device type read: 0x" << std::hex << result << std::dec << std::endl;
            }
            catch (const std::exception& e) {
                std::cout << "❌ Device type read failed: " << e.what() << std::endl;
            }
        }).detach();

        std::cout << "🔄 SDO demonstrations initiated (check results above)" << std::endl;
    }

    void sendEmergencyMessage()
    {
        std::cout << "🚨 Sending Emergency message..." << std::endl;

        uint16_t error_code = 0x1000;
        uint8_t error_register = 0x01;

        uint32_t emergency_id = 0x80 + can_reader_->getNodeId();
        std::vector<uint8_t> emergency_data(8);

        endian::to_little_endian(error_code, &emergency_data[0]);
        emergency_data[2] = error_register;
        emergency_data[3] = 0x12;
        emergency_data[4] = 0x34;
        emergency_data[5] = 0x56;
        emergency_data[6] = 0x78;
        emergency_data[7] = 0x9A;

        if (can_reader_->sendMessage(emergency_id, emergency_data)) {
            std::cout << "🚨 Emergency message sent: Error code 0x" << std::hex << error_code << std::dec << std::endl;
        }
        else {
            std::cout << "❌ Failed to send Emergency message" << std::endl;
        }
    }

    void showStatistics()
    {
        std::cout << "\n📊 System Statistics:" << std::endl;
        std::cout << "=====================================" << std::endl;

        uint32_t msg_count = 0, error_count = 0, uptime = 0;

        if (can_reader_->getODValue<uint32_t>(0x2400, 1, msg_count)) {
            std::cout << "📨 Message Count: " << msg_count << std::endl;
        }

        if (can_reader_->getODValue<uint32_t>(0x2400, 2, error_count)) {
            std::cout << "❌ Error Count: " << error_count << std::endl;
        }

        if (can_reader_->getODValue<uint32_t>(0x2400, 3, uptime)) {
            std::cout << "⏰ Uptime: " << uptime << " seconds (" << uptime / 60 << ":" << std::setfill('0') << std::setw(2) << uptime % 60
                      << ")" << std::endl;
        }

        uint16_t temp = 0, pressure = 0, rpm = 0;
        uint32_t current = 0;

        std::cout << "\n📈 Current Sensor Values:" << std::endl;
        if (can_reader_->getODValue<uint16_t>(0x2000, 1, temp)) {
            std::cout << "🌡️  Temperature: " << temp / 10.0 << "°C" << std::endl;
        }

        if (can_reader_->getODValue<uint16_t>(0x2000, 2, pressure)) {
            std::cout << "🌪️  Pressure: " << pressure << " hPa" << std::endl;
        }

        if (can_reader_->getODValue<uint16_t>(0x2001, 1, rpm)) {
            std::cout << "⚡ Engine RPM: " << rpm << std::endl;
        }

        if (can_reader_->getODValue<uint32_t>(0x2001, 2, current)) {
            std::cout << "🔌 Engine Current: " << current << " mA" << std::endl;
        }

        std::cout << "\n🖥️  System Information:" << std::endl;
        std::cout << "🏷️  Node ID: " << static_cast<int>(can_reader_->getNodeId()) << std::endl;
        std::cout << "🔄 NMT State: " << static_cast<int>(can_reader_->getNMTState()) << std::endl;

        uint32_t custom_param = 0;
        float pi_value = 0.0f;

        if (can_reader_->getODValue<uint32_t>(0x2100, 0, custom_param)) {
            std::cout << "🔧 Custom Parameter: 0x" << std::hex << custom_param << std::dec << std::endl;
        }

        if (can_reader_->getODValue<float>(0x2101, 0, pi_value)) {
            std::cout << "🥧 Pi Value: " << std::fixed << std::setprecision(5) << pi_value << std::endl;
        }

        std::cout << "=====================================" << std::endl;
    }

    void sendTestMessages()
    {
        std::cout << "🧪 Sending comprehensive CANopen test messages..." << std::endl;

        std::cout << "🔄 Sending SYNC..." << std::endl;
        can_reader_->transmitSync();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        std::cout << "📡 Sending PDO test messages..." << std::endl;

        can_reader_->transmitPDO(0x181, static_cast<uint16_t>(245), static_cast<uint16_t>(1015));
        std::this_thread::sleep_for(std::chrono::milliseconds(200));

        can_reader_->transmitPDO(0x281, static_cast<uint16_t>(1800), static_cast<uint32_t>(1200));
        std::this_thread::sleep_for(std::chrono::milliseconds(200));

        can_reader_->transmitPDO(0x381, static_cast<int16_t>(125), static_cast<int16_t>(-30), 15.5f);
        std::this_thread::sleep_for(std::chrono::milliseconds(200));

        can_reader_->transmitPDO(0x481, static_cast<uint8_t>(1), static_cast<uint8_t>(3), static_cast<uint8_t>(90));
        std::this_thread::sleep_for(std::chrono::milliseconds(200));

        std::cout << "❤️  Sending Heartbeat..." << std::endl;
        can_reader_->transmitHeartbeat();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        std::cout << "🔧 Sending legacy auto-deduced messages..." << std::endl;
        can_reader_->sendMessage(0x100, {0xFA, 0x00, 0xF5, 0x03});
        std::this_thread::sleep_for(std::chrono::milliseconds(200));

        can_reader_->sendMessage(0x200, {0x2A, 0x00, 0x00, 0x00, 0xE8, 0x03, 0x00, 0x00});
        std::this_thread::sleep_for(std::chrono::milliseconds(200));

        uint32_t msg_count;
        if (can_reader_->getODValue<uint32_t>(0x2400, 1, msg_count)) {
            can_reader_->setODValue<uint32_t>(0x2400, 1, msg_count + 10);
        }

        std::cout << "✅ Comprehensive test messages sent successfully" << std::endl;
    }
};

// ========================================
// Global Application Instance and Signal Handling
// ========================================

std::unique_ptr<FullCANopenApplication> g_canopen_app;

void signalHandler(int signal)
{
    std::cout << "\n🛑 Received signal " << signal << ", shutting down CANopen system gracefully..." << std::endl;
    if (g_canopen_app) {
        g_canopen_app->shutdown();
    }
    std::cout << "👋 Goodbye!" << std::endl;
    std::exit(0);
}

// ========================================
// Main Function
// ========================================

int main(int argc, char* argv[])
{
    std::string interface = argc > 1 ? argv[1] : "can0";
    uint8_t node_id = argc > 2 ? static_cast<uint8_t>(std::atoi(argv[2])) : 1;

    if (node_id == 0 || node_id > 127) {
        std::cerr << "❌ Invalid node ID: " << static_cast<int>(node_id) << " (must be 1-127)" << std::endl;
        return EXIT_FAILURE;
    }

    std::signal(SIGINT, signalHandler);
    std::signal(SIGTERM, signalHandler);

    std::cout << "🌟 ============================================= 🌟" << std::endl;
    std::cout << "     CANopen Enhanced Auto Type Deduction System   " << std::endl;
    std::cout << "              🚀 Full Featured Edition 🚀          " << std::endl;
    std::cout << "🌟 ============================================= 🌟" << std::endl;
    std::cout << "\n🎛️  Full CANopen System Ready!" << std::endl;
    std::cout << "Enhanced Commands:" << std::endl;
    std::cout << "  q/Q     - Quit application" << std::endl;
    std::cout << "  l/L     - List registered callbacks" << std::endl;
    std::cout << "  o/O     - Show Object Dictionary entries" << std::endl;
    std::cout << "  t/T     - Send comprehensive test messages" << std::endl;
    std::cout << "  s/S     - Send SYNC message" << std::endl;
    std::cout << "  h/H     - Send Heartbeat" << std::endl;
    std::cout << "  n/N     - NMT commands menu" << std::endl;
    std::cout << "  p/P     - PDO transmission test" << std::endl;
    std::cout << "  d/D     - SDO demonstration" << std::endl;
    std::cout << "  e/E     - Send Emergency message" << std::endl;
    std::cout << "  c/C     - Show statistics/counters" << std::endl;
    std::cout << "  1-4     - Send specific PDO messages" << std::endl;
    std::cout << "=================================================" << std::endl;

    char input;
    while (running_.load() && std::cin >> input) {
        switch (input) {
            case 'q':
            case 'Q':
                std::cout << "🛑 Shutting down..." << std::endl;
                return;
            case 'l':
            case 'L':
                can_reader_->listCallbacks();
                break;
            case 'o':
            case 'O':
                can_reader_->listODEntries();
                break;
            case 't':
            case 'T':
                sendTestMessages();
                break;
            case 's':
            case 'S':
                can_reader_->transmitSync();
                std::cout << "🔄 SYNC sent" << std::endl;
                break;
            case 'h':
            case 'H':
                can_reader_->transmitHeartbeat();
                std::cout << "❤️  Heartbeat sent" << std::endl;
                break;
            case 'n':
            case 'N':
                handleNMTMenu();
                break;
            case 'p':
            case 'P':
                testPDOTransmission();
                break;
            case 'd':
            case 'D':
                demonstrateSDO();
                break;
            case 'e':
            case 'E':
                sendEmergencyMessage();
                break;
            case 'c':
            case 'C':
                showStatistics();
                break;
            case '1':
                can_reader_->transmitPDO(0x181, static_cast<uint16_t>(250), static_cast<uint16_t>(1013));
                std::cout << "📡 PDO1 transmitted" << std::endl;
                break;
            case '2':
                can_reader_->transmitPDO(0x281, static_cast<uint16_t>(2000), static_cast<uint32_t>(1500));
                std::cout << "📡 PDO2 transmitted" << std::endl;
                break;
            case '3':
                can_reader_->transmitPDO(0x381, static_cast<int16_t>(100), static_cast<int16_t>(-50), 12.5f);
                std::cout << "📡 PDO3 transmitted" << std::endl;
                break;
            case '4':
                can_reader_->transmitPDO(0x481, static_cast<uint8_t>(1), static_cast<uint8_t>(2), static_cast<uint8_t>(75));
                std::cout << "📡 PDO4 transmitted" << std::endl;
                break;
            default:
                std::cout << "❓ Unknown command: " << input << std::endl;
                break;
        }
    }
}
