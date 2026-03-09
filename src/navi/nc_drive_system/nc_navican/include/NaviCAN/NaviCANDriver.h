#ifndef NAVIFRA_NAVICAN_DRIVER_H
#define NAVIFRA_NAVICAN_DRIVER_H

#include "NaviCAN/Utils/CANopenTypeTraits.h"
#include "NaviCAN/Utils/SyncPromise.h"
#include "NaviCAN/Utils/exchange.hpp"

#include <boost/lockfree/queue.hpp>
#include <boost/optional.hpp>
#include <boost/thread.hpp>
#include <lely/co/dcf.hpp>
#include <lely/co/dev.hpp>
#include <lely/co/obj.hpp>
#include <lely/coapp/fiber_driver.hpp>
#include <lely/coapp/master.hpp>
#include <lely/coapp/sdo_error.hpp>
#include <lely/ev/co_task.hpp>
#include <lely/ev/future.hpp>
#include <sys/stat.h>

#include <atomic>
#include <condition_variable>
#include <future>
#include <iostream>
#include <memory>
#include <mutex>
#include <shared_mutex>
#include <string>
#include <system_error>
#include <thread>
#include <vector>

using namespace std::chrono_literals;
using namespace lely;

namespace NaviFra {
struct pdo_mapping {
    bool is_tpdo;
    bool is_rpdo;
};

typedef std::map<uint16_t, std::map<uint8_t, pdo_mapping>> PDOMap;
class DriverDictionary : public lely::CODev {
public:
    DriverDictionary(std::string eds_file)
        : lely::CODev(eds_file.c_str())
    {
    }
    ~DriverDictionary()
    {
        // lely::CODev::~CODev();
    }

    std::shared_ptr<PDOMap> createPDOMapping()
    {
        std::shared_ptr<PDOMap> pdo_map = std::make_shared<PDOMap>();
        fetchRPDO(pdo_map);
        fetchTPDO(pdo_map);

        return pdo_map;
    }

    void fetchRPDO(std::shared_ptr<PDOMap> map)
    {
        for (int index = 0; index < 256; index++) {
            for (int subindex = 1; subindex < 9; subindex++) {
                auto obj = find(0x1600 + index, subindex);
                if (obj == nullptr) {
                    continue;
                }
                uint32_t data;
                {
                    data = obj->getVal<CO_DEFTYPE_UNSIGNED32>();
                }
                uint8_t tmps = (data >> 8) & 0xFF;
                uint16_t tmpi = (data >> 16) & 0xFFFF;
                if (tmpi == 0U) {
                    continue;
                }
                pdo_mapping mapping;
                mapping.is_rpdo = true;
                mapping.is_tpdo = false;
                (*map)[tmpi][tmps] = mapping;
            }
        }
    }
    void fetchTPDO(std::shared_ptr<PDOMap> map)
    {
        for (int index = 0; index < 256; index++) {
            for (int subindex = 1; subindex < 9; subindex++) {
                auto obj = find(0x1A00 + index, subindex);
                if (obj == nullptr) {
                    continue;
                }
                uint32_t data;
                {
                    data = obj->getVal<CO_DEFTYPE_UNSIGNED32>();
                }
                uint8_t tmps = (data >> 8) & 0xFF;
                uint16_t tmpi = (data >> 16) & 0xFFFF;
                if (tmpi == 0U) {
                    continue;
                }

                pdo_mapping mapping;
                mapping.is_rpdo = false;
                mapping.is_tpdo = true;
                (*map)[tmpi][tmps] = mapping;
            }
        }
    }

    bool checkObjRPDO(uint16_t idx, uint8_t subidx)
    {
        // std::cout << "Checking for rpo mapping of object: index=" << std::hex << (int)idx
        //           << " subindex=" << (int)subidx << std::endl;
        for (int i = 0; i < 256; i++) {
            if (this->checkObjInPDO(i, 0x1600, idx, subidx)) {
                return true;
            }
        }
        return false;
    }

    bool checkObjTPDO(uint16_t idx, uint8_t subidx)
    {
        // std::cout << "Checking for rpo mapping of object: index=" << std::hex << (int)idx
        //          << " subindex=" << (int)subidx << std::endl;
        for (int i = 0; i < 256; i++) {
            if (this->checkObjInPDO(i, 0x1A00, idx, subidx)) {
                return true;
            }
        }
        return false;
    }

    bool checkObjInPDO(uint8_t pdo, uint16_t mapping_idx, uint16_t idx, uint8_t subindex)
    {
        for (int i = 1; i < 9; i++) {
            auto obj = find(mapping_idx + pdo, i);
            if (obj == nullptr) {
                return false;
            }
            uint32_t data;
            {
                data = obj->getVal<CO_DEFTYPE_UNSIGNED32>();
            }
            uint8_t tmps = (data >> 8) & 0xFF;
            uint16_t tmpi = (data >> 16) & 0xFFFF;

            if (tmps == subindex && tmpi == idx) {
                // std::cout << "Found object in pdo: " << (int)pdo << std::endl;
                return true;
            }
        }

        return false;
    }
};

enum class LelyBridgeErrc
{
    NotListedDevice = 'A',
    NoResponseOnDeviceType = 'B',
    DeviceTypeDifference = 'C',
    VendorIdDifference = 'D',
    HeartbeatIssue = 'E',
    NodeGuardingIssue = 'F',
    InconsistentProgramDownload = 'G',
    SoftwareUpdateRequired = 'H',
    SoftwareDownloadFailed = 'I',
    ConfigurationDownloadFailed = 'J',
    StartErrorControlFailed = 'K',
    NmtSlaveInitiallyOperational = 'L',
    ProductCodeDifference = 'M',
    RevisionCodeDifference = 'N',
    SerialNumberDifference = 'O'
};

struct LelyBridgeErrCategory : std::error_category {
    const char* name() const noexcept override;
    std::string message(int ev) const override;
};
}  // namespace NaviFra

namespace std {
template <>
struct is_error_code_enum<NaviFra::LelyBridgeErrc> : true_type {
};
std::error_code make_error_code(NaviFra::LelyBridgeErrc);
}  // namespace std

namespace NaviFra {

class NaviCANDriver : public canopen::FiberDriver {
protected:
    // Dictionary for driver based on DCF and BIN files.
    std::unique_ptr<DriverDictionary> dictionary_;
    std::shared_mutex dictionary_mutex_;
    std::shared_ptr<PDOMap> pdo_map_;

    /**
     * @brief 타입별로 Dictionary에서 값을 가져오는 헬퍼 함수
     *
     * CANopenTypeTraits를 사용하여 각 C++ 타입을 적절한 CO_DEFTYPE으로 매핑합니다.
     * if constexpr를 사용하여 컴파일 타임에 타입을 결정합니다.
     */
    template <typename T>
    T getValueByType(uint16_t index, uint8_t subindex, T& fallback_value)
    {
        if constexpr (std::is_same_v<T, uint8_t>) {
            return this->dictionary_->getVal<CO_DEFTYPE_UNSIGNED8>(index, subindex);
        }
        else if constexpr (std::is_same_v<T, uint16_t>) {
            return this->dictionary_->getVal<CO_DEFTYPE_UNSIGNED16>(index, subindex);
        }
        else if constexpr (std::is_same_v<T, uint32_t>) {
            return this->dictionary_->getVal<CO_DEFTYPE_UNSIGNED32>(index, subindex);
        }
        else if constexpr (std::is_same_v<T, uint64_t>) {
            return this->dictionary_->getVal<CO_DEFTYPE_UNSIGNED64>(index, subindex);
        }
        else if constexpr (std::is_same_v<T, int8_t>) {
            return this->dictionary_->getVal<CO_DEFTYPE_INTEGER8>(index, subindex);
        }
        else if constexpr (std::is_same_v<T, int16_t>) {
            return this->dictionary_->getVal<CO_DEFTYPE_INTEGER16>(index, subindex);
        }
        else if constexpr (std::is_same_v<T, int32_t>) {
            return this->dictionary_->getVal<CO_DEFTYPE_INTEGER32>(index, subindex);
        }
        else if constexpr (std::is_same_v<T, int64_t>) {
            return this->dictionary_->getVal<CO_DEFTYPE_INTEGER64>(index, subindex);
        }
        else if constexpr (std::is_same_v<T, float>) {
            return this->dictionary_->getVal<CO_DEFTYPE_REAL32>(index, subindex);
        }
        else if constexpr (std::is_same_v<T, double>) {
            return this->dictionary_->getVal<CO_DEFTYPE_REAL64>(index, subindex);
        }
        else if constexpr (std::is_same_v<T, bool>) {
            return this->dictionary_->getVal<CO_DEFTYPE_BOOLEAN>(index, subindex);
        }
        else {
            // 지원하지 않는 타입
            static_assert(NaviCAN::Utils::is_canopen_type_v<T>, "Unsupported type for CANopen dictionary access");
            return fallback_value;
        }
    }

    // 동기화 유틸리티 (리팩토링으로 간소화)
    // SDO 동기화
    NaviCAN::Utils::SDOSync sdo_sync_;
    std::shared_ptr<std::promise<COData>> sdo_read_data_promise;
    std::shared_ptr<std::promise<bool>> sdo_write_data_promise;

    // NMT 동기화
    NaviCAN::Utils::SyncPromise<canopen::NmtState> nmt_sync_;

    // RPDO 동기화
    NaviCAN::Utils::SyncPromise<COData> rpdo_sync_;
    std::shared_ptr<SafeQueue<COData>> rpdo_queue;

    // EMCY 동기화
    NaviCAN::Utils::SyncPromise<COEmcy> emcy_sync_;
    std::shared_ptr<SafeQueue<COEmcy>> emcy_queue;

    // BOOT 동기화
    NaviCAN::Utils::BootSync boot_sync_;

    std::chrono::milliseconds sdo_timeout_;

    std::function<void()> on_sync_function_;

    std::atomic<double> latest_rpdo_write_time_count;

    // void set_sync_function(std::function<void()> on_sync_function)
    // {
    //   on_sync_function_ = on_sync_function;
    // }

    // void unset_sync_function()
    // {
    //   on_sync_function_ = std::function<void()>();
    // }

    void OnSync(uint8_t cnt, const time_point& t) noexcept override
    {
        if (on_sync_function_ != nullptr) {
            try {
                on_sync_function_();
            }
            catch (const std::exception& e) {
                // OnSync는 noexcept이므로 예외를 던질 수 없음 - stderr로 출력
                std::cerr << "[ERROR] OnSync callback error for node " << (int)id() << ": " << e.what() << std::endl;
            }
            catch (...) {
                std::cerr << "[ERROR] OnSync unknown error for node " << (int)id() << std::endl;
            }
        }
    }

    /**
     * @brief OnState Callback
     *
     * This callback function is called when an Nmt state
     * change is detected on the connected device.
     *
     * @param [in] state    NMT State
     */
    void OnState(canopen::NmtState state) noexcept override;

    /**
     * @brief OnBoot Callback
     * This callback is called when the Boot process of the
     * slave that was initiated by the master has been success
     * fully finished.
     *
     * @param st
     * @param es
     * @param what
     */
    virtual void OnBoot(canopen::NmtState st, char es, const ::std::string& what) noexcept override;

    /**
     * @brief OnRpdoWrite Callback
     *
     * This callback function is called when an RPDO
     * write request is received from the connected device.
     * @todo This function should use a threadsafe queue not the icky implementation we have now.
     *
     * @param [in] idx      Object Index
     * @param [in] subidx   Object Subindex
     */
    void OnRpdoWrite(uint16_t idx, uint8_t subidx) noexcept override;

    /**
     * The function invoked when an EMCY message is received from the remote node.
     * @todo This function should use a threadsafe queue not the icky implementation we have now.
     *
     * @param eec  the emergency error code.
     * @param er   the error register.
     * @param msef the manufacturer-specific error code.
     */
    void OnEmcy(uint16_t eec, uint8_t er, uint8_t msef[5]) noexcept override;

public:
    using FiberDriver::FiberDriver;

    /**
     * @brief Construct a new Lely Bridge object
     *
     * @param [in] exec     Executor to use
     * @param [in] master   Master to use
     * @param [in] id       Node ID of the device
     * @param [in] eds      EDS file
     * @param [in] bin      BIN file (concise dcf)
     * @param [in] timeout  Timeout in milliseconds for SDO reads/writes
     *
     */
    explicit NaviCANDriver(
        ev_exec_t* exec, canopen::AsyncMaster& master, uint8_t id, std::string eds, std::string bin,
        std::chrono::milliseconds sdo_timeout = 300ms)
        : FiberDriver(exec, master, id)
        , rpdo_queue(std::make_shared<SafeQueue<COData>>())
        , emcy_queue(std::make_shared<SafeQueue<COEmcy>>())
        , sdo_timeout_(sdo_timeout)
    {
        dictionary_ = std::make_unique<DriverDictionary>(eds.c_str());
        struct stat buffer;
        if (stat(bin.c_str(), &buffer) == 0) {
            co_unsigned16_t* a = NULL;
            co_unsigned16_t* b = NULL;
            dictionary_->readDCF(a, b, bin.c_str());
        }
        pdo_map_ = dictionary_->createPDOMapping();
    }

    double getLatestRpdoWriteTimeCount() const { return latest_rpdo_write_time_count; }

    /**
     * @brief Asynchronous SDO Write
     *
     * Writes the data passed to the function via SDO to
     * the connected device.
     *
     * @param [in] data     Data to written.
     *
     * @return std::future<bool>
     * Returns an std::future<bool> that is fulfilled
     * when the write request was done. An error is
     * stored when the write request was unsuccessful.
     */
    std::future<bool> async_sdo_write(COData data);

    template <typename T>
    std::future<bool> async_sdo_write_typed(uint16_t idx, uint8_t subidx, T value)
    {
        // SDOSync를 사용하여 동기화
        sdo_sync_.waitAndLock();

        auto prom = std::make_shared<std::promise<bool>>();
        lely::COSub* sub = this->dictionary_->find(idx, subidx);
        if (sub == nullptr) {
            std::cout << "async_sdo_write_typed: id=" << (unsigned int)id() << " index=0x" << std::hex << (unsigned int)idx
                      << " subindex=" << (unsigned int)subidx << " object does not exist" << std::endl;
            prom->set_value(false);
            sdo_sync_.unlock();
            return prom->get_future();
        }

        this->SubmitWrite(
            idx, subidx, value,
            [this, value, prom](uint8_t id, uint16_t idx, uint8_t subidx, ::std::error_code ec) mutable {
                if (ec) {
                    prom->set_exception(lely::canopen::make_sdo_exception_ptr(id, idx, subidx, ec, "AsyncDownload"));
                }
                else {
                    std::unique_lock<std::shared_mutex> lck(this->dictionary_mutex_);
                    this->dictionary_->setVal<T>(idx, subidx, value);
                    prom->set_value(true);
                }
                sdo_sync_.unlock();
            },
            this->sdo_timeout_);
        return prom->get_future();
    }

    template <typename T>
    bool sync_sdo_write_typed(uint16_t idx, uint8_t subidx, T value, std::chrono::milliseconds timeout)
    {
        auto fut = async_sdo_write_typed(idx, subidx, value);
        auto wait_res = fut.wait_for(timeout);
        if (wait_res == std::future_status::timeout) {
            std::cout << "sync_sdo_write_typed: id=" << (unsigned int)id() << " index=0x" << std::hex << (unsigned int)idx
                      << " subindex=" << (unsigned int)subidx << " timed out." << std::endl;
            return false;
        }
        bool res = false;
        try {
            res = fut.get();
        }
        catch (std::exception& e) {
            // RCLCPP_ERROR(rclcpp::get_logger(name_), e.what());
        }
        return res;
    }

    /**
     * @brief Aynchronous SDO Read
     *
     * Reads the indicated SDO object from the connected
     * device.
     *
     * @param [in] data       Data to be read, the data entry is not used.
     * @return std::future<COData>
     * Returns an std::future<COData> that is fulfilled
     * when the read request was done. The result of the request
     * is stored in the future. An error is stored when the read
     * request was unsuccessful.
     */
    std::future<COData> async_sdo_read(COData data);

    template <typename T>
    std::future<T> async_sdo_read_typed(uint16_t idx, uint8_t subidx)
    {
        sdo_sync_.waitAndLock();

        auto prom = std::make_shared<std::promise<T>>();
        lely::COSub* sub = this->dictionary_->find(idx, subidx);
        if (sub == nullptr) {
            std::cout << "async_sdo_read: id=" << (unsigned int)id() << " index=0x" << std::hex << (unsigned int)idx
                      << " subindex=" << (unsigned int)subidx << " object does not exist" << std::endl;
            try {
                throw lely::canopen::SdoError(id(), idx, subidx, lely::canopen::SdoErrc::NO_OBJ);
            }
            catch (...) {
                prom->set_exception(std::current_exception());
            }
            sdo_sync_.unlock();
            return prom->get_future();
        }
        this->SubmitRead<T>(
            idx, subidx,
            [this, prom](uint8_t id, uint16_t idx, uint8_t subidx, ::std::error_code ec, T value) mutable {
                if (ec) {
                    prom->set_exception(lely::canopen::make_sdo_exception_ptr(id, idx, subidx, ec, "AsyncUpload"));
                }
                else {
                    std::unique_lock<std::shared_mutex> lck(this->dictionary_mutex_);
                    this->dictionary_->setVal<T>(idx, subidx, value);
                    prom->set_value(value);
                }
                sdo_sync_.unlock();
            },
            this->sdo_timeout_);
        return prom->get_future();
    }

    template <typename T>
    bool sync_sdo_read_typed(uint16_t idx, uint8_t subidx, T& value, std::chrono::milliseconds timeout)
    {
        auto fut = async_sdo_read_typed<T>(idx, subidx);
        auto wait_res = fut.wait_for(timeout);
        if (wait_res == std::future_status::timeout) {
            std::cout << "sync_sdo_read_typed: id=" << (unsigned int)id() << " index=0x" << std::hex << (unsigned int)idx
                      << " subindex=" << (unsigned int)subidx << " timed out." << std::endl;
            return false;
        }
        bool res = false;
        try {
            value = fut.get();
            res = true;
        }
        catch (std::exception& e) {
            // RCLCPP_ERROR(rclcpp::get_logger(name_), e.what());
            res = false;
        }
        return res;
    }

    /**
     * @brief Asynchronous request for NMT
     *
     * Waits for an NMT state change to occur. The new
     * state is stored in the future returned by the function.
     *
     * @return std::future<canopen::NmtState>
     * The returned future is set when NMT State changes.
     */
    std::future<canopen::NmtState> async_request_nmt();

    /**
     * @brief Asynchronous request for RPDO
     *
     * Waits for an RPDO write request to be received from
     * the slave. The content of the request are stored in
     * the returned future.
     * @todo This function should use a threadsafe queue not the icky implementation we have now.
     *
     * @return std::future<COData>
     * The returned future is set when an RPDO event is detected.
     */
    std::shared_ptr<SafeQueue<COData>> get_rpdo_queue();

    /**
     * @brief Asynchronous request for EMCY
     * @todo This function should use a threadsafe queue not the icky implementation we have now.
     *
     * @return std::future<COEmcy>
     * The returned future is set when an EMCY event is detected.
     */
    std::shared_ptr<SafeQueue<COEmcy>> get_emcy_queue();

    /**
     * @brief Executes a TPDO transmission
     *
     * This function executes a TPDO transmission. The{false, true}
     * object specified in the input data is sent if it
     * is registered as a TPDO with the master.
     *
     * @param [in] data       Object and data to be written
     */
    void tpdo_transmit(COData data);

    /**
     * @brief Executes a NMT Command
     *
     * This function sends the NMT command specified as
     * parameter.
     *
     * @param [in] command    NMT Command to execute
     */
    void nmt_command(canopen::NmtCommand command);

    /**
     * @brief Wait for device to be booted
     *
     * @return true
     * @return false
     */
    bool wait_for_boot()
    {
        if (boot_sync_.isBooted()) {
            return true;
        }

        if (!boot_sync_.waitForBoot(std::chrono::seconds(10))) {
            throw std::runtime_error("Boot timeout");
        }

        auto boot_info = boot_sync_.getBootInfo();
        if ((boot_info.status != 0) && (boot_info.status != 'L')) {
            throw std::system_error(boot_info.status, LelyBridgeErrCategory(), "Boot Issue");
        }

        return true;
    }

    void set_sync_function(std::function<void()> on_sync_function) { on_sync_function_ = on_sync_function; }

    void unset_sync_function() { on_sync_function_ = std::function<void()>(); }

    /**
     * @brief Request master to boot device
     *
     */
    void Boot()
    {
        boot_sync_.reset();
        FiberDriver::Boot();
    }

    /**
     * @brief Indicates if Device is booted
     *
     * @return true
     * @return false
     */
    bool is_booted() { return boot_sync_.isBooted(); }

    template <typename T>
    void submit_write(COData data)
    {
        T value = 0;
        std::memcpy(&value, &data.data_, sizeof(value));

        this->SubmitWrite(
            data.index_, data.subindex_, value,
            [this, value](uint8_t id, uint16_t idx, uint8_t subidx, ::std::error_code ec) mutable {
                if (ec) {
                    this->sdo_write_data_promise->set_exception(
                        lely::canopen::make_sdo_exception_ptr(id, idx, subidx, ec, "AsyncDownload"));
                }
                else {
                    std::unique_lock<std::shared_mutex> lck(this->dictionary_mutex_);
                    this->dictionary_->setVal<T>(idx, subidx, value);
                    this->sdo_write_data_promise->set_value(true);
                }
                this->sdo_sync_.unlock();
            },
            this->sdo_timeout_);
    }

    template <typename T>
    void submit_read(COData data)
    {
        static_assert(sizeof(T) <= sizeof(uint32_t),
                      "COData.data_ is uint32_t (4 bytes). Use types <= 4 bytes only. "
                      "For 8-byte types (int64_t, uint64_t, double), use a different approach.");

        this->SubmitRead<T>(
            data.index_, data.subindex_,
            [this](uint8_t id, uint16_t idx, uint8_t subidx, ::std::error_code ec, T value) mutable {
                if (ec) {
                    this->sdo_read_data_promise->set_exception(lely::canopen::make_sdo_exception_ptr(id, idx, subidx, ec, "AsyncUpload"));
                }
                else {
                    std::unique_lock<std::shared_mutex> lck(this->dictionary_mutex_);
                    this->dictionary_->setVal<T>(idx, subidx, value);
                    COData d = {idx, subidx, 0};
                    std::memcpy(&d.data_, &value, sizeof(T));
                    this->sdo_read_data_promise->set_value(d);
                }
                this->sdo_sync_.unlock();
            },
            this->sdo_timeout_);
    }

    template <typename T>
    const T universal_get_value(uint16_t index, uint8_t subindex)
    {
        T value = 0;
        bool is_tpdo = false;
        if (this->pdo_map_->find(index) != this->pdo_map_->end()) {
            auto object = this->pdo_map_->at(index);
            if (object.find(subindex) != object.end()) {
                auto entry = object.at(subindex);
                is_tpdo = entry.is_tpdo;
            }
        }
        if (!is_tpdo) {
            if (sync_sdo_read_typed<T>(index, subindex, value, this->sdo_timeout_)) {
                return value;
            }
        }

        std::shared_lock<std::shared_mutex> lck(this->dictionary_mutex_);
        // CANopenTypeTraits를 사용하여 타입을 컴파일 타임에 매핑
        // 각 타입별로 적절한 CO_DEFTYPE으로 getVal 호출
        return getValueByType<T>(index, subindex, value);
    }

    template <typename T>
    void universal_set_value(uint16_t index, uint8_t subindex, T value)
    {
        bool is_rpdo = false;
        if (this->pdo_map_->find(index) != this->pdo_map_->end()) {
            auto object = this->pdo_map_->at(index);
            if (object.find(subindex) != object.end()) {
                auto entry = object.at(subindex);
                is_rpdo = entry.is_rpdo;
            }
        }
        if (is_rpdo) {
            std::unique_lock<std::shared_mutex> lck(this->dictionary_mutex_);
            this->dictionary_->setVal<T>(index, subindex, value);
            this->tpdo_mapped[index][subindex] = value;
            this->tpdo_mapped[index][subindex].WriteEvent();
        }
        else {
            sync_sdo_write_typed(index, subindex, value, this->sdo_timeout_);
        }
    }
};

}  // namespace NaviFra

#endif
