
//
#include "NaviCAN/NaviCANDriver.h"
#include "NaviCAN/Utils/CANopenTypeDispatcher.h"
#include "NaviCAN/Utils/CANopenTypeTraits.h"

#include "util/logger.hpp"

#include <bitset>
#include <memory>

const NaviFra::LelyBridgeErrCategory LelyBridgeErrCategoryInstance{};

std::error_code std::make_error_code(NaviFra::LelyBridgeErrc e)
{
    return {static_cast<int>(e), LelyBridgeErrCategoryInstance};
}

namespace NaviFra {

const char* LelyBridgeErrCategory::name() const noexcept
{
    return "LelyBridgeError";
}

std::string LelyBridgeErrCategory::message(int ev) const
{
    switch (static_cast<LelyBridgeErrc>(ev)) {
        case LelyBridgeErrc::NotListedDevice:
            return "The CANopen device is not listed in object 1F81.";
        case LelyBridgeErrc::NoResponseOnDeviceType:
            return "No response received for upload request of object 1000.";
        case LelyBridgeErrc::DeviceTypeDifference:
            return "Value of object 1000 from CANopen device is different to value in object 1F84 "
                   "(Device type).";
        case LelyBridgeErrc::VendorIdDifference:
            return "Value of object 1018:01 from CANopen device is different to value in object 1F85 "
                   "(Vendor-ID).";
        case LelyBridgeErrc::HeartbeatIssue:
            return "Heartbeat event. No heartbeat message received from CANopen device.";
        case LelyBridgeErrc::NodeGuardingIssue:
            return "Node guarding event. No confirmation for guarding request received from CANopen "
                   "device.";
        case LelyBridgeErrc::InconsistentProgramDownload:
            return "Objects for program download are not configured or inconsistent.";
        case LelyBridgeErrc::SoftwareUpdateRequired:
            return "Software update is required, but not allowed because of configuration or current "
                   "status.";
        case LelyBridgeErrc::SoftwareDownloadFailed:
            return "Software update is required, but program download failed.";
        case LelyBridgeErrc::ConfigurationDownloadFailed:
            return "Configuration download failed.";
        case LelyBridgeErrc::StartErrorControlFailed:
            return "Heartbeat event during start error control service. No heartbeat message received "
                   "from CANopen device during start error control service.";
        case LelyBridgeErrc::NmtSlaveInitiallyOperational:
            return "NMT slave was initially operational. (CANopen manager may resume operation with "
                   "other CANopen devices)";
        case LelyBridgeErrc::ProductCodeDifference:
            return "Value of object 1018:02 from CANopen device is different to value in object 1F86 "
                   "(Product code).";
        case LelyBridgeErrc::RevisionCodeDifference:
            return "Value of object 1018:03 from CANopen device is different to value in object 1F87 "
                   "(Revision number).";
        case LelyBridgeErrc::SerialNumberDifference:
            return "Value of object 1018:04 from CANopen device is different to value in object 1F88 "
                   "(Serial number).";
        default:
            return "(unrecognized error)";
    }
}

void NaviCANDriver::OnState(canopen::NmtState state) noexcept
{
    canopen::NmtState st = state;
    // We assume 1F80 bit 2 is false. All slaves are put into Operational after boot-up.
    // Lelycore does not track NMT states in this mode except BOOTUP.
    if (st == canopen::NmtState::BOOTUP) {
        st = canopen::NmtState::START;
    }

    // SyncPromise를 사용하여 스레드 안전하게 값 설정
    nmt_sync_.setValue(st);
}

void NaviCANDriver::OnBoot(canopen::NmtState st, char es, const ::std::string& what) noexcept
{
    FiberDriver::OnBoot(st, es, what);

    // BootSync를 사용하여 boot 정보 설정
    boot_sync_.setBoot(st, es, what);
}

void NaviCANDriver::OnRpdoWrite(uint16_t idx, uint8_t subidx) noexcept
{
    using namespace NaviCAN::Utils;

    latest_rpdo_write_time_count = std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();

    lely::COSub* sub = this->dictionary_->find(idx, subidx);
    if (sub == nullptr) {
        NLOG(info) << "OnRpdoWrite: id=" << (unsigned int)id() << " index=0x" << std::hex << (unsigned int)idx
                   << " subindex=" << (unsigned int)subidx << " object does not exist";
        return;
    }

    uint8_t co_def = (uint8_t)sub->getType();
    uint32_t data = 0;

    try {
        // 타입 디스패처를 사용하여 중복 제거
        dispatchByCoDefTypeVoid(co_def, [&]<typename T>() {
            std::unique_lock<std::shared_mutex> lck(this->dictionary_mutex_);
            T value = static_cast<T>(rpdo_mapped[idx][subidx]);
            sub->setVal<CANopenTypeTraits<T>::deftype>(value);
            std::memcpy(&data, &sub->getVal<CANopenTypeTraits<T>::deftype>(), sizeof(T));
        });
    } catch (const std::exception& e) {
        NLOG(severity_level::error) << "OnRpdoWrite failed: id=" << (unsigned int)id()
                                    << " index=0x" << std::hex << (unsigned int)idx
                                    << " subindex=" << (unsigned int)subidx
                                    << " error: " << e.what();
        return;
    }

    COData codata = {idx, subidx, data};
    //  We do not care so much about missing a message, rather push them through.
    //rpdo_queue->push(codata);
}

void NaviCANDriver::OnEmcy(uint16_t eec, uint8_t er, uint8_t msef[5]) noexcept
{
    FiberDriver::OnEmcy(eec, er, msef);

    COEmcy emcy;
    emcy.eec = eec;
    emcy.er = er;
    for (int i = 0; i < 5; i++)
        emcy.msef[i] = msef[i];

    //emcy_queue->push(emcy);
}

std::future<bool> NaviCANDriver::async_sdo_write(COData data)
{
    using namespace NaviCAN::Utils;

    // SDOSync를 사용하여 동기화
    sdo_sync_.waitAndLock();

    sdo_write_data_promise = std::make_shared<std::promise<bool>>();
    lely::COSub* sub = this->dictionary_->find(data.index_, data.subindex_);
    if (sub == nullptr) {
        NLOG(info) << "async_sdo_write: id=" << (unsigned int)id() << " index=0x" << std::hex << (unsigned int)data.index_
                   << " subindex=" << (unsigned int)data.subindex_ << " object does not exist";

        this->sdo_write_data_promise->set_value(false);
        sdo_sync_.unlock();
        return sdo_write_data_promise->get_future();
    }

    uint8_t co_def = (uint8_t)sub->getType();
    try {
        // 타입 디스패처를 사용하여 중복 제거
        dispatchByCoDefTypeVoid(co_def, [&]<typename T>() {
            this->submit_write<T>(data);
        });
    }
    catch (lely::canopen::SdoError& e) {
        this->sdo_read_data_promise->set_exception(
            lely::canopen::make_sdo_exception_ptr(id(), data.index_, data.subindex_, e.code(), "AsyncDownload"));
        sdo_sync_.unlock();
    }
    catch (const std::exception& e) {
        NLOG(severity_level::error) << "async_sdo_write failed: id=" << (unsigned int)id()
                                    << " index=0x" << std::hex << (unsigned int)data.index_
                                    << " subindex=" << (unsigned int)data.subindex_
                                    << " error: " << e.what();
        this->sdo_write_data_promise->set_value(false);
        sdo_sync_.unlock();
    }

    return sdo_write_data_promise->get_future();
}

std::future<COData> NaviCANDriver::async_sdo_read(COData data)
{
    using namespace NaviCAN::Utils;

    // SDOSync를 사용하여 동기화
    sdo_sync_.waitAndLock();

    sdo_read_data_promise = std::make_shared<std::promise<COData>>();
    lely::COSub* sub = this->dictionary_->find(data.index_, data.subindex_);
    if (sub == nullptr) {
        NLOG(info) << "async_sdo_read: id=" << (unsigned int)id() << " index=0x" << std::hex << (unsigned int)data.index_
                   << " subindex=" << (unsigned int)data.subindex_ << " object does not exist";
        try {
            throw lely::canopen::SdoError(id(), data.index_, data.subindex_, lely::canopen::SdoErrc::NO_OBJ);
        }
        catch (...) {
            this->sdo_read_data_promise->set_exception(std::current_exception());
        }
        sdo_sync_.unlock();
        return sdo_read_data_promise->get_future();
    }

    uint8_t co_def = (uint8_t)sub->getType();
    try {
        // 타입 디스패처를 사용하여 중복 제거
        dispatchByCoDefTypeVoid(co_def, [&]<typename T>() {
            this->submit_read<T>(data);
        });
    }
    catch (lely::canopen::SdoError& e) {
        this->sdo_read_data_promise->set_exception(
            lely::canopen::make_sdo_exception_ptr(id(), data.index_, data.subindex_, e.code(), "AsyncUpload"));
        sdo_sync_.unlock();
    }
    catch (const std::exception& e) {
        NLOG(severity_level::error) << "async_sdo_read failed: id=" << (unsigned int)id()
                                    << " index=0x" << std::hex << (unsigned int)data.index_
                                    << " subindex=" << (unsigned int)data.subindex_
                                    << " error: " << e.what();
        sdo_sync_.unlock();
    }

    return sdo_read_data_promise->get_future();
}

std::future<canopen::NmtState> NaviCANDriver::async_request_nmt()
{
    // SyncPromise를 사용하여 새로운 future 반환
    return nmt_sync_.getFuture();
}

std::shared_ptr<SafeQueue<COData>> NaviCANDriver::get_rpdo_queue()
{
    return rpdo_queue;
}

std::shared_ptr<SafeQueue<COEmcy>> NaviCANDriver::get_emcy_queue()
{
    return emcy_queue;
}

void NaviCANDriver::tpdo_transmit(COData data)
{
    using namespace NaviCAN::Utils;

    lely::COSub* sub = this->dictionary_->find(data.index_, data.subindex_);
    if (sub == nullptr) {
        NLOG(info) << "tpdo_transmit: id=" << (unsigned int)id() << " index=0x" << std::hex << (unsigned int)data.index_
                   << " subindex=" << (unsigned int)data.subindex_ << " object does not exist";
        return;
    }

    uint8_t co_def = (uint8_t)sub->getType();
    try {
        // 타입 디스패처를 사용하여 중복 제거
        dispatchByCoDefTypeVoid(co_def, [&]<typename T>() {
            T val;
            std::memcpy(&val, &data.data_, sizeof(T));
            tpdo_mapped[data.index_][data.subindex_] = val;

            std::unique_lock<std::shared_mutex> lck(this->dictionary_mutex_);
            sub->setVal<CANopenTypeTraits<T>::deftype>(val);
        });

        tpdo_mapped[data.index_][data.subindex_].WriteEvent();
        NLOG(info) << "tpdo_transmit: id=" << (unsigned int)id() << " index=0x" << std::hex << (unsigned int)data.index_
                   << " subindex=" << (unsigned int)data.subindex_ << " data:" << (uint32_t)data.data_;
    }
    catch (lely::canopen::SdoError& e) {
        NLOG(severity_level::error) << "tpdo_transmit SDO error: id=" << (unsigned int)id()
                                    << " index=0x" << std::hex << (unsigned int)data.index_
                                    << " subindex=" << (unsigned int)data.subindex_
                                    << " error: " << e.what();
        return;
    }
    catch (const std::exception& e) {
        NLOG(severity_level::error) << "tpdo_transmit failed: id=" << (unsigned int)id()
                                    << " index=0x" << std::hex << (unsigned int)data.index_
                                    << " subindex=" << (unsigned int)data.subindex_
                                    << " error: " << e.what();
        return;
    }
}

void NaviCANDriver::nmt_command(canopen::NmtCommand command)
{
    this->master.Command(command, id());
}

}  // namespace NaviFra
