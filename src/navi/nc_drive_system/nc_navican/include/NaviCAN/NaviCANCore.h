#ifndef NAVIFRA_NAVICAN_CAN_CORE_H
#define NAVIFRA_NAVICAN_CAN_CORE_H

#include "NaviCAN/EncoderDriver/ExternalEncoder.h"
#include "NaviCAN/MotorController/BaseMotorController.h"
#include "NaviCAN/MotorController/NullMotorController.h"
#include "NaviCAN/MotorController/interface/IMotorController.h"
#include "NaviCAN/MotorDriver/MotorDriver.h"
#include "NaviCAN/MotorDriver/MotorDriverFactory.h"
#include "NaviCAN/MotorDriver/interface/IMotorDriverBase.h"
#include "NaviCAN/MotorExtraInfo/MotorExtraInfo.h"
#include "NaviCAN/NaviCANDriver.h"
#include "NaviCAN/NaviCANMaster.h"
#include "NaviCAN/Utils/ConfigLoader.h"
#include "NaviCAN/Utils/LelyComponents.h"
#include "NaviCAN/interface/IMotor.h"

#include <lely/coapp/node.hpp>
#include <lely/coapp/slave.hpp>
#include <lely/io2/sys/sigset.hpp>

#include <functional>
#include <thread>
#include <unordered_map>

namespace NaviFra {
class NaviCANCore : public IMotor {
public:
    using OnSyncFunc = std::function<void(uint8_t, const io::TimerBase::time_point&)>;
    NaviCANCore() = default;
    ~NaviCANCore() = default;

    bool initialize();
    void finalize();

    void setTarget(uint8_t motorId, double value) override { getController(motorId)->setTarget(value); }
    int8_t getOperationMode(uint8_t motorId) override { return getController(motorId)->getOperationMode(); }
    double getPosition(uint8_t motorId) override { return getController(motorId)->getPosition(); }
    double getSpeed(uint8_t motorId) override { return getController(motorId)->getSpeed(); }

    double getCurrent(uint8_t motorId) override { return getController(motorId)->getCurrent(); }
    double getVoltage(uint8_t motorId) override { return getController(motorId)->getVoltage(); }
    uint32_t getErrorCode(uint8_t motorId) override { return getController(motorId)->getErrorCode(); }
    std::string getErrorMessage(uint8_t motorId) override { return getController(motorId)->getErrorMessage(); }
    uint16_t getStatus(uint8_t motorId) override { return getController(motorId)->getStatus(); }

    int getState(uint8_t motorId) override { return getController(motorId)->getState(); }
    bool isEnable(uint8_t motorId) override { return getController(motorId)->isEnable(); }
    bool enable(uint8_t motorId) override { return getController(motorId)->enable(); }
    bool disable(uint8_t motorId) override { return getController(motorId)->disable(); }
    bool shutdown(uint8_t motorId) override { return getController(motorId)->shutdown(); }
    bool resetError(uint8_t motorId) override { return getController(motorId)->resetError(); }
    bool homing(uint8_t motorId, int8_t homing_method, std::chrono::milliseconds timeout) override { return getController(motorId)->homing(homing_method, timeout); }
    bool isFault(uint8_t motorId) override { return getController(motorId)->isFault(); }
    double getLatestRpdoWriteTimeCount(uint8_t motorId) const override
    {
        return getDriverByMotorId(motorId)->getLatestRpdoWriteTimeCount();
    }
    int getCanBusState() override { return master_lely_->getCanBusState(); }

    bool presetEncoderAll();
    bool disableAll();
    bool enableAll();
    bool shutdownAll();
    bool resetErrorAll();

    auto& getControllers() { return controllers_; }

    void OnSyncWithMaster(OnSyncFunc func) { master_->OnSync(func); }

private:
    std::string config_path_ = std::string(std::getenv("HOME")) + "/navifra_solution/navicore/install/share/navican/config/";
    std::string config_filename_ = "config.yaml";

    std::shared_ptr<LelyComponents> master_lely_;
    std::shared_ptr<NaviCANMaster> master_;
    std::thread master_lely_loop_thread_;
    std::atomic<bool> master_lely_loop_running_ = false;

    std::shared_ptr<LelyComponents> slave_lely_;
    std::vector<std::shared_ptr<canopen::BasicSlave>> slaves_;
    std::thread slave_lely_loop_thread_;
    std::atomic<bool> slave_lely_loop_running_ = false;

    ConfigLoader::MasterConfig master_config_;
    std::vector<ConfigLoader::DriverConfig> driver_configs_;
    std::vector<ConfigLoader::ExternalEncoderConfig> encoder_configs_;
    std::vector<ConfigLoader::MotorConfig> motor_configs_;
    std::vector<ConfigLoader::MotorExtraInfo> motor_extra_configs_;

    std::unordered_map<uint8_t, std::shared_ptr<NaviCANDriver>> drivers_;
    std::unordered_map<uint8_t, std::shared_ptr<IMotorDriverBase>> motor_drivers_;
    std::unordered_map<uint8_t, std::shared_ptr<IExternalEncoder>> external_encoders_;
    std::unordered_map<uint8_t, std::shared_ptr<IMotorExtraInfo>> motor_extra_infos_;

    std::shared_ptr<NaviFra::NullMotorController> nullController_;
    std::unordered_map<uint8_t, std::shared_ptr<NaviFra::IMotorController>> controllers_;

    void registerMotorDrivers(void);

    bool initializeMotors(const std::vector<ConfigLoader::MotorConfig>& configs);
    bool initializeDrivers(const std::vector<ConfigLoader::DriverConfig>& configs);
    bool initializeExternalEncoders(const std::vector<ConfigLoader::ExternalEncoderConfig>& configs);
    bool initializeMotorExtraInfos(const std::vector<ConfigLoader::MotorExtraInfo>& configs);
    bool initializeMotorControllers(const std::vector<ConfigLoader::MotorConfig>& configs);

    std::shared_ptr<NaviCANDriver> getDriver(uint8_t driverId);
    std::shared_ptr<NaviCANDriver> getDriver(uint8_t driverId) const;
    std::shared_ptr<IMotorController> getController(uint8_t motorId);
    std::shared_ptr<IMotorController> getController(uint8_t motorId) const;

    template <typename Func>
    bool all(const std::string& name, Func func);

    template <typename Config, typename Func>
    bool initializeComponent(const std::vector<Config>& configs, const std::string& name, Func func);

    std::shared_ptr<NaviCANDriver> getDriverByMotorId(uint8_t motorId) const;
};
}  // namespace NaviFra
#endif  // NAVIFRA_CAN_CORE_H