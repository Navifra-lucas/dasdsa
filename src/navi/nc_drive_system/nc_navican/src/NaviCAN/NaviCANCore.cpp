#include "NaviCAN/NaviCANCore.h"

#include "NaviCAN/MotorController/PIDMotorController.h"
#include "NaviCAN/MotorDriver/MotorDriverFactory.h"
#include "NaviCAN/MotorDriver/MotorDriverRegistrar.h"
#include "NaviCAN/MotorDriver/custom/curtis/theprime/MotorDriver.h"
#include "NaviCAN/MotorDriver/custom/swzm/MotorDriver.h"
#include "util/logger.hpp"

#include <chrono>
#include <fstream>
#include <iostream>

using namespace NaviFra;
using namespace std::chrono_literals;

bool NaviCANCore::initialize()
{
    nullController_ = std::make_shared<NaviFra::NullMotorController>();

    registerMotorDrivers();

    if (ConfigLoader::loadConfig(
            config_path_, config_filename_, master_config_, driver_configs_, encoder_configs_, motor_configs_, motor_extra_configs_) ==
        false) {
        return false;
    }

    try {
        master_lely_ = std::make_shared<LelyComponents>(master_config_.interface_name);
        slave_lely_ = std::make_shared<LelyComponents>(master_config_.interface_name);
        NLOG(info) << "lely initialized sucessfully.";
    }
    catch (std::exception& e) {
        NLOG(info) << "lely initialization failed.";
        return false;
    }

    try {
        master_ = std::make_shared<NaviCANMaster>(
            *master_lely_->exec, master_lely_->timer, master_lely_->chan, master_config_.dcf_txt, master_config_.dcf_bin,
            master_config_.master_id);

        auto slave = std::make_shared<canopen::BasicSlave>(
            *slave_lely_->exec, slave_lely_->timer, slave_lely_->chan, driver_configs_[0].eds_path, "", master_config_.master_id + 1);

        slave->OnCommand([&](lely::canopen::NmtCommand cs) {
            switch (cs) {
                case lely::canopen::NmtCommand::RESET_COMM:
                    for (auto slave_id : master_config_.fake_bootup_nodes) {
                        std::this_thread::sleep_for(50ms);
                        slave_lely_->chan.write(
                            {.id = static_cast<uint_least32_t>(0x700 + slave_id), .flags = 0, .len = 1, .data = {0x00}});
                    }
                    break;
                default:
                    break;
            }
        });

        slaves_.push_back(slave);

        master_->Reset();
        master_lely_loop_thread_ = std::thread([this]() {
            master_lely_loop_running_ = true;
            try {
                NLOG(info) << "Lely Loop Thread Started";
                master_lely_->loop.run();
            }
            catch (const std::exception& e) {
                NLOG(info) << e.what();
            }
            NLOG(info) << "Lely Loop Thread stopped";
        });

        std::this_thread::sleep_for(5ms);
        for (auto& slave : slaves_) {
            slave->Reset();
        }
    }
    catch (std::exception& e) {
        NLOG(severity_level::error) << "Master Initialization failed. e=" << e.what();
        return false;
    }

    try {
        if (initializeDrivers(driver_configs_) == false) {
            NLOG(severity_level::error) << "Failed to initialize drivers";
            return false;
        }

        for (auto node_id : master_config_.force_start_nodes) {
            NLOG(info) << "force start: node id: " << (int)node_id;
            master_->Command(lely::canopen::NmtCommand::START, node_id);
        }

        if (initializeExternalEncoders(encoder_configs_) == false) {
            NLOG(severity_level::error) << "Failed to initialize external encoders";
            return false;
        }

        if (initializeMotors(motor_configs_) == false) {
            NLOG(severity_level::error) << "Failed to initialize motors";
            return false;
        }

        if (!initializeMotorExtraInfos(motor_extra_configs_)) {
            NLOG(warning) << "Failed to initialize motor extra infos";
            return false;
        }

        if (!initializeMotorControllers(motor_configs_)) {
            NLOG(warning) << "Failed to initialize motor controllers";
            return false;
        }

        if (all("init", [](auto& controller) { return controller->init(); }) == false) {
            NLOG(severity_level::error) << "Failed to initialize some controllers";
            return false;
        }

        NLOG(info) << "initialization end";
        return true;
    }
    catch (const std::exception& e) {
        NLOG(severity_level::warning) << "initailize failed: " << e.what();
        return false;
    }

    slaves_.clear();
    slave_lely_.reset();
    return true;
}

void NaviCANCore::finalize()
{
    try {
        NLOG(info) << "NaviCANCore finalizing";
        all("shutdown", [](auto& controller) { return controller->shutdown(); });
        master_->Command(lely::canopen::NmtCommand::STOP, 0);
        controllers_.clear();
        motor_drivers_.clear();
        external_encoders_.clear();
        drivers_.clear();

        master_lely_loop_running_ = false;
        if (master_lely_loop_thread_.joinable()) {
            master_lely_loop_thread_.join();
        }

        slave_lely_.reset();
        slaves_.clear();

        master_lely_->chan.close();
        master_lely_.reset();
        master_.reset();

        NLOG(info) << "NaviCANCore finalized.";
    }
    catch (...) {
    }
}

void NaviCANCore::registerMotorDrivers(void)
{
    using namespace NaviFra::NaviCAN::MotorDriver;
    MotorDriverFactory::getInstance().registerMotorDriver<Custom::swzm::MotorDriver, Custom::swzm::MotorDriverParams>("swzm");
    MotorDriverFactory::getInstance()
        .registerMotorDriver<Custom::curtis::ThePrime::RTV3000::MotorDriver, Custom::curtis::ThePrime::RTV3000::MotorDriverParams>(
            "theprime_rtv3000_drive");
    MotorDriverFactory::getInstance()
        .registerMotorDriver<
            Custom::curtis::ThePrime::RTV3000::HydraulicsDriver, Custom::curtis::ThePrime::RTV3000::HydraulicsDriverParams>(
            "theprime_rtv3000_hydraulics");
    MotorDriverFactory::getInstance().registerMotorDriver<NaviFra::MotorDriver, NaviFra::MotorDriverParams>("default");
}

template <typename Config, typename Func>
bool NaviCANCore::initializeComponent(const std::vector<Config>& configs, const std::string& name, Func func)
{
    NLOG(info) << "===== Initialize Component  : " << name;
    bool success = std::all_of(configs.begin(), configs.end(), [this, &func, &name](const auto& cfg) {
        try {
            return func(cfg);
        }
        catch (const std::exception& e) {
            NLOG(severity_level::error) << "Failed to initialize " << name << ": " << e.what();
            return false;
        }
    });
    return success;
}

bool NaviCANCore::initializeDrivers(const std::vector<ConfigLoader::DriverConfig>& configs)
{
    return initializeComponent(configs, "drivers", [this](const auto& cfg) {
        if (drivers_.count(cfg.driver_id)) {
            NLOG(severity_level::warning) << "Driver for id " << int(cfg.driver_id) << " already exists";
            return true;
        }
        auto prom = std::make_shared<std::promise<std::shared_ptr<NaviCANDriver>>>();
        auto f = prom->get_future();

        master_lely_->exec->post([this, prom, cfg]() {
            try {
                auto driver = std::make_shared<NaviCANDriver>(
                    *master_lely_->exec, *master_, cfg.driver_id, cfg.eds_path, cfg.binary_path, cfg.sdo_timeout);
                drivers_[cfg.driver_id] = driver;
                prom->set_value(driver);
            }
            catch (const std::exception& e) {
                prom->set_exception(std::make_exception_ptr(e));
            }
            catch (...) {
                prom->set_exception(std::make_exception_ptr(std::runtime_error("Unknown error during driver initialization")));
            }
        });

        auto future_status = f.wait_for(500ms);
        if (future_status != std::future_status::ready) {
            NLOG(severity_level::error) << "Error: Driver initialization for id " << (int)cfg.driver_id << " timed out.";
            return false;
        }

        try {
            auto driver = f.get();
        }
        catch (const std::exception& e) {
            NLOG(severity_level::error) << "Error initializing driver for id " << (int)cfg.driver_id << ": " << e.what();
            return false;
        }

        NLOG(info) << "[Driver " << int(cfg.driver_id) << "] initialized";
        return true;
    });
}

bool NaviCANCore::initializeExternalEncoders(const std::vector<ConfigLoader::ExternalEncoderConfig>& configs)
{
    return initializeComponent(configs, "encoders", [this](const auto& cfg) {
        auto driver_it = drivers_.find(cfg.driver_id);
        if (driver_it == drivers_.end()) {
            NLOG(severity_level::error) << "Driver for id " << int(cfg.driver_id) << " not found";
            return false;
        }

        if (external_encoders_.count(cfg.encoder_id)) {
            NLOG(severity_level::warning) << "External encoder " << int(cfg.encoder_id) << " already exists";
            return true;
        }

        external_encoders_[cfg.encoder_id] = std::make_shared<ExternalEncoder>(driver_it->second);
        NLOG(info) << "[Encoder " << int(cfg.encoder_id) << "] initialized";
        return true;
    });
}

bool NaviCANCore::initializeMotors(const std::vector<ConfigLoader::MotorConfig>& configs)
{
    return initializeComponent(configs, "motors", [this](const auto& cfg) {
        // Create motor driver
        auto driver_it = drivers_.find(cfg.driver_id);
        if (driver_it == drivers_.end()) {
            NLOG(severity_level::error) << "Driver for id " << int(cfg.driver_id) << " not found";
            return false;
        }

        // addMotor
        std::unique_ptr<BaseMotorDriverParams> params;

        if (cfg.motor_driver == "swzm") {
            auto p = std::make_unique<NaviFra::NaviCAN::MotorDriver::Custom::swzm::MotorDriverParams>();
            p->driver = driver_it->second;
            p->handle_interval = cfg.handle_interval;
            p->motor_id = cfg.motor_id;
            params = std::move(p);
        }
        else if (cfg.motor_driver == "theprime_rtv3000_drive") {
            auto p = std::make_unique<NaviFra::NaviCAN::MotorDriver::Custom::curtis::ThePrime::RTV3000::MotorDriverParams>();
            p->driver = driver_it->second;
            p->handle_interval = cfg.handle_interval;
            p->motor_id = cfg.motor_id;
            p->steer_index = cfg.multiaxis_index;
            params = std::move(p);
        }
        else if (cfg.motor_driver == "theprime_rtv3000_hydraulics") {
            auto p = std::make_unique<NaviFra::NaviCAN::MotorDriver::Custom::curtis::ThePrime::RTV3000::HydraulicsDriverParams>();
            p->driver = driver_it->second;
            p->handle_interval = cfg.handle_interval;
            p->motor_id = cfg.motor_id;
            params = std::move(p);
        }
        else if (cfg.motor_driver == "default") {
            auto p = std::make_unique<NaviFra::MotorDriverParams>();
            p->driver = driver_it->second;
            p->handle_interval = cfg.handle_interval;
            p->motor_id = cfg.motor_id;
            p->mode_of_operation = cfg.mode_of_operation;
            p->multiaxis_index = cfg.multiaxis_index;
            p->initial_state = NaviCAN::Canopen::CIA402::StateHandler::State::OPERATION_ENABLE;
            p->enable_first = cfg.enable_first;
            params = std::move(p);
        }

        auto motor_driver = MotorDriverFactory::getInstance().createMotorDriver(cfg.motor_driver, *params);

        if (!motor_driver) {
            NLOG(info) << "[MotorDriver " << cfg.motor_driver << "] not found.";
            return false;
        }

        motor_drivers_[cfg.motor_id] = motor_driver;
        NLOG(info) << "[motorId " << int(cfg.motor_id) << "] initialized";

        return true;
    });
}

bool NaviCANCore::initializeMotorExtraInfos(const std::vector<ConfigLoader::MotorExtraInfo>& configs)
{
    return initializeComponent(configs, "extra", [this](const auto& cfg) {
        auto motor_driver_it = motor_drivers_.find(cfg.motor_id);
        if (motor_driver_it == motor_drivers_.end()) {
            NLOG(severity_level::error) << "MotorDriver for motor_id " << int(cfg.motor_id) << " not found";
            return false;
        }

        auto motor_extra_info = std::make_shared<NaviFra::MotorExtraInfo>(cfg, motor_driver_it->second);

        motor_extra_infos_[cfg.motor_id] = motor_extra_info;
        NLOG(info) << "[Extra Info motorId " << int(cfg.motor_id) << "] initialized";
        return true;
    });
}

bool NaviCANCore::initializeMotorControllers(const std::vector<ConfigLoader::MotorConfig>& configs)
{
    return initializeComponent(configs, "controller", [this](const auto& cfg) {
        // Get external encoder if needed
        std::shared_ptr<IExternalEncoder> external_encoder = nullptr;
        if (cfg.use_external_encoder) {
            auto enc_it = external_encoders_.find(cfg.encoder_id);
            if (enc_it != external_encoders_.end()) {
                external_encoder = enc_it->second;
                NLOG(info) << "Motor " << int(cfg.motor_id) << " using external encoder " << int(cfg.encoder_id);
            }
            else {
                NLOG(severity_level::warning) << "External encoder " << int(cfg.encoder_id) << " not found for motor " << int(cfg.motor_id);
            }
        }

        // Create controller
        if (cfg.use_pid) {
            controllers_[cfg.motor_id] = std::make_shared<PIDMotorController>(
                cfg.kp, cfg.ki, cfg.kd, motor_drivers_[cfg.motor_id], motor_drivers_[cfg.motor_id], external_encoder,
                motor_extra_infos_[cfg.motor_id]);
        }
        else {
            controllers_[cfg.motor_id] = std::make_shared<BaseMotorController>(
                motor_drivers_[cfg.motor_id], motor_drivers_[cfg.motor_id], external_encoder, motor_extra_infos_[cfg.motor_id]);
        }
        NLOG(info) << "[Controller motorId " << int(cfg.motor_id) << "] initialized";
        return true;
    });
}

std::shared_ptr<IMotorController> NaviCANCore::getController(uint8_t motorId)
{
    auto ct = controllers_.find(motorId);
    if (ct != controllers_.end()) {
        return ct->second;
    }
    else {
        return nullController_;
    }
}

std::shared_ptr<IMotorController> NaviCANCore::getController(uint8_t motorId) const
{
    auto ct = controllers_.find(motorId);
    if (ct != controllers_.end()) {
        return ct->second;
    }
    else {
        return nullController_;
    }
}

std::shared_ptr<NaviCANDriver> NaviCANCore::getDriver(uint8_t driverId)
{
    auto it = drivers_.find(driverId);
    if (it != drivers_.end()) {
        return it->second;
    }
    else {
        NLOG(severity_level::error) << "Driver with id " << int(driverId) << " not found";
        return nullptr;
    }
}

std::shared_ptr<NaviCANDriver> NaviCANCore::getDriver(uint8_t driverId) const
{
    auto it = drivers_.find(driverId);
    if (it != drivers_.end()) {
        return it->second;
    }
    else {
        NLOG(severity_level::error) << "Driver with id " << int(driverId) << " not found";
        return nullptr;
    }
}

std::shared_ptr<NaviCANDriver> NaviCANCore::getDriverByMotorId(uint8_t motorId) const
{
    auto config = std::find_if(motor_configs_.begin(), motor_configs_.end(), [motorId](const ConfigLoader::MotorConfig& config) {
        return config.motor_id == motorId;
    });
    if (config != motor_configs_.end()) {
        return getDriver(config->driver_id);
    }
    return nullptr;
}

template <typename Func>
bool NaviCANCore::all(const std::string& operationName, Func func)
{
    try {
        std::vector<std::future<bool>> futures;

        for (auto& [motorId, controller] : controllers_) {
            futures.push_back(std::async(std::launch::async, [&controller, &func]() { return func(controller); }));
        }

        bool success = true;
        for (auto& future : futures) {
            if (future.get() == false) {
                success = false;
            }
        }
        return success;
    }
    catch (const std::exception& ex) {
        NLOG(severity_level::error) << operationName << "() threw exception: " << ex.what();
        return false;
    }
}

bool NaviCANCore::disableAll()
{
    return all("disableAll", [](auto& controller) {
        controller->disable();
        return true;  // disable은 void 반환이므로 항상 true
    });
}

bool NaviCANCore::enableAll()
{
    return all("enableAll", [](auto& controller) { return controller->enable(); });
}

bool NaviCANCore::shutdownAll()
{
    return all("shutdownAll", [](auto& controller) { return controller->shutdown(); });
}

bool NaviCANCore::resetErrorAll()
{
    return all("resetErrorAll", [](auto& controller) {
        controller->resetError();
        return true;  // resetError가 void 반환이므로 항상 true
    });
}

bool NaviCANCore::presetEncoderAll()
{
    return all("presetEncoder", [](auto& controller) {
        controller->presetEncoder();
        return true;  // presetEncoder는 void 반환이므로 항상 true
    });
}
