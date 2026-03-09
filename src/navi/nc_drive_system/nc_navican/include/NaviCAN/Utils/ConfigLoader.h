#ifndef NAVIFRA_NAVICAN_CONFIG_LOADER_H
#define NAVIFRA_NAVICAN_CONFIG_LOADER_H

#include "util/logger.hpp"

#include <yaml-cpp/yaml.h>

#include <chrono>
#include <string>
#include <vector>

using namespace std::chrono_literals;

namespace NaviFra {

class ConfigLoader {
public:
    struct MasterConfig {
        std::string interface_name;
        std::string dcf_txt;
        std::string dcf_bin;
        uint8_t master_id;
        std::vector<uint8_t> fake_bootup_nodes;
        std::vector<uint8_t> force_start_nodes;
    };

    struct DriverConfig {
        uint8_t driver_id;
        std::string eds_path;
        std::string binary_path;
        std::chrono::milliseconds sdo_timeout;
    };

    struct MotorConfig {
        uint8_t motor_id;
        uint8_t driver_id;
        std::string motor_driver;
        uint8_t multiaxis_index;
        uint8_t mode_of_operation;
        std::chrono::milliseconds handle_interval;
        double ratio;
        int ppr;
        bool enable_first;
        bool use_pid;
        double kp, ki, kd;
        bool use_external_encoder;
        uint8_t encoder_id;
    };

    struct ExternalEncoderConfig {
        uint8_t encoder_id;
        uint8_t driver_id;
    };

    struct ErrorCode {
        uint16_t value;
        std::string message;
    };

    struct SensorInfo {
        uint16_t index;
        uint8_t subindex;
        double multiplier;
        std::string type;
    };

    struct MotorExtraInfo {
        uint8_t motor_id;
        SensorInfo voltage;
        SensorInfo current;
        struct {
            uint16_t index;
            uint8_t subindex;
            std::string type;
            uint16_t sto;
            std::vector<ErrorCode> codes;
        } error;
    };

    template <typename Func>
    static bool tryLoadConfig(const std::string& name, Func func)
    {
        try {
            func();
            return true;
        }
        catch (const YAML::Exception& e) {
            NLOG(severity_level::error) << "Failed to load " << name << ": " << e.what();
            return false;
        }
        catch (const std::exception& e) {
            NLOG(severity_level::error) << "Exception during loading " << name << ": " << e.what();
            return false;
        }
        catch (...) {
            NLOG(severity_level::error) << "Unknown error during loading " << name;
            return false;
        }
    }

    static bool loadConfig(
        const std::string& config_path, const std::string& config_filename, MasterConfig& master_config, std::vector<DriverConfig>& drivers,
        std::vector<ExternalEncoderConfig>& external_encoders, std::vector<MotorConfig>& motors, std::vector<MotorExtraInfo>& motor_extras)
    {
        YAML::Node config;
        if (!tryLoadConfig("YAML File", [&]() { config = YAML::LoadFile(config_path + config_filename); }))
            return false;

        if (!tryLoadConfig("master section", [&]() {
                master_config.interface_name = config["master"]["interface_name"].as<std::string>();
                master_config.dcf_txt = config_path + config["master"]["dcf_txt"].as<std::string>();
                master_config.dcf_bin = config["master"]["dcf_bin"] ? config_path + config["master"]["dcf_bin"].as<std::string>() : "";
                master_config.master_id = static_cast<uint8_t>(config["master"]["master_id"].as<int>());
                if (config["master"]["fake_bootup_nodes"]) {
                    for (const auto& node : config["master"]["fake_bootup_nodes"]) {
                        master_config.fake_bootup_nodes.push_back(static_cast<uint8_t>(node.as<int>()));
                    }
                }
                if (config["master"]["force_start_nodes"]) {
                    for (const auto& node : config["master"]["force_start_nodes"]) {
                        master_config.force_start_nodes.push_back(static_cast<uint8_t>(node.as<int>()));
                    }
                }
            }))
            return false;

        if (!tryLoadConfig("drivers section", [&]() {
                for (const auto& item : config["drivers"]) {
                    drivers.push_back({
                        static_cast<uint8_t>(item["driver_id"].as<int>()),
                        config_path + item["eds_path"].as<std::string>(),
                        config_path + item["binary_path"].as<std::string>(),
                        std::chrono::milliseconds(item["sdo_timeout"].as<uint32_t>(100)),
                    });
                }
            }))
            return false;

        if (!tryLoadConfig("external_encoders section", [&]() {
                for (const auto& item : config["external_encoders"]) {
                    external_encoders.push_back(
                        {static_cast<uint8_t>(item["encoder_id"].as<int>(0)), static_cast<uint8_t>(item["driver_id"].as<int>(0))});
                }
            }))
            return false;

        if (!tryLoadConfig("motors section", [&]() {
                for (const auto& item : config["motors"]) {
                    motors.push_back(
                        {static_cast<uint8_t>(item["motor_id"].as<int>(0)), static_cast<uint8_t>(item["driver_id"].as<int>(0)),
                         item["motor_driver"].as<std::string>(), static_cast<uint8_t>(item["multiaxis_index"].as<int>(0)),
                         static_cast<uint8_t>(item["mode_of_operation"].as<int>(3)),
                         std::chrono::milliseconds(item["handle_interval"].as<uint32_t>(1)), item["ratio"].as<double>(),
                         item["ppr"].as<int>(), item["enable_first"].as<bool>(true), item["use_pid"].as<bool>(false),
                         item["kp"].as<double>(0.0), item["ki"].as<double>(0.0), item["kd"].as<double>(0.0),
                         item["use_external_encoder"].as<bool>(false), static_cast<uint8_t>(item["encoder_id"].as<int>(0))});
                }
            }))
            return false;

        if (!tryLoadConfig("motor_extra_info section", [&]() {
                if (!config["motor_extra_info"])
                    return;

                for (const auto& item : config["motor_extra_info"]) {
                    MotorExtraInfo extra;
                    extra.motor_id = static_cast<uint8_t>(item["motor_id"].as<int>());

                    if (item["voltage"]) {
                        extra.voltage.index = static_cast<uint16_t>(item["voltage"]["index"].as<int>());
                        extra.voltage.subindex = static_cast<uint8_t>(item["voltage"]["subindex"].as<int>());
                        extra.voltage.multiplier = item["voltage"]["multiplier"].as<double>(1.0);
                        extra.voltage.type = item["voltage"]["type"].as<std::string>();
                    }

                    if (item["current"]) {
                        extra.current.index = static_cast<uint16_t>(item["current"]["index"].as<int>());
                        extra.current.subindex = static_cast<uint8_t>(item["current"]["subindex"].as<int>());
                        extra.current.multiplier = item["current"]["multiplier"].as<double>(1.0);
                        extra.current.type = item["current"]["type"].as<std::string>();
                    }

                    if (item["error"]) {
                        extra.error.index = static_cast<uint16_t>(item["error"]["index"].as<int>());
                        extra.error.subindex = static_cast<uint8_t>(item["error"]["subindex"].as<int>());
                        extra.error.type = item["error"]["type"].as<std::string>();
                        extra.error.sto = static_cast<uint16_t>(item["error"]["sto"].as<int>(0));
                        if (item["error"]["codes"]) {
                            for (const auto& code : item["error"]["codes"]) {
                                extra.error.codes.push_back(
                                    {static_cast<uint16_t>(code["value"].as<int>()), code["message"].as<std::string>()});
                            }
                        }
                    }

                    motor_extras.push_back(extra);
                }
            }))
            return false;

        printLoadedConfig(config_path, master_config, drivers, external_encoders, motors, motor_extras);
        return true;
    }

    static void printConfiguration(
        const std::string& config_path, const MasterConfig& master_config, const std::vector<DriverConfig>& drivers,
        const std::vector<ExternalEncoderConfig>& external_encoders, const std::vector<MotorConfig>& motors,
        const std::vector<MotorExtraInfo>& motor_extras)
    {
        printLoadedConfig(config_path, master_config, drivers, external_encoders, motors, motor_extras);
    }

private:
    static void printLoadedConfig(
        const std::string& config_path, const MasterConfig& master_config, const std::vector<DriverConfig>& drivers,
        const std::vector<ExternalEncoderConfig>& external_encoders, const std::vector<MotorConfig>& motors,
        const std::vector<MotorExtraInfo>& motor_extras)
    {
        NLOG(info) << "=== Configuration Loaded Successfully ===";
        NLOG(info) << "Configuration path: " << config_path;

        printMasterConfig(master_config);
        printDriverConfigs(drivers);
        printEncoderConfigs(external_encoders);
        printMotorConfigs(motors);
        printMotorExtraInfo(motor_extras);

        NLOG(info) << "=== Summary ===";
        NLOG(info) << "Total loaded: " << drivers.size() << " drivers, " << external_encoders.size() << " encoders, " << motors.size()
                   << " motors, " << motor_extras.size() << " motor extras";
    }

    static void printMasterConfig(const MasterConfig& master_config)
    {
        NLOG(info) << "[Master Configuration]";
        NLOG(info) << "  Node ID: " << static_cast<int>(master_config.master_id);
        NLOG(info) << "  Interface Name: " << master_config.interface_name;
        NLOG(info) << "  Master DCF txt: " << master_config.dcf_txt;
        NLOG(info) << "  Master DCF bin: " << master_config.dcf_bin;
        if (!master_config.fake_bootup_nodes.empty()) {
            NLOG(info) << "  Fake Bootup Nodes: ";
            for (const auto& node : master_config.fake_bootup_nodes) {
                NLOG(info) << "    Node ID " << static_cast<int>(node);
            }
        }
        if (!master_config.force_start_nodes.empty()) {
            NLOG(info) << "  Force Start Nodes: ";
            for (const auto& node : master_config.force_start_nodes) {
                NLOG(info) << "    Node ID " << static_cast<int>(node);
            }
        }
    }

    static void printDriverConfigs(const std::vector<DriverConfig>& drivers)
    {
        NLOG(info) << "[Driver Configurations] (Total: " << drivers.size() << ")";
        for (const auto& driver : drivers) {
            NLOG(info) << "  Driver ID " << static_cast<int>(driver.driver_id) << ":";
            NLOG(info) << "    EDS Path: " << driver.eds_path;
            NLOG(info) << "    Binary Path: " << driver.binary_path;
            NLOG(info) << "    SDO Timeout: " << driver.sdo_timeout.count() << " ms";
        }
    }
    static void printEncoderConfigs(const std::vector<ExternalEncoderConfig>& encoders)
    {
        NLOG(info) << "[External Encoder Configurations] (Total: " << encoders.size() << ")";
        for (const auto& encoder : encoders) {
            NLOG(info) << "  Encoder ID " << static_cast<int>(encoder.encoder_id) << " -> Driver ID "
                       << static_cast<int>(encoder.driver_id);
        }
    }

    static void printMotorConfigs(const std::vector<MotorConfig>& motors)
    {
        NLOG(info) << "[Motor Configurations] (Total: " << motors.size() << ")";
        for (const auto& motor : motors) {
            NLOG(info) << "  Motor ID " << static_cast<int>(motor.motor_id) << ":";
            NLOG(info) << "    Driver ID: " << static_cast<int>(motor.driver_id);
            NLOG(info) << "    Motor Driver: " << motor.motor_driver;
            NLOG(info) << "    MuitlAxis Index: " << static_cast<int>(motor.multiaxis_index);
            NLOG(info) << "    Mode Of Operation: " << static_cast<int>(motor.mode_of_operation);
            NLOG(info) << "    Handle Interval: " << motor.handle_interval.count() << " ms";
            NLOG(info) << "    Ratio: " << motor.ratio;
            NLOG(info) << "    PPR: " << motor.ppr;
            NLOG(info) << "    Enable First: " << (motor.enable_first ? "Yes" : "No");
            NLOG(info) << "    Use PID: " << (motor.use_pid ? "Yes" : "No");
            if (motor.use_pid) {
                NLOG(info) << "      Kp: " << motor.kp << ", Ki: " << motor.ki << ", Kd: " << motor.kd;
            }
            NLOG(info) << "    Use External Encoder: " << (motor.use_external_encoder ? "Yes" : "No");
            if (motor.use_external_encoder) {
                NLOG(info) << "      Encoder ID: " << static_cast<int>(motor.encoder_id);
            }
        }
    }

    static void printMotorExtraInfo(const std::vector<MotorExtraInfo>& motor_extras)
    {
        NLOG(info) << "[Motor Extra Information] (Total: " << motor_extras.size() << ")";
        for (const auto& extra : motor_extras) {
            NLOG(info) << "  Motor ID " << static_cast<int>(extra.motor_id) << ":";

            // Voltage info
            NLOG(info) << "    Voltage:";
            NLOG(info) << "      Index: 0x" << std::hex << extra.voltage.index << std::dec;
            NLOG(info) << "      Subindex: 0x" << std::hex << static_cast<int>(extra.voltage.subindex) << std::dec;
            NLOG(info) << "      Multiplier: " << extra.voltage.multiplier;

            // Current info
            NLOG(info) << "    Current:";
            NLOG(info) << "      Index: 0x" << std::hex << extra.current.index << std::dec;
            NLOG(info) << "      Subindex: 0x" << std::hex << static_cast<int>(extra.current.subindex) << std::dec;
            NLOG(info) << "      Multiplier: " << extra.current.multiplier;

            // Error info
            NLOG(info) << "    Error:";
            NLOG(info) << "      Index: 0x" << std::hex << extra.error.index << std::dec;
            NLOG(info) << "      Subindex: 0x" << std::hex << static_cast<int>(extra.error.subindex) << std::dec;
            NLOG(info) << "      STO Code: 0x" << std::hex << extra.error.sto << std::dec;
            NLOG(info) << "      Error Codes: " << extra.error.codes.size() << " defined";

            for (const auto& code : extra.error.codes) {
                NLOG(info) << "        0x" << std::hex << code.value << std::dec << ": " << code.message;
            }
        }
    }
};

}  // namespace NaviFra

#endif