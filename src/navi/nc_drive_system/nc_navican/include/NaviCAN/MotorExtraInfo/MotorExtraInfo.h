// MotorExtraInfo.h
#ifndef NAVIFRA_NAVICAN_MOTOR_EXTRA_INFO_H
#define NAVIFRA_NAVICAN_MOTOR_EXTRA_INFO_H

#include "NaviCAN/MotorExtraInfo/interface/IMotorExtraInfo.h"
#include "NaviCAN/MotorDriver/MotorDriver.h"
#include "NaviCAN/NaviCANDriver.h"
#include "NaviCAN/Utils/CANopenTypeTraits.h"
#include "NaviCAN/Utils/ConfigLoader.h"
#include "util/logger.hpp"

#include <vector>
#include <memory>
#include <sstream>
#include <algorithm>

namespace NaviFra
{

class MotorExtraInfo : public IMotorExtraInfo
{
public:
    explicit MotorExtraInfo(const ConfigLoader::MotorExtraInfo& extra_info, 
                           std::shared_ptr<IMotorDriverBase> motor_driver)
        : motor_driver_(motor_driver) {

        voltage_index_ = extra_info.voltage.index;
        voltage_subindex_ = extra_info.voltage.subindex;
        voltage_multiplier_ = extra_info.voltage.multiplier;
        voltage_type_ = extra_info.voltage.type;
        
        current_index_ = extra_info.current.index;
        current_subindex_ = extra_info.current.subindex;
        current_multiplier_ = extra_info.current.multiplier;
        current_type_ = extra_info.current.type;
            
        error_index_ = extra_info.error.index;
        error_subindex_ = extra_info.error.subindex;
        error_type_ = extra_info.error.type;
        error_sto_ = extra_info.error.sto;
        error_codes_ = extra_info.error.codes;
    }

    double getVoltage() override {
        return universalGetValue(voltage_index_, voltage_subindex_, voltage_type_, voltage_multiplier_);
    }

    double getCurrent() override {      
        return universalGetValue(current_index_, current_subindex_, current_type_, current_multiplier_);
    }

    uint32_t getErrorCode() override {
        return  static_cast<uint32_t>(universalGetValue(error_index_, error_subindex_, error_type_, 1.0));
    }

    std::string getErrorMessage() override {
        auto error_code = getErrorCode();
        
        std::stringstream ss;
        bool first = true;
        
        for (const auto& code : error_codes_) {
            if (static_cast<uint16_t>(error_code) == code.value) {
                if (!first) {
                    ss << ", ";
                }
                ss << code.message;
                first = false;
            }
        }
        
        if (first) {
            ss << "The ErrorCode is 0x" << std::hex << error_code;
        }
        
        return ss.str();
    }

    uint16_t getSTOCode() override {
        return error_sto_;
    }

 private:
    std::shared_ptr<IMotorDriverBase> motor_driver_;
    
    // Voltage info
    uint16_t voltage_index_ = 0;
    uint8_t voltage_subindex_ = 0;
    double voltage_multiplier_ = 1.0;
    std::string voltage_type_;
    
    // Current info
    uint16_t current_index_ = 0;
    uint8_t current_subindex_ = 0;
    double current_multiplier_ = 1.0;
    std::string current_type_;
    
    // Error info
    uint16_t error_index_ = 0;
    uint8_t error_subindex_ = 0;
    std::string error_type_;
    uint16_t error_sto_ = 0;
    std::vector<ConfigLoader::ErrorCode> error_codes_;
    
    /**
     * @brief 타입별로 값을 읽는 헬퍼 템플릿 함수
     */
    template<typename T>
    double getValueTyped(uint16_t index, uint8_t subindex, double multiplier) {
        return motor_driver_->getRpdoMapped<T>(index, subindex) * multiplier;
    }

    /**
     * @brief 타입 이름 문자열에 따라 적절한 타입으로 값을 읽음
     *
     * CANopenTypeHelper를 사용하여 타입 유효성을 검증하고,
     * 컴파일 타임 타입 디스패치를 통해 런타임 오버헤드를 최소화합니다.
     */
    double universalGetValue(uint16_t index, uint8_t subindex, const std::string& type, double multiplier) {
        using Helper = NaviCAN::Utils::CANopenTypeHelper;

        if (!Helper::isValidTypeName(type)) {
            NLOG(severity_level::warning) << "Unknown type: " << type;
            return 0.0;
        }

        try {
            // 타입 문자열에 따른 디스패치
            if (type == "uint8_t") {
                return getValueTyped<uint8_t>(index, subindex, multiplier);
            } else if (type == "uint16_t") {
                return getValueTyped<uint16_t>(index, subindex, multiplier);
            } else if (type == "uint32_t") {
                return getValueTyped<uint32_t>(index, subindex, multiplier);
            } else if (type == "int8_t") {
                return getValueTyped<int8_t>(index, subindex, multiplier);
            } else if (type == "int16_t") {
                return getValueTyped<int16_t>(index, subindex, multiplier);
            } else if (type == "int32_t") {
                return getValueTyped<int32_t>(index, subindex, multiplier);
            } else if (type == "float") {
                return getValueTyped<float>(index, subindex, multiplier);
            } else if (type == "double") {
                return getValueTyped<double>(index, subindex, multiplier);
            } else {
                // 이미 isValidTypeName에서 체크했으므로 여기 도달하지 않음
                return 0.0;
            }
        } catch (const std::exception& e) {
            NLOG(severity_level::error) << "Failed to read RPDO mapped value (index=0x"
                                        << std::hex << index << ", subindex=" << std::dec
                                        << (int)subindex << ", type=" << type << "): " << e.what();
            return 0.0;
        }
    }
};

} // namespace NaviFra

#endif