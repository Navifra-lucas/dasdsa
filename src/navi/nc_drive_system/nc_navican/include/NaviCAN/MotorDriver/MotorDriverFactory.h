#ifndef NAVIFRA_NAVICAN_MOTOR_DRIVER_FACTORY_H
#define NAVIFRA_NAVICAN_MOTOR_DRIVER_FACTORY_H

#include <memory>
#include <string>
#include <unordered_map>
#include <functional>
#include <chrono>
#include <mutex>

#include "NaviCAN/NaviCANDriver.h"
#include "NaviCAN/MotorDriver/interface/IMotorInfo.h"
#include "NaviCAN/MotorDriver/BaseMotorDriver.h"
#include "NaviCAN/MotorDriver/interface/IMotorDriverBase.h"

#include "util/logger.hpp"
namespace NaviFra
{

class MotorDriverFactory {
public:

    using MotorDriverPtr = std::shared_ptr<IMotorDriverBase>;
    using MotorDriverCreator = std::function<MotorDriverPtr(const BaseMotorDriverParams&)>;

    static MotorDriverFactory& getInstance() {
        static std::once_flag first_call;
        std::call_once(first_call, []() {
            NLOG(info) << "MotorDriverFactory instance created.";
        });

        static MotorDriverFactory instance;
        return instance;
    }

    template<typename DriverType, typename ParamsType = BaseMotorDriverParams>
    void registerMotorDriver(const std::string& name) {
        creators_[name] = [name](const BaseMotorDriverParams& base_params) -> MotorDriverPtr {
            // 다운캐스팅
            if constexpr (!std::is_same_v<ParamsType, BaseMotorDriverParams>) {
                auto* specific_params = dynamic_cast<const ParamsType*>(&base_params);
                if (!specific_params) {
                    throw std::runtime_error("Invalid parameter type for " + name);
                }
                return std::make_shared<DriverType>(*specific_params);
            } else {
                return std::make_shared<DriverType>(base_params);
            }
        };
    }

    MotorDriverPtr createMotorDriver(const std::string& name, const BaseMotorDriverParams& params) {

        auto it = creators_.find(name);
        if (it != creators_.end()) {
            return it->second(params);
        }
        return nullptr;
    }

private:
    std::unordered_map<std::string, MotorDriverCreator> creators_;
};

} // namespace NaviFra



#endif