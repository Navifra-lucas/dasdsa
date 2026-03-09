#ifndef NAVIFRA_MOTOR_COMMANDER_ERROR_CHECKER_H
#define NAVIFRA_MOTOR_COMMANDER_ERROR_CHECKER_H

#include "data_struct.hpp"

#include <any>
#include <chrono>
#include <functional>

namespace NaviFra {
namespace MotorCommander {

template <MotorId TraitMotorId>
class ErrorCheck {
private:
    using Traits = MotorTraits<TraitMotorId>;
    static constexpr float TIMEOUT_THRESHOLD = 0.4f;
    static constexpr auto TO_NS = 1e9;

public:
    // Fault - TRACTION MOTOR, STEERING MOTOR, UPPER TURNTABLE
    template <bool HasFault = !Traits::is_abs_encoder>
    [[nodiscard]] static inline typename std::enable_if<HasFault, bool>::type Fault(
        const motor_msgs::MotorData& data, NaviFra::Motor_ERROR& error, std::function<void(const std::string&, const std::any&)>& notify)
    {
        if (data.is_error == true) {
            Traits::setError(error, Traits::fault_code, Traits::fault_text);
            notify("error_callback", error);
            return true;
        }
        Traits::setError(error, 0, "");
        return false;
    }

    // Fault - ABS_ENCODER (항상 false)
    template <bool HasFault = !Traits::is_abs_encoder>
    [[nodiscard]] static inline typename std::enable_if<!HasFault, bool>::type Fault(
        const motor_msgs::MotorData& data, NaviFra::Motor_ERROR& error, std::function<void(const std::string&, const std::any&)>& notify)
    {
        return false;
    }

    // Timeout - TRACTION MOTOR, STEERING MOTOR, ABS_ENCODER, UPPER TURNTABLE
    [[nodiscard]] static inline bool Timeout(
        const motor_msgs::MotorData& data, NaviFra::Motor_ERROR& error, std::function<void(const std::string&, const std::any&)>& notify)
    {
        constexpr int64_t threshold_ns = static_cast<int64_t>(TIMEOUT_THRESHOLD * TO_NS);

        auto last_update_ns = static_cast<int64_t>(data.last_update_time * TO_NS);
        auto now_ns = std::chrono::steady_clock::now().time_since_epoch().count();
        auto diff_ns = now_ns - last_update_ns;

        if (diff_ns < threshold_ns) {
            Traits::setError(error, 0, "");
            return false;
        }
        Traits::setError(error, Traits::timeout_code, Traits::timeout_text);
        notify("error_callback", error);
        return true;
    }

    // Stall - TRACTION MOTOR
    template <bool HasStall = Traits::is_traction>
    [[nodiscard]] static inline typename std::enable_if<HasStall, bool>::type Stall(
        const motor_msgs::MotorData& data, NaviFra::Motor_ERROR& error, std::function<void(const std::string&, const std::any&)>& notify,
        std::chrono::system_clock::time_point& stall_start_time, std::chrono::system_clock::time_point now)
    {
        if (std::abs(data.input_velocity) > 30.0f && std::abs(data.feedback_velocity) < 2.0f) {
            if (stall_start_time == std::chrono::system_clock::time_point()) {
                stall_start_time = now;
            }

            auto duration = std::chrono::duration<float>(now - stall_start_time).count();
            if (duration < 5.0f) {
                Traits::setError(error, 0, "");
                return false;
            }
            Traits::setError(error, Traits::stall_code, Traits::stall_text);
            notify("error_callback", error);
            return true;
        }

        stall_start_time = std::chrono::system_clock::time_point();  // Reset
        return false;
    }

    // Stall - STEERING MOTOR, ABS_ENCODER, UPPER TURNTABLE (항상 false)
    template <bool HasStall = Traits::is_traction>
    [[nodiscard]] static inline typename std::enable_if<!HasStall, bool>::type Stall(
        const motor_msgs::MotorData& data, NaviFra::Motor_ERROR& error, std::function<void(const std::string&, const std::any&)>& notify,
        std::chrono::system_clock::time_point& stall_start_time, std::chrono::system_clock::time_point now)
    {
        return false;
    }

    // OverCurrent - TRACTION MOTOR
    template <bool HasOverCurrent = (Traits::is_traction || Traits::is_steering)>
    [[nodiscard]] static inline typename std::enable_if<HasOverCurrent && Traits::is_traction, bool>::type OverCurrent(
        const motor_msgs::MotorData& data, NaviFra::Motor_ERROR& error, std::function<void(const std::string&, const std::any&)>& notify,
        const DriverParameters_t& params, std::chrono::system_clock::time_point& latest_normal_time,
        std::chrono::system_clock::time_point now)
    {
        const float amp = params.st_traction_param.f_traction_overcurrent_amp_std;
        const float time = params.st_traction_param.f_traction_overcurrent_time_std;

        if (data.current < amp) {
            latest_normal_time = now;
            return false;
        }

        auto duration = std::chrono::duration<float>(now - latest_normal_time).count();
        if (duration < time) {
            Traits::setError(error, 0, "");
            return false;
        }
        Traits::setError(error, Traits::overcurrent_code, Traits::overcurrent_text);
        notify("error_callback", error);
        return true;
    }

    // OverCurrent - STEERING MOTOR
    template <bool HasOverCurrent = (Traits::is_traction || Traits::is_steering || Traits::is_upper_turntable)>
    [[nodiscard]] static inline typename std::enable_if<HasOverCurrent && Traits::is_steering, bool>::type OverCurrent(
        const motor_msgs::MotorData& data, NaviFra::Motor_ERROR& error, std::function<void(const std::string&, const std::any&)>& notify,
        const DriverParameters_t& params, std::chrono::system_clock::time_point& latest_normal_time,
        std::chrono::system_clock::time_point now)
    {
        const float amp = params.st_steer_param.f_steer_overcurrent_amp_std;
        const float time = params.st_steer_param.f_steer_overcurrent_time_std;

        if (data.current < amp) {
            latest_normal_time = now;
            return false;
        }

        auto duration = std::chrono::duration<float>(now - latest_normal_time).count();
        if (duration < time) {
            Traits::setError(error, 0, "");
            return false;
        }

        Traits::setError(error, Traits::overcurrent_code, Traits::overcurrent_text);
        notify("error_callback", error);
        return true;
    }

    // OverCurrent - ABS_ENCODER (항상 false)
    template <bool HasOverCurrent = (Traits::is_traction || Traits::is_steering || Traits::is_upper_turntable)>
    [[nodiscard]] static inline typename std::enable_if<!HasOverCurrent, bool>::type OverCurrent(
        const motor_msgs::MotorData& data, NaviFra::Motor_ERROR& error, std::function<void(const std::string&, const std::any&)>& notify,
        const DriverParameters_t& params, std::chrono::system_clock::time_point& latest_normal_time,
        std::chrono::system_clock::time_point now)
    {
        return false;
    }

    // OverCurrent - UPPER TURNTABLE
    template <bool HasOverCurrent = (Traits::is_traction || Traits::is_steering || Traits::is_upper_turntable)>
    [[nodiscard]] static inline typename std::enable_if<HasOverCurrent && Traits::is_upper_turntable, bool>::type OverCurrent(
        const motor_msgs::MotorData& data, NaviFra::Motor_ERROR& error, std::function<void(const std::string&, const std::any&)>& notify,
        const DriverParameters_t& params, std::chrono::system_clock::time_point& latest_normal_time,
        std::chrono::system_clock::time_point now)
    {
        const float amp = params.st_upper_param.f_turntable_overcurrent_amp_std;
        const float time = params.st_upper_param.f_turntable_overcurrent_time_std;

        if (data.current < amp) {
            latest_normal_time = now;
            return false;
        }

        auto duration = std::chrono::duration<float>(now - latest_normal_time).count();
        if (duration < time) {
            Traits::setError(error, 0, "");
            return false;
        }
        Traits::setError(error, Traits::overcurrent_code, Traits::overcurrent_text);
        notify("error_callback", error);
        return true;
    }

    // SafetyStop - TRACTION MOTOR, STEERING MOTOR, UPPER TURNTABLE
    template <bool HasSafetyStop = !Traits::is_abs_encoder>
    [[nodiscard]] static inline typename std::enable_if<HasSafetyStop, bool>::type SafetyStop(
        const motor_msgs::MotorData& data, NaviFra::Motor_ERROR& error, std::function<void(const std::string&, const std::any&)>& notify)
    {
        if ((data.is_error == true) && (data.error_code == data.sto_code)) {
            Traits::setError(error, Traits::safety_stop_code, Traits::safety_stop_text);
            notify("error_callback", error);
            return true;
        }

        Traits::setError(error, 0, "");
        return false;
    }

    // SafetyStop - ABS_ENCODER (항상 false)
    template <bool HasSafetyStop = !Traits::is_abs_encoder>
    [[nodiscard]] static inline typename std::enable_if<!HasSafetyStop, bool>::type SafetyStop(
        const motor_msgs::MotorData& data, NaviFra::Motor_ERROR& error, std::function<void(const std::string&, const std::any&)>& notify)
    {
        return false;
    }
};

class ErrorChecker {
public:
    ErrorChecker(std::function<void(const std::string&, const std::any&)> notify)
        : notify_(notify)
    {
    }

    [[nodiscard]] bool checkErrorsCustom(const MotorDataMap& motor_data, const DriverParameters_t& params)
    {
        const auto now = std::chrono::system_clock::now();

        using AllMotors = MotorCheckDispatcher<
            MOTOR_TRACTION_FL, MOTOR_TRACTION_FR, MOTOR_TRACTION_RL, MOTOR_TRACTION_RR, MOTOR_STEER_FL, MOTOR_STEER_FR, MOTOR_STEER_RL,
            MOTOR_STEER_RR, MOTOR_ABS_ENCODER_FL, MOTOR_ABS_ENCODER_FR, MOTOR_ABS_ENCODER_RL, MOTOR_ABS_ENCODER_RR>;

        for (const auto& [motor_id, data] : motor_data) {
            if (AllMotors::check<MOTOR_TRACTION_FL, MOTOR_TRACTION_RR>(motor_id, data, error_, notify_, params, current_time_, stall_time_, now)) {
                return true;
            }
        }

        return false;
    }

    [[nodiscard]] bool checkErrorsWithType(const MotorDataMap& motor_data, const DriverParameters_t& params)
    {
        const auto now = std::chrono::system_clock::now();

        using TractionMotors = MotorCheckDispatcher<MOTOR_TRACTION_FL, MOTOR_TRACTION_FR, MOTOR_TRACTION_RL, MOTOR_TRACTION_RR>;

        using SteerMotors = MotorCheckDispatcher<MOTOR_STEER_FL, MOTOR_STEER_FR, MOTOR_STEER_RL, MOTOR_STEER_RR>;

        using AbsEncoders = MotorCheckDispatcher<MOTOR_ABS_ENCODER_FL, MOTOR_ABS_ENCODER_FR, MOTOR_ABS_ENCODER_RL, MOTOR_ABS_ENCODER_RR>;

        using TurnTables = MotorCheckDispatcher<MOTOR_UPPER_TURNTABLE>;

        bool hasError = false;

        for (const auto& [motor_id, data] : motor_data) {
            switch (motor_id) {
                case MOTOR_TRACTION_FL:
                case MOTOR_TRACTION_FR:
                case MOTOR_TRACTION_RL:
                case MOTOR_TRACTION_RR:
                    hasError = TractionMotors::checkSingle(motor_id, data, error_, notify_, params, current_time_, stall_time_, now);
                    break;
                case MOTOR_STEER_FL:
                case MOTOR_STEER_FR:
                case MOTOR_STEER_RL:
                case MOTOR_STEER_RR:
                    hasError = SteerMotors::checkSingle(motor_id, data, error_, notify_, params, current_time_, stall_time_, now);
                    break;
                case MOTOR_ABS_ENCODER_FL:
                case MOTOR_ABS_ENCODER_FR:
                case MOTOR_ABS_ENCODER_RL:
                case MOTOR_ABS_ENCODER_RR:
                    hasError = AbsEncoders::checkSingle(motor_id, data, error_, notify_, params, current_time_, stall_time_, now);
                    break;
                case MOTOR_UPPER_TURNTABLE:
                    hasError = TurnTables::checkSingle(motor_id, data, error_, notify_, params, current_time_, stall_time_, now);
                    break;
                default:
                    // unknown motor type
                    continue;
            }

            if (hasError) {
                return true;
            }
        }

        return false;
    }

    NaviFra::Motor_ERROR getError() const { return error_; }

private:
    std::map<MotorId, std::chrono::system_clock::time_point> current_time_{};
    std::map<MotorId, std::chrono::system_clock::time_point> stall_time_{};
    NaviFra::Motor_ERROR error_{};
    std::function<void(const std::string&, const std::any&)> notify_;

    template <MotorId... MotorIds>
    struct MotorCheckDispatcher {
        [[nodiscard]] static inline bool checkSingle(
            MotorId motor_id,

            const motor_msgs::MotorData& data, NaviFra::Motor_ERROR& error,
            std::function<void(const std::string&, const std::any&)>& notify,

            const DriverParameters_t& params, std::map<MotorId, std::chrono::system_clock::time_point>& current_time,
            std::map<MotorId, std::chrono::system_clock::time_point>& stall_time, std::chrono::system_clock::time_point now)
        {
            return check<MotorIds...>(motor_id, data, error, notify, params, current_time, stall_time, now);
        }

        template <MotorId First, MotorId... Rest>
        [[nodiscard]] static inline bool check(
            MotorId motor_id,

            const motor_msgs::MotorData& data, NaviFra::Motor_ERROR& error,
            std::function<void(const std::string&, const std::any&)>& notify,

            const DriverParameters_t& params, std::map<MotorId, std::chrono::system_clock::time_point>& current_time,
            std::map<MotorId, std::chrono::system_clock::time_point>& stall_time, std::chrono::system_clock::time_point now)
        {
            if (motor_id == First) {
                using Check = ErrorCheck<First>;

                return Check::SafetyStop(data, error, notify) || Check::Fault(data, error, notify) || Check::Timeout(data, error, notify) ||
                    Check::OverCurrent(data, error, notify, params, current_time[First], now) || Check::Stall(data, error, notify, stall_time[First], now);
            }

            if constexpr (sizeof...(Rest) > 0) {
                return check<Rest...>(motor_id, data, error, notify, params, current_time, stall_time, now);
            }

            return false;
        }
    };
};

}  // namespace MotorCommander
}  // namespace NaviFra

#endif