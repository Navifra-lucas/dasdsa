#include "core_astra/initializer/motor_status_initializer.h"

#include "core_agent/data/memory_repository.h"
#include "core_astra/zmq_handler.h"
#include "motor_status.pb.h"
#include "util/logger.hpp"

using namespace NaviFra;

// 저장에 사용할 키
static constexpr const char* KEY_MOTOR_DATA = "MotorStatusList";

static inline void fillMotor(core_astra::MotorStatus* out, const motor_msgs::MotorData& in)
{
    out->set_motor_id(in.motor_id);
    out->set_status(in.status);
    out->set_error_code(in.error_code);
    out->set_voltage(in.voltage);
    out->set_current(in.current);
    out->set_target_velocity(in.input_velocity);
    out->set_feedback_velocity(in.feedback_velocity);
    out->set_target_angle(in.input_angle);
    out->set_feedback_angle(in.feedback_angle);
    out->set_encoder(in.encoder);
    out->set_accumulated_encoder(in.acc_encoder);
    // out->set_fault_flag(in.);
    out->set_bus_state(in.bus_state);
    out->set_last_update_time(in.last_update_time);
}

MotorStatusInitializer::~MotorStatusInitializer()
{
    shutdown();
}

void MotorStatusInitializer::initialize()
{
    ros::NodeHandle nh;

    // 파라미터로 퍼블리시 주기 조정 가능
    nh.param("motor_status_publish_hz", publish_hz_, 20.0);

    motor_data_subscriber_ =
        nh.subscribe("motor_data/info", 5, &MotorStatusInitializer::recvMotorData, this, ros::TransportHints().tcpNoDelay(true));

    running_ = true;
    pub_thread_ = std::thread(&MotorStatusInitializer::publishLoop, this);

    LOG_INFO("MotorStatusInitializer completed initialization");
}

void MotorStatusInitializer::finalize()
{
    shutdown();
}

void MotorStatusInitializer::recvMotorData(const motor_msgs::MotorDataInfo::ConstPtr& msg)
{
    std::lock_guard<std::mutex> lock(motor_status_mutex_);
    motor_status_list_.clear_motors();

    for (const auto& m : msg->data) {
        auto* o = motor_status_list_.add_motors();
        o->set_motor_id(m.motor_id);
        o->set_status(m.status);
        o->set_error_code(m.error_code);
        o->set_voltage(m.voltage);
        o->set_current(m.current);
        o->set_target_velocity(m.input_velocity);
        o->set_feedback_velocity(m.feedback_velocity);
        o->set_target_angle(m.input_angle);
        o->set_feedback_angle(m.feedback_angle);
        o->set_encoder(m.encoder);
        o->set_accumulated_encoder(m.acc_encoder);
        // o->set_fault_flag(m.fault_flag);
        o->set_bus_state(m.bus_state);
        o->set_last_update_time(m.last_update_time);
    }
}

void MotorStatusInitializer::shutdown()
{
    running_ = false;
    if (pub_thread_.joinable())
        pub_thread_.join();

    motor_data_subscriber_.shutdown();
}

void MotorStatusInitializer::publishLoop()
{
    using namespace std::chrono;
    const auto period = duration<double>(1.0 / std::max(1.0, publish_hz_));

    auto& zmq = ZMQHandler::instance();
    // 필요 시 토픽명 파라미터화
    const std::string topic = "motor_status";

    while (running_) {
        auto t0 = steady_clock::now();

        std::string buffer;
        {
            std::lock_guard<std::mutex> lock(motor_status_mutex_);
            buffer = motor_status_list_.SerializeAsString();
        }

        if (!buffer.empty()) {
            zmq.send(topic, buffer);
        }

        // 주기 유지
        auto elapsed = steady_clock::now() - t0;
        auto sleep_dur = period - elapsed;
        if (sleep_dur > duration<double>::zero())
            std::this_thread::sleep_for(sleep_dur);
    }
}