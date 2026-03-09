#ifndef NC_IO_CONTROLLER_H
#define NC_IO_CONTROLLER_H

#include "core_msgs/NaviAlarm.h"
#include "core_msgs/PLCInfo.h"
#include "core_msgs/BmsInfo.h"
#include "motor_msgs/MotorCmd.h"
#include "nc_io_manager/controller/nc_sequence_controller.h"
#include "nc_io_manager/data/data_handler.h"
#include "nc_io_manager/plc/plc_communicator.h"
#include "pos/pos.hpp"

#include <core_msgs/ARPoseList.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int64.h>
#include <std_msgs/String.h>
#include <task_msgs/LiftInfo.h>
#include <task_msgs/Loading.h>

#include <algorithm>
#include <cctype>
#include <chrono>
#include <map>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

namespace NaviFra {
class IOController {
public:
    enum class ConnectionState
    {
        CONNECTED,  // PLC 클라이언트가 연결됨
        LISTENING,  // 서버가 클라이언트 연결을 대기 중
        RECONNECTING,  // 서버 재시작 시도 중
        FAILED  // 서버 시작 실패
    };

public:
    IOController(ros::NodeHandle nh);
    virtual ~IOController();

private:
    ros::NodeHandle nh_;
    std::shared_ptr<DataHandler> data_handler_;
    std::shared_ptr<PLCCommunicator> plc_comm_;
    std::shared_ptr<SequenceController> sequence_controller_;

    // 스레드 관리
    std::thread worker_thread_;
    bool running_;

    // ROS 인터페이스
    ros::Subscriber output_cmd_sub_;
    ros::Subscriber charge_cmd_sub_;
    ros::Subscriber lift_control_sub_;
    ros::Subscriber brake_cmd_sub_;
    ros::Subscriber emergency_sub_;
    ros::Subscriber undocking_status_sub_;
    ros::Subscriber sub_navi_alarm_;
    ros::Subscriber ar_pose_sub_;
    ros::Subscriber bms_info_sub_;
    ros::Subscriber cmd_vel_sub_;

    ros::Publisher cmd_pub_;
    ros::Publisher brake_pub_;
    ros::Publisher lift_status_pub_;
    ros::Publisher error_pub_;
    ros::Publisher navi_alarm_pub_;
    ros::Publisher sequence_complete_pub_;
    ros::Publisher plc_info_pub_;
    ros::Publisher charge_state_pub_;
    ros::ServiceClient motor_cmd_req_;

    // 설정 파라미터 (ROS 파라미터에서 로드) - 서버 설정으로 변경
    std::string server_bind_ip_;  // 서버 바인드 IP (기존: plc_ip_)
    int server_port_;  // 서버 포트 (기존: plc_port_)
    int max_retry_count_;  // 서버 시작 재시도 횟수
    double retry_delay_seconds_;  // 재시도 지연 시간
    double reconnect_interval_seconds_;  // 서버 재시작 간격
    int max_consecutive_failures_;  // 연속 통신 실패 허용 횟수
    int communication_interval_ms_;  // 고정 100ms
    std_msgs::Int64 s_error_;

    // 연결 관리
    ConnectionState connection_state_;
    int current_retry_count_;

    // 시퀀스 관리
    std::string last_sequnce_name_;
    SequenceState current_sequnce_state_;
    SequenceState prev_sequnce_state_ = SequenceState::IDLE;

    std::chrono::steady_clock::time_point last_plc_info_pub_{};

    std::atomic<bool> undock_up_check_pending_{false};
    std::chrono::steady_clock::time_point undock_up_check_start_tp_;
    std::chrono::steady_clock::time_point pin_up_3s_start_tp_;
    double undock_up_check_delay_sec_{3.0};  // 파라미터로 변경 가능
    bool b_last_pin_up_command_ = false;
    bool b_ar_detect_ = false;

    NaviFra::Pos o_robot_pos_;
    std::mutex mtx_pos_;

    float f_linear_x_;
    float f_pack_current_ = 0.0;

public:
    void initialize();
    void run();
    void stop();

    void publishPlcInfoOnce_();  // 기존 plc_info_send 시퀀스 동작
    void scanAndPublishPinErrorsOnce_();  // 기존 pin_error_monitor 시퀀스 동작

    // 상태 조회
    ConnectionState getConnectionState() const;
    int getCurrentRetryCount() const;
    std::string getConnectionStateString() const;

    // 설정값 조회 (서버 설정으로 변경)
    std::string getServerBindIP() const;
    int getServerPort() const;
    int getCommunicationInterval() const;

    // 추가: 서버 상태 조회
    std::string getServerState() const;  // 서버 상태 (STOPPED, LISTENING, CONNECTED, ERROR)
    std::string getConnectedClientIP() const;  // 연결된 클라이언트 IP

    bool b_charge_flag = false;

    // 에러 내용 0.5초 뒤에 보낼거임 -피  터-
    void publishErrorWithDelay(const std_msgs::Int64& error_msg, double delay_sec);
    void checkAndPublishDelayedErrors();

    bool b_error_pending_{false};
    bool b_error_first_published_{false};

    Poco::Timestamp tp_error_start_;
    std_msgs::Int64 pending_error_msg_;

    double wait_seconds_{0.0}; 

private:
    // 파라미터 로드
    void loadParameters();

    // 초기화 관련
    bool initializeIODevice();
    void setupROSInterface();
    void initializeSequence();

    // 서버 관리
    bool startServerWithRetry(const std::string& bind_ip, int port);
    bool handleServerRestart();

    // 메인 루프
    void workerLoop();
    bool performIOCommunication();
    bool isStateChanged(SequenceState current, SequenceState previous);

    // ROS 콜백 함수들
    void outputCommandCallback(const std_msgs::String::ConstPtr& msg);
    void liftControlCallback(const task_msgs::Loading::ConstPtr& msg);
    void RecvBrakeCmd(const std_msgs::Bool::ConstPtr& msg);
    void RecvEmergency(const std_msgs::Bool::ConstPtr& msg);
    void RecvUndockingStatus(const std_msgs::String::ConstPtr& msg);
    void processUndockUpCheck_();
    void NaviAlarmCallback(const core_msgs::NaviAlarm::ConstPtr& msg);
    void RecvRobotPose(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg);
    void CmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg);
    void RecvARPose(const core_msgs::ARPoseList::ConstPtr& msg);
    void RecvBmsInfo(const core_msgs::BmsInfo::ConstPtr& msg);

    // 유틸리티 함수들
    void parseAndExecuteCommand(const std::string& raw_command);

private:
    // 시퀀스 등록
    void RegisterPinSequence();
    void RegisterChargingSequence();
    void RegisterSafetySequence();
    void RegisterControllerSequence();

    // pin sequence
    void PinInitSequence();
    void PinDownSequence();
    void PinUpSequence();
    void Pin3secondAfterUpSequence();

    // charge sequence
    void ChargingStartSequence();
    void ChargingStopSequence();
    void ChargingResetSequence();
    void ChargingResetFalseSequence();
    void ChargingStopBmsSequence();

    // safety sequence
    void BrakeReleaseOnSequence();
    void BrakeReleaseOffSequence();
    void QuickStopOnSequence();
    void QuickStopOffSequence();
    void StoCheckSequence();
    void StoClearOnSequence();
    void StoClearOffSequence();
    void BumperClearOffSequence();
    void BumperClearOnSequence();

    // control sequence
    void ManualModeOnSequence();
    void ManualModeOffSequence();
    void EquipmentSendSequence();
    void OssdByPassOnSequence();
    void OssdByPassOffSequence();
    void LidarOssdEnableSequence();
    void LidarOssdDisableSequence();
    void PlcInfoSequence();
};
}  // namespace NaviFra

#endif  // NC_IO_CONTROLLER_H