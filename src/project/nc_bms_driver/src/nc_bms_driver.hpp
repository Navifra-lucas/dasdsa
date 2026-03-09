#ifndef NAVIFRA_BMSDRIVER_HPP_
#define NAVIFRA_BMSDRIVER_HPP_

#include "interface/can/can_driver_base.hpp"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "core/util/logger.hpp"
#include "core_msgs/BmsInfo.h"
#include "core_msgs/NavicoreStatus.h"
#include <boost/any.hpp>
#include <std_msgs/UInt8.h>
#include <std_msgs/UInt32.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Int64.h>
#include "geometry_msgs/Twist.h"
#include <thread>
#include <mutex>

using namespace std;
namespace NaviFra {

const std::map<int, std::string> charger_error_map_ = {
    {1, "충전모듈 전원 인가용 MC(마그네트) 불량 - MC ON시 보조접점에서 신호 미입력 상태"},
    {2, "충전전류 미감지"},
    {3, "충전모듈 내 DC Link 전압 불량"},
    {4, "충전용 릴레이 불량 : 릴레이 ON 지령시 보조접점이 ON 상태로 유지"},
    {11, "상태 이상"},
    {12, "충전 전압 이상"},
    {13, "충전 전류 이상"},
    {14, "충전 중 목표 전압 초과"},
    {15, "충전 중 목표 전류 초과"},
    {16, "충전 중 전류 출력 차단됨"},
    {17, "온도 발열 이상"},
    {18, "충전 모듈 상태 이상"}
};
// 유작 남기고 갑니다 -피터-

enum CAN_BD
{
    NC_CAN_BAUD_125K = 796,
    NC_CAN_BAUD_250K = 284,
    NC_CAN_BAUD_500K = 28,
    NC_CAN_BAUD_800K = 22,
    NC_CAN_BAUD_1M = 20,
};

// BmsError 정의 (enum)
enum BmsErrorFlag : uint32_t {
    BMS_ERR_PACK_OV            = 1 << 0,
    BMS_ERR_PACK_UV            = 1 << 1,
    BMS_ERR_CELL_OV            = 1 << 2,
    BMS_ERR_CELL_UV            = 1 << 3,
    BMS_ERR_SOC_OV             = 1 << 4,
    BMS_ERR_SOC_UV             = 1 << 5,
    BMS_ERR_OT                 = 1 << 6,
    BMS_ERR_UT                 = 1 << 7,
    BMS_ERR_VOLT_DEV           = 1 << 8,
    BMS_ERR_TEMP_DEV           = 1 << 9,
    BMS_ERR_OC                 = 1 << 10,
    BMS_ERR_RELAY              = 1 << 11,
    BMS_ERR_FUSE               = 1 << 12,
    BMS_ERR_RELAY_FUSE         = 1 << 13,
    BMS_ERR_EEPROM             = 1 << 14,
    BMS_ERR_PRECHARGE          = 1 << 15,
    BMS_ERR_AFIC               = 1 << 16,
    BMS_ERR_CT_MISMATCH        = 1 << 17,
    BMS_ERR_PVS_MISMATCH       = 1 << 18,
    BMS_ERR_CT_FAIL            = 1 << 19,
    BMS_ERR_PVS_FAIL           = 1 << 20,
    BMS_ERR_TEMP_SENSOR        = 1 << 21,
    BMS_ERR_CELL_VOLT_SENSOR   = 1 << 22,
    BMS_ERR_CT_ZS_FAIL         = 1 << 23,
    BMS_ERR_PVS_ZS_FAIL        = 1 << 24,
};

// 전압, 전류, SOC 정보
union VoltageInfo {
    struct {
        uint16_t un16_cum_volt; // 누적 전체 전압   
        uint16_t un16_gather_volt; // 집합 전압
        uint16_t un16_current;
        uint16_t un16_soc;
    };
    uint8_t byte_space[8];
};

// 최대, 최소 셀 전압 정보
union CellVoltageInfo {
    struct {
        uint8_t un8_max_cell_vt_high;
        uint8_t un8_max_cell_vt_low;
        uint8_t un8_max_no; // 최대 전압 셀 번호
        uint8_t un8_min_cell_vt_high;
        uint8_t un8_min_cell_vt_low;
        uint8_t un8_min_no; // 최소 전압 셀 번호
    };
    uint8_t byte_space[6];
};

// 최대, 최소 셀 온도 정보
union CellTemperatureInfo {
    struct {
        uint8_t un8_max_cell_temp;
        uint8_t un8_max_cell_no;
        uint8_t un8_min_cell_temp;
        uint8_t un8_min_cell_no;
    };
    uint8_t byte_space[4];
};

// MOS 상태 및 잔여 용량
union MosAndCapacity {
    struct {
        uint8_t un8_state; // 동작 상태 - 1: 충전 , 2: 방전
        uint8_t un8_charge_mos; // 충전 MOS 상태
        uint8_t un8_discharge_mos; // 방전 MOS 상태
        uint8_t un8_bms_life; // BMS 수명
        uint32_t un32_remain_cap; // 잔여 용량
    };
    uint8_t byte_space[8];
};

class BmsDriver {
public:
    BmsDriver(ros::NodeHandle& nh, ros::NodeHandle& nhp);
    ~BmsDriver();

    void RegistTalker();
    void RegistListener();
    void HeartBeatLoop();
    void CanCallback(const boost::any& any_type_var);
    void SleepCmdCallback(const std_msgs::UInt8::ConstPtr& msg);
    void UpdateCmdCallback(const std_msgs::UInt32::ConstPtr& msg);
    void LogExtractCmdCallback(const std_msgs::Empty::ConstPtr& msg);
    void outputCmdCallback(const std_msgs::String::ConstPtr& msg);
    void NavifraInfoCallback(const core_msgs::NavicoreStatus::ConstPtr& msg);
    void CmdCallback(const geometry_msgs::Twist::ConstPtr& msg);

    bool ManualMoveAutoChargeFlag();

private:
    ros::NodeHandle nh_;
    ros::NodeHandle nhp_;

    int n_can_bus_bms = 0x37;
    int n_can_baud_bms = 0x284;
    int n_charge_fail_count_ = 0;
    
    float f_delta_v_ = 0.0;
    float f_avg_temp_ = 0.0;
    float f_resume_percent_ = 75.0f; 
    float f_stop_percent_ = 79.0f;
    float f_linear_x_ = 0.0;
    float f_linear_y_ = 0.0;
    float f_target_linear_x_ = 0.01;

    ros::Publisher pub_bms_;
    ros::Publisher error_pub_;
    ros::Publisher pub_output_cmd_;
    ros::Subscriber sub_sleep_cmd_;
    ros::Subscriber sub_bms_update_cmd_;
    ros::Subscriber sub_bms_log_extract_cmd_;
    ros::Subscriber sub_output_cmd_;
    ros::Subscriber sub_navifra_info_;
    ros::Subscriber sub_cmd_vel_;

    core_msgs::BmsInfo o_bms_info_;

    std::thread th_heartbeat_;
    Can_driver o_can_driver_;

    std::string s_robot_status_ = "";

    bool b_terminate_ = false;
    bool b_bms_error = false;
    bool b_charge_cmd_ = false;
    bool b_error_check_ = false;
    bool b_auto_resume_pending_ = false;
    bool b_manaul_flag_ = false;
    std::chrono::steady_clock::time_point tp_last_log_;
    std_msgs::Int64 s_error_;

    std::mutex mtx_info_;
};
};  // namespace NaviFra
// #else  // peak can usb 모듈이 없는 경우
// class BmsDriver {
// public:
//     BmsDriver(ros::NodeHandle& nh, ros::NodeHandle& nhp) { NLOG(error) << "can not build"; };
//     ~BmsDriver(){};
// };
// #endif
#endif
