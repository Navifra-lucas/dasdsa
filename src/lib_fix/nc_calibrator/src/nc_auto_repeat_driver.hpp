#ifndef AUTO_REPEAT_DRIVER_HPP
#define AUTO_REPEAT_DRIVER_HPP

#include "core/util/logger.hpp"
#include "core_msgs/Goal.h"
#include "core_msgs/JsonLinkList.h"
#include "core_msgs/JsonList.h"
#include "core_msgs/JsonNodeList.h"
#include "core_msgs/NavicoreStatus.h"
#include "core_msgs/PGVPoseList.h"
#include "core_msgs/RepeatTestMsg.h"
#include "nav_msgs/Odometry.h"
#include "std_msgs/String.h"

#include <Poco/Dynamic/Var.h>
#include <Poco/JSON/Object.h>
#include <Poco/JSON/Parser.h>
#include <boost/thread.hpp>
#include <core_msgs/CommonString.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <ros/ros.h>

#include <fstream>

namespace NaviFra {

struct RobotStatus  // 로봇 상태
{
    float f_robot_speed_;
    float f_x_error_m_;
    float f_y_error_m_;
    float f_deg_error_;

    std::string s_curr_node_;
    std::string s_next_node_;
    std::string s_goal_node_;

    std::string s_robot_status_;
    std::string s_cmd_;

    RobotStatus()
        : f_robot_speed_(0.0)
        , s_robot_status_("idle")
        , s_cmd_("idle")
    {
    }
};

struct QrError  // QR 오류
{
    float f_qr_x_error_m_;
    float f_qr_y_error_m_;
    float f_qr_deg_error_;

    std::string s_qr_tag_;

    QrError()
        : f_qr_x_error_m_(0.0)
        , f_qr_y_error_m_(0.0)
        , f_qr_deg_error_(0.0)
        , s_qr_tag_("")
    {
    }
};

struct RepeatTestInfo  //반복주행 정보
{
    int n_curr_repeat_;
    int n_target_idx_;
    int n_repeat_cnt_;

    float f_stop_sec_;


    RepeatTestInfo()
        : n_curr_repeat_(0)
        , n_target_idx_(0)
        , n_repeat_cnt_(0)
        , f_stop_sec_(0)
    {
    }
};

class RepeatDriver {
public:
    RepeatDriver(ros::NodeHandle& nh, ros::NodeHandle& nhp);
    virtual ~RepeatDriver();

private:
    ros::NodeHandle& nh_;
    ros::NodeHandle& nhp_;

    ros::Subscriber navi_status_sub_;
    ros::Subscriber qr_offset_sub_;
    ros::Subscriber repeat_test_sub_;
    ros::Subscriber repeat_test_sub_cmd_;
    ros::Subscriber node_link_sub_;
    ros::Subscriber odom_sub_;
    ros::Subscriber cmd_sub_;

    ros::Publisher cmd_goal_pub_;
    ros::Publisher init_pose_pub_;
    ros::Publisher log_pub_;
    ros::Publisher navi_cmd_pub_;

    std::mutex status_mtx_;

    RobotStatus robot_status_;
    QrError qr_error_;
    RepeatTestInfo repeat_info_;

    int n_error_check_cnt_ = 0;

    bool b_is_running_;
    bool b_node_select_ = false;
    bool b_start_once_ = false;

    bool b_talker_init_ = false;
    bool b_listener_init_ = false;

    double d_decel_time_ = ros::Time::now().toSec();
    float d_decel_time_result_ = 0.0;
    float d_decel_qr_result_ = 0.0;

    std::string s_curr_time_;

    geometry_msgs::PoseStamped qr_offset_;

    std::unordered_map<std::string, std::string> node_link_;
    std::vector<std::string> node_list_;
    std::vector<std::string> save_node_list_;
    std::vector<std::string> err_list_;
    std::vector<std::pair<std::string, double>> qr_time_list_;

    boost::thread th_;

private:
    void Initialize();
    void DateTime();
    void RegistTalker();
    void RegistListener();
    void SaveError();
    void ResetData();

    void NaviStatusCallback(const core_msgs::NavicoreStatus::ConstPtr& msg);
    void QrOffsetCallback(const core_msgs::PGVPoseList::ConstPtr& msg);
    void RepeatCallback(const core_msgs::RepeatTestMsg::ConstPtr& msg);
    void RepeatCmdCallback(const std_msgs::String::ConstPtr& msg);
    void NodeLinkCallback(const core_msgs::JsonList::ConstPtr& msg);
    void OdomCallback(const nav_msgs::Odometry::ConstPtr& msg);
    void CmdCallback(const std_msgs::String::ConstPtr& msg);

    void excuteTask(std::string target_node);
    void DoThreadProcess();
    ros::Time last_log_time_;

    std::string SetPrecision(float value, int len);
};
}  // namespace NaviFra
#endif