#include "core/util/logger.hpp"
#include "core_msgs/LidarContamination.h"
#include "core_msgs/LidarInfoMsg.h"
#include "core_msgs/NaviAlarm.h"
#include "ros/publisher.h"
#include "ros/ros.h"
#include "ros/subscriber.h"
#include "sensor_msgs/LaserScan.h"

#include <iostream>
#include <string>
#include <thread>
#include <vector>

using namespace std;

enum LIDAR_INDEX
{
    FRONT = 0,
    REAR,
    LEFT,
    RIGHT
};

struct Lidar_Info_t {
    bool b_lidar_use = false;
    string s_lidar_frame_id = "";
    int n_lidar_error_code = 0;
    string s_lidar_error_text = "";
    int n_lidar_point_size = 0;
    bool b_lidar_contamination = false;
    std::chrono::steady_clock::time_point tp_lidar_timestamp;
};
class nc_lidar_info_publisher {
private:
    /* data */
public:
    nc_lidar_info_publisher(ros::NodeHandle& nh, ros::NodeHandle& nhp);
    ~nc_lidar_info_publisher();

    ros::NodeHandle nh_;
    ros::NodeHandle nhp_;
    ros::Publisher pub_lidar_info_;

    float f_lidar_timeout_threshold_;
    bool b_not_use_localization_ = false;
    bool b_lidar_clean_check_use_ = false;
    bool b_use_lidar_info_ = false;
    int n_max_lidar_num_;
    string s_target_frame_id;

    vector<string> vec_input_topics_;
    vector<string> vec_raw_datas_;
    vector<ros::Subscriber> vec_lidar_subscriber_;
    vector<ros::Subscriber> vec_raw_data_subscriber_;

    std::vector<Lidar_Info_t> vec_lidar_info_;
    vector<vector<float>> vec_f_dist_;
    std::vector<std::vector<std::chrono::steady_clock::time_point>> vec_tp_lidar_timestamp_;

    std::thread th_lidar_info_pub_;
    std::mutex mutex_get_data_;
    bool thread_run_ = true;

    void GetFrontLidar(const sensor_msgs::LaserScanConstPtr scan);
    void GetRearLidar(const sensor_msgs::LaserScanConstPtr scan);
    void GetLeftLidar(const sensor_msgs::LaserScanConstPtr scan);
    void GetRightLidar(const sensor_msgs::LaserScanConstPtr scan);

    void GetFrontRawData(const core_msgs::LidarContamination raw_data);
    void GetRearRawData(const core_msgs::LidarContamination raw_data);
    void GetLeftRawData(const core_msgs::LidarContamination raw_data);
    void GetRightRawData(const core_msgs::LidarContamination raw_data);

    void PublishLidarInfo();
    float GetHz(std::vector<std::chrono::steady_clock::time_point>& vec_time);
};

nc_lidar_info_publisher::nc_lidar_info_publisher(ros::NodeHandle& nh, ros::NodeHandle& nhp)
    : nh_(nh)
    , nhp_(nhp)
{
    ros::param::get("lidarmerger/input_scans", vec_input_topics_);

    ros::param::param<float>("lidarmerger/lidar_timeout_threshold_", f_lidar_timeout_threshold_, 3.0);
    ros::param::get("lidarmerger/max_lidar_num", n_max_lidar_num_);
    ros::param::param<string>("lidarmerger/destination_frame", s_target_frame_id, std::string("/base_link"));
    ros::param::param<bool>("lidarmerger/b_lidar_clean_check_use", b_lidar_clean_check_use_, false);
    ros::param::param<bool>("lidarmerger/b_use_lidar_info", b_use_lidar_info_, false);

    pub_lidar_info_ = nh_.advertise<core_msgs::LidarInfoMsg>("lidar_info", 5);
    vec_lidar_info_.resize(vec_input_topics_.size());
    vec_tp_lidar_timestamp_.resize(vec_input_topics_.size());
    vec_f_dist_.resize(vec_input_topics_.size(), vector<float>(9, 0));
    for (int i = 0; i < vec_input_topics_.size(); i++) {
        if (i == LIDAR_INDEX::FRONT) {
            vec_lidar_subscriber_.emplace_back(nh_.subscribe(
                vec_input_topics_[LIDAR_INDEX::FRONT], 1, &nc_lidar_info_publisher::GetFrontLidar, this,
                ros::TransportHints().tcpNoDelay(true)));
        }
        if (i == LIDAR_INDEX::REAR) {
            vec_lidar_subscriber_.emplace_back(nh_.subscribe(
                vec_input_topics_[LIDAR_INDEX::REAR], 1, &nc_lidar_info_publisher::GetRearLidar, this,
                ros::TransportHints().tcpNoDelay(true)));
        }
        if (i == LIDAR_INDEX::LEFT) {
            vec_lidar_subscriber_.emplace_back(nh_.subscribe(
                vec_input_topics_[LIDAR_INDEX::LEFT], 1, &nc_lidar_info_publisher::GetLeftLidar, this,
                ros::TransportHints().tcpNoDelay(true)));
        }
        if (i == LIDAR_INDEX::RIGHT) {
            vec_lidar_subscriber_.emplace_back(nh_.subscribe(
                vec_input_topics_[LIDAR_INDEX::RIGHT], 1, &nc_lidar_info_publisher::GetRightLidar, this,
                ros::TransportHints().tcpNoDelay(true)));
        }
        LOG_INFO("lidar info pub %s", vec_input_topics_[i].c_str());
        vec_lidar_info_[i].tp_lidar_timestamp = std::chrono::steady_clock::now();
    }

    ros::param::get("lidarmerger/raw_datas", vec_raw_datas_);
    if (vec_raw_datas_.size() > 0) {
        LOG_INFO("raw data subs");
        for (int i = 0; i < vec_raw_datas_.size(); i++) {
            if (i == LIDAR_INDEX::FRONT) {
                vec_raw_data_subscriber_.emplace_back(nh_.subscribe(
                    vec_raw_datas_[LIDAR_INDEX::FRONT], 1, &nc_lidar_info_publisher::GetFrontRawData, this,
                    ros::TransportHints().tcpNoDelay(true)));
            }
            if (i == LIDAR_INDEX::REAR) {
                vec_raw_data_subscriber_.emplace_back(nh_.subscribe(
                    vec_raw_datas_[LIDAR_INDEX::REAR], 1, &nc_lidar_info_publisher::GetRearRawData, this,
                    ros::TransportHints().tcpNoDelay(true)));
            }
            if (i == LIDAR_INDEX::LEFT) {
                vec_raw_data_subscriber_.emplace_back(nh_.subscribe(
                    vec_raw_datas_[LIDAR_INDEX::LEFT], 1, &nc_lidar_info_publisher::GetLeftRawData, this,
                    ros::TransportHints().tcpNoDelay(true)));
            }
            if (i == LIDAR_INDEX::RIGHT) {
                vec_raw_data_subscriber_.emplace_back(nh_.subscribe(
                    vec_raw_datas_[LIDAR_INDEX::RIGHT], 1, &nc_lidar_info_publisher::GetRightRawData, this,
                    ros::TransportHints().tcpNoDelay(true)));
            }
        }
    }
    else {
        LOG_INFO("raw data param not set");
    }

    LOG_INFO("info publisher initialized");
    th_lidar_info_pub_ = std::thread(&nc_lidar_info_publisher::PublishLidarInfo, this);
}

nc_lidar_info_publisher::~nc_lidar_info_publisher()
{
    thread_run_ = false;
}

float nc_lidar_info_publisher::GetHz(std::vector<std::chrono::steady_clock::time_point>& vec_time)
{
    float f_sec_sum = 0;
    int n_sec_cnt = 0;
    if (vec_time.size() == 0)
        return 0;
    for (int i = 1; i < vec_time.size(); i++) {
        std::chrono::duration<double> sec = vec_time[i] - vec_time[i - 1];
        f_sec_sum += sec.count();
        n_sec_cnt++;
    }
    if (n_sec_cnt == 0 || f_sec_sum == 0) {
        return 0;
    }
    return float(1 / (f_sec_sum / n_sec_cnt));
}
void nc_lidar_info_publisher::GetFrontLidar(const sensor_msgs::LaserScanConstPtr scan)
{
    mutex_get_data_.lock();
    vec_lidar_info_[LIDAR_INDEX::FRONT].s_lidar_frame_id = s_target_frame_id;
    vec_lidar_info_[LIDAR_INDEX::FRONT].n_lidar_point_size = scan->ranges.size();
    vec_lidar_info_[LIDAR_INDEX::FRONT].tp_lidar_timestamp = std::chrono::steady_clock::now();
    vec_tp_lidar_timestamp_[LIDAR_INDEX::FRONT].emplace_back(vec_lidar_info_[LIDAR_INDEX::FRONT].tp_lidar_timestamp);

    if (vec_tp_lidar_timestamp_[LIDAR_INDEX::FRONT].size() > 10)
        vec_tp_lidar_timestamp_[LIDAR_INDEX::FRONT].erase(vec_tp_lidar_timestamp_[LIDAR_INDEX::FRONT].begin());

    mutex_get_data_.unlock();
}
void nc_lidar_info_publisher::GetRearLidar(const sensor_msgs::LaserScanConstPtr scan)
{
    mutex_get_data_.lock();
    vec_lidar_info_[LIDAR_INDEX::REAR].s_lidar_frame_id = s_target_frame_id;
    vec_lidar_info_[LIDAR_INDEX::REAR].n_lidar_point_size = scan->ranges.size();
    vec_lidar_info_[LIDAR_INDEX::REAR].tp_lidar_timestamp = std::chrono::steady_clock::now();
    vec_tp_lidar_timestamp_[LIDAR_INDEX::REAR].emplace_back(vec_lidar_info_[LIDAR_INDEX::REAR].tp_lidar_timestamp);

    if (vec_tp_lidar_timestamp_[LIDAR_INDEX::REAR].size() > 10)
        vec_tp_lidar_timestamp_[LIDAR_INDEX::REAR].erase(vec_tp_lidar_timestamp_[LIDAR_INDEX::REAR].begin());

    mutex_get_data_.unlock();
}
void nc_lidar_info_publisher::GetLeftLidar(const sensor_msgs::LaserScanConstPtr scan)
{
    mutex_get_data_.lock();
    vec_lidar_info_[LIDAR_INDEX::LEFT].s_lidar_frame_id = s_target_frame_id;
    vec_lidar_info_[LIDAR_INDEX::LEFT].n_lidar_point_size = scan->ranges.size();
    vec_lidar_info_[LIDAR_INDEX::LEFT].tp_lidar_timestamp = std::chrono::steady_clock::now();
    vec_tp_lidar_timestamp_[LIDAR_INDEX::LEFT].emplace_back(vec_lidar_info_[LIDAR_INDEX::LEFT].tp_lidar_timestamp);

    if (vec_tp_lidar_timestamp_[LIDAR_INDEX::LEFT].size() > 10)
        vec_tp_lidar_timestamp_[LIDAR_INDEX::LEFT].erase(vec_tp_lidar_timestamp_[LIDAR_INDEX::LEFT].begin());

    mutex_get_data_.unlock();
}
void nc_lidar_info_publisher::GetRightLidar(const sensor_msgs::LaserScanConstPtr scan)
{
    mutex_get_data_.lock();
    vec_lidar_info_[LIDAR_INDEX::RIGHT].s_lidar_frame_id = s_target_frame_id;
    vec_lidar_info_[LIDAR_INDEX::RIGHT].n_lidar_point_size = scan->ranges.size();
    vec_lidar_info_[LIDAR_INDEX::RIGHT].tp_lidar_timestamp = std::chrono::steady_clock::now();
    vec_tp_lidar_timestamp_[LIDAR_INDEX::RIGHT].emplace_back(vec_lidar_info_[LIDAR_INDEX::RIGHT].tp_lidar_timestamp);

    if (vec_tp_lidar_timestamp_[LIDAR_INDEX::RIGHT].size() > 10)
        vec_tp_lidar_timestamp_[LIDAR_INDEX::RIGHT].erase(vec_tp_lidar_timestamp_[LIDAR_INDEX::RIGHT].begin());

    mutex_get_data_.unlock();
}

void nc_lidar_info_publisher::GetFrontRawData(const core_msgs::LidarContamination raw_data)
{
    bool b_raw_data_conatamination_error = raw_data.b_contamination;
    if (b_raw_data_conatamination_error == vec_lidar_info_[LIDAR_INDEX::FRONT].b_lidar_contamination)
        return;

    mutex_get_data_.lock();
    vec_lidar_info_[LIDAR_INDEX::FRONT].b_lidar_contamination = b_raw_data_conatamination_error;
    mutex_get_data_.unlock();
}
void nc_lidar_info_publisher::GetRearRawData(const core_msgs::LidarContamination raw_data)
{
    bool b_raw_data_conatamination_error = raw_data.b_contamination;
    if (b_raw_data_conatamination_error == vec_lidar_info_[LIDAR_INDEX::REAR].b_lidar_contamination)
        return;

    mutex_get_data_.lock();
    vec_lidar_info_[LIDAR_INDEX::REAR].b_lidar_contamination = b_raw_data_conatamination_error;
    mutex_get_data_.unlock();
}
void nc_lidar_info_publisher::GetLeftRawData(const core_msgs::LidarContamination raw_data)
{
    bool b_raw_data_conatamination_error = raw_data.b_contamination;
    if (b_raw_data_conatamination_error == vec_lidar_info_[LIDAR_INDEX::LEFT].b_lidar_contamination)
        return;

    mutex_get_data_.lock();
    vec_lidar_info_[LIDAR_INDEX::LEFT].b_lidar_contamination = b_raw_data_conatamination_error;
    mutex_get_data_.unlock();
}
void nc_lidar_info_publisher::GetRightRawData(const core_msgs::LidarContamination raw_data)
{
    bool b_raw_data_conatamination_error = raw_data.b_contamination;
    if (b_raw_data_conatamination_error == vec_lidar_info_[LIDAR_INDEX::RIGHT].b_lidar_contamination)
        return;

    mutex_get_data_.lock();
    vec_lidar_info_[LIDAR_INDEX::RIGHT].b_lidar_contamination = b_raw_data_conatamination_error;
    mutex_get_data_.unlock();
}

void nc_lidar_info_publisher::PublishLidarInfo()
{
    core_msgs::LidarInfoMsg lidar_info_msg;
    core_msgs::LidarDistList lidar_dist_msg;

    std::vector<Lidar_Info_t> vec_lidar_info;
    vec_lidar_info.resize(vec_input_topics_.size());

    int n_lidar_clean_front = 0;
    int n_lidar_clean_rear = 0;
    int n_lidar_clean_left = 0;
    int n_lidar_clean_right = 0;

    int n_clean_alarm_count = 100;
    // if (b_not_use_localization_ == true)
    //     return;
    int n_count_error = 0;
    while (thread_run_) {
        for (int scan_id = 0; scan_id < vec_input_topics_.size(); scan_id++) {
            mutex_get_data_.lock();
            auto lidar_time = vec_lidar_info_[scan_id].tp_lidar_timestamp;
            auto frame_id = vec_lidar_info_[scan_id].s_lidar_frame_id;
            auto point_size = vec_lidar_info_[scan_id].n_lidar_point_size;
            bool b_lidar_contamination = vec_lidar_info_[scan_id].b_lidar_contamination;

            vector<vector<float>> vec_f_dist = vec_f_dist_;
            std::vector<std::vector<std::chrono::steady_clock::time_point>> vec_tp_lidar_timestamp = vec_tp_lidar_timestamp_;
            mutex_get_data_.unlock();
            std::chrono::duration<double> lidar_scan_time_gap = std::chrono::steady_clock::now() - lidar_time;
            float f_lidar_scan_time_gap = lidar_scan_time_gap.count();

            if (b_lidar_contamination == true && b_lidar_clean_check_use_) {
                if (scan_id == LIDAR_INDEX::FRONT)
                    n_lidar_clean_front++;
                if (scan_id == LIDAR_INDEX::REAR)
                    n_lidar_clean_rear++;
                if (scan_id == LIDAR_INDEX::LEFT)
                    n_lidar_clean_left++;
                if (scan_id == LIDAR_INDEX::RIGHT)
                    n_lidar_clean_right++;
            }
            else if (b_lidar_contamination == false && b_lidar_clean_check_use_) {
                if (scan_id == LIDAR_INDEX::FRONT)
                    n_lidar_clean_front = 0;
                if (scan_id == LIDAR_INDEX::REAR)
                    n_lidar_clean_rear = 0;
                if (scan_id == LIDAR_INDEX::LEFT)
                    n_lidar_clean_left = 0;
                if (scan_id == LIDAR_INDEX::RIGHT)
                    n_lidar_clean_right = 0;
            }

            if (f_lidar_scan_time_gap > f_lidar_timeout_threshold_) {
                n_count_error++;
                if (n_count_error % 10 == 0)
                    LOG_WARNING("SIGNAL_TIMEOUT !!!! : %f,  %f", f_lidar_scan_time_gap, f_lidar_timeout_threshold_);

                if (scan_id == LIDAR_INDEX::FRONT) {
                    vec_lidar_info[scan_id].n_lidar_error_code = core_msgs::NaviAlarm::ERROR_FRONT_LIDAR_SIGNAL_TIMEOUT;
                    vec_lidar_info[scan_id].s_lidar_error_text = "ERROR_FRONT_LIDAR_SIGNAL_TIMEOUT";
                }
                if (scan_id == LIDAR_INDEX::REAR) {
                    vec_lidar_info[scan_id].n_lidar_error_code = core_msgs::NaviAlarm::ERROR_REAR_LIDAR_SIGNAL_TIMEOUT;
                    vec_lidar_info[scan_id].s_lidar_error_text = "ERROR_REAR_LIDAR_SIGNAL_TIMEOUT";
                }
                if (scan_id == LIDAR_INDEX::LEFT) {
                    vec_lidar_info[scan_id].n_lidar_error_code = core_msgs::NaviAlarm::ERROR_LEFT_LIDAR_SIGNAL_TIMEOUT;
                    vec_lidar_info[scan_id].s_lidar_error_text = "ERROR_LEFT_LIDAR_SIGNAL_TIMEOUT";
                }
                if (scan_id == LIDAR_INDEX::RIGHT) {
                    vec_lidar_info[scan_id].n_lidar_error_code = core_msgs::NaviAlarm::ERROR_RIGHT_LIDAR_SIGNAL_TIMEOUT;
                    vec_lidar_info[scan_id].s_lidar_error_text = "ERROR_RIGHT_LIDAR_SIGNAL_TIMEOUT";
                }

                mutex_get_data_.lock();
                vec_tp_lidar_timestamp_[scan_id].clear();
                mutex_get_data_.unlock();

                vec_tp_lidar_timestamp[scan_id].clear();
            }
            else if (
                n_lidar_clean_front >= n_clean_alarm_count || n_lidar_clean_rear >= n_clean_alarm_count ||
                n_lidar_clean_left >= n_clean_alarm_count || n_lidar_clean_right >= n_clean_alarm_count) {
                if (scan_id == LIDAR_INDEX::FRONT) {
                    vec_lidar_info[scan_id].n_lidar_error_code = core_msgs::NaviAlarm::ERROR_FRONT_LIDAR_CLEAN;
                    vec_lidar_info[scan_id].s_lidar_error_text = "ERROR_FRONT_LIDAR_CLEAN";
                }
                if (scan_id == LIDAR_INDEX::REAR) {
                    vec_lidar_info[scan_id].n_lidar_error_code = core_msgs::NaviAlarm::ERROR_REAR_LIDAR_CLEAN;
                    vec_lidar_info[scan_id].s_lidar_error_text = "ERROR_REAR_LIDAR_CLEAN";
                }
                if (scan_id == LIDAR_INDEX::LEFT) {
                    vec_lidar_info[scan_id].n_lidar_error_code = core_msgs::NaviAlarm::ERROR_LEFT_LIDAR_CLEAN;
                    vec_lidar_info[scan_id].s_lidar_error_text = "ERROR_LEFT_LIDAR_CLEAN";
                }
                if (scan_id == LIDAR_INDEX::RIGHT) {
                    vec_lidar_info[scan_id].n_lidar_error_code = core_msgs::NaviAlarm::ERROR_RIGHT_LIDAR_CLEAN;
                    vec_lidar_info[scan_id].s_lidar_error_text = "ERROR_RIGHT_LIDAR_CLEAN";
                }
            }
            else {
                vec_lidar_info[scan_id].n_lidar_error_code = 0;
                vec_lidar_info[scan_id].s_lidar_error_text = "";
            }

            switch (scan_id) {
                case LIDAR_INDEX::FRONT:
                {
                    lidar_info_msg.front_lidar_frame_id = frame_id;
                    lidar_info_msg.front_lidar_error_code = vec_lidar_info[scan_id].n_lidar_error_code;
                    lidar_info_msg.front_lidar_error_text = vec_lidar_info[scan_id].s_lidar_error_text;
                    lidar_info_msg.front_lidar_frame_hz = GetHz(vec_tp_lidar_timestamp[scan_id]);
                    lidar_info_msg.front_lidar_range_length = point_size;
                    break;
                }
                case LIDAR_INDEX::REAR:
                {
                    lidar_info_msg.rear_lidar_frame_id = frame_id;
                    lidar_info_msg.rear_lidar_error_code = vec_lidar_info[scan_id].n_lidar_error_code;
                    lidar_info_msg.rear_lidar_error_text = vec_lidar_info[scan_id].s_lidar_error_text;
                    lidar_info_msg.rear_lidar_frame_hz = GetHz(vec_tp_lidar_timestamp[scan_id]);
                    lidar_info_msg.rear_lidar_range_length = point_size;
                    break;
                }
                case LIDAR_INDEX::LEFT:
                {
                    lidar_info_msg.left_lidar_frame_id = frame_id;
                    lidar_info_msg.left_lidar_error_code = vec_lidar_info[scan_id].n_lidar_error_code;
                    lidar_info_msg.left_lidar_error_text = vec_lidar_info[scan_id].s_lidar_error_text;
                    lidar_info_msg.left_lidar_frame_hz = GetHz(vec_tp_lidar_timestamp[scan_id]);
                    lidar_info_msg.left_lidar_range_length = point_size;
                    break;
                }
                case LIDAR_INDEX::RIGHT:
                {
                    lidar_info_msg.right_lidar_frame_id = frame_id;
                    lidar_info_msg.right_lidar_error_code = vec_lidar_info[scan_id].n_lidar_error_code;
                    lidar_info_msg.right_lidar_error_text = vec_lidar_info[scan_id].s_lidar_error_text;
                    lidar_info_msg.right_lidar_frame_hz = GetHz(vec_tp_lidar_timestamp[scan_id]);
                    lidar_info_msg.right_lidar_range_length = point_size;
                    break;
                }
            }
        }
        pub_lidar_info_.publish(lidar_info_msg);
        std::this_thread::sleep_for(50ms);
    }
}
int main(int argc, char** argv)
{
    ros::init(argc, argv, "lidar_info_publisher");
    ros::NodeHandle nh;
    ros::NodeHandle nhp("~");

    ros::AsyncSpinner spinner(0);
    spinner.start();
    nc_lidar_info_publisher lidardriver(nh, nhp);
    // ros::spin();
    ros::waitForShutdown();
    return 0;
}