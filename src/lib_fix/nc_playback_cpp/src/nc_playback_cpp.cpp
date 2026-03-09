#include <nc_playback_cpp.hpp>

using namespace std;
using namespace NaviFra;
namespace fs = std::filesystem;

using Poco::JSON::Object;
using namespace NaviBrain;
using namespace Poco::JSON;
using Poco::File;
using Poco::Path;
using Poco::Net::FilePartSource;
using Poco::Net::HTMLForm;
using Poco::Net::HTTPResponse;

ncPlayback::ncPlayback()
{
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    tp_start_time_ = std::chrono::steady_clock::now();
    s_home = std::getenv("HOME");
    s_dir_ = s_home + "/navifra_solution/navicore/configs/bag/";
    std::filesystem::path p(s_dir_);

    if (!std::filesystem::is_directory(p)) {
        NLOG(info) << "make directory ros/bag";
        std::filesystem::create_directory(p);
    }

    ros::param::param<int>("playback_cpp/n_lidar_skip_count", n_lidar_skip_count_, 2);
    ros::param::param<float>("playback_cpp/f_move_sec", f_move_sec_, 5);
    ros::param::param<float>("playback_cpp/f_change_sec", f_change_sec_, 2);
    ros::param::param<float>("playback_cpp/f_bag_size", f_bag_size_, 5000);
    ros::param::param<float>("playback_cpp/f_bag_onefile_size", f_bag_onefile_size_, 100);
    ros::param::param<bool>("playback_cpp/b_use", f_use_, true);

    navifrainfo_pub = node_handle_.advertise<core_msgs::NavicoreStatus>("/PLAYBACK/navifra/info", 1);
    motorinfo_pub = node_handle_.advertise<core_msgs::MotorInfo>("/PLAYBACK/motor_info", 1);
    scan_front_pub = node_handle_.advertise<sensor_msgs::PointCloud>("/PLAYBACK/front_cloud_global", 1);
    scan_rear_pub = node_handle_.advertise<sensor_msgs::PointCloud>("/PLAYBACK/rear_cloud_global", 1);
    scan_left_pub = node_handle_.advertise<sensor_msgs::PointCloud>("/PLAYBACK/left_cloud_global", 1);
    scan_right_pub = node_handle_.advertise<sensor_msgs::PointCloud>("/PLAYBACK/right_cloud_global", 1);
    scan_v2v_pub = node_handle_.advertise<sensor_msgs::PointCloud>("/PLAYBACK/v2v_cloud_global", 1);
    scan_camera_pub = node_handle_.advertise<sensor_msgs::PointCloud>("/PLAYBACK/camera_cloud_global", 1);
    global_path_pub = node_handle_.advertise<nav_msgs::Path>("/PLAYBACK/NaviFra/visualize/ui_global_path", 1);
    local_path_pub = node_handle_.advertise<nav_msgs::Path>("/PLAYBACK/NaviFra/visualize/ui_local_path", 1);
    robot_col_pub = node_handle_.advertise<geometry_msgs::PolygonStamped>("/PLAYBACK/NaviFra/visualize/robot_collision", 1);
    robot_col_pre_pub = node_handle_.advertise<geometry_msgs::PolygonStamped>("/PLAYBACK/NaviFra/visualize/robot_collision_predict", 1);
    robot_obs_pos_pub = node_handle_.advertise<geometry_msgs::PolygonStamped>("/PLAYBACK/NaviFra/visualize/ui_obstacle_pos", 1);
    
    robot_uimsg_pub = node_handle_.advertise<std_msgs::String>("/PLAYBACK/etc_custom_message", 1);

    time_pub = node_handle_.advertise<std_msgs::String>("/PLAYBACK/play_time", 1);

    playback_sub_ = node_handle_.subscribe("/playback_cmd", 10, &ncPlayback::RecvCmd, this);
    cmdvel_sub_ = node_handle_.subscribe("/cmd_vel", 10, &ncPlayback::RecvCmdVel, this);

    navifraalarm_sub_ = node_handle_.subscribe("/navifra/alarm", 1, &ncPlayback::RecvAlarm, this);
    navifrainfo_sub_ = node_handle_.subscribe("/navifra/info", 1, &ncPlayback::RecordInfo, this);
    motorinfo_sub_ = node_handle_.subscribe("/motor_info", 1, &ncPlayback::RecordMotorInfo, this);
    scan_front_sub_ = node_handle_.subscribe("/front_cloud_global", 1, &ncPlayback::RecordFront, this);
    scan_rear_sub_ = node_handle_.subscribe("/rear_cloud_global", 1, &ncPlayback::RecordRear, this);
    scan_left_sub_ = node_handle_.subscribe("/left_cloud_global", 1, &ncPlayback::RecordLeft, this);
    scan_right_sub_ = node_handle_.subscribe("/right_cloud_global", 1, &ncPlayback::RecordRight, this);
    scan_v2v_sub_ = node_handle_.subscribe("/v2v_cloud_global", 1, &ncPlayback::RecordV2V, this);
    scan_camera_sub_ = node_handle_.subscribe("/camera_cloud_global", 1, &ncPlayback::RecordCamera, this);
    global_path_sub_ = node_handle_.subscribe("/NaviFra/visualize/ui_global_path", 1, &ncPlayback::RecordGPath, this);
    local_path_sub_ = node_handle_.subscribe("/NaviFra/visualize/ui_local_path", 1, &ncPlayback::RecordLPath, this);
    robot_col_sub = node_handle_.subscribe("/NaviFra/visualize/robot_collision", 1, &ncPlayback::RecordC, this);
    robot_col_pre_sub = node_handle_.subscribe("/NaviFra/visualize/robot_collision_predict", 1, &ncPlayback::RecordCP, this);
    robot_obs_pos_sub = node_handle_.subscribe("/NaviFra/visualize/ui_obstacle_pos", 1, &ncPlayback::RecordOP, this);
    robot_uimsg_sub = node_handle_.subscribe("/etc_custom_message", 1, &ncPlayback::RecordMSG, this);

    // rosbag::compression::LZ4
    th_record_ = std::thread(&ncPlayback::RecordLoop, this);
    th_play_ = std::thread(&ncPlayback::PlayLoop, this);
    th_download_ = std::thread(&ncPlayback::DownloadLoop, this);
}

ncPlayback::~ncPlayback()
{
    b_terminate_ = true;
    th_record_.join();
    th_play_.join();
    th_download_.join();
    LOG_INFO("destructor");

    if (s_dir_record_.size() > 0) {
        std::lock_guard<std::mutex> lock(mtx_record_);
        bag_record_.close();
        string s_cmd = "mv " + s_dir_record_ + ".active " + s_dir_record_ + ".bag";
        int n_result = std::system(s_cmd.c_str());
        NLOG(info) << "mv n_result " << n_result;
        s_dir_record_ = "";
    }
    if (bag_play_.isOpen()) {
        bag_play_.close();
        delete bag_view_;
    }
}

string ncPlayback::MakeFileName()
{
    auto now = std::chrono::system_clock::now();
    std::time_t t = std::chrono::system_clock::to_time_t(now);
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count();

    std::stringstream ss;
    ss << std::put_time(std::localtime(&t), "%F_%T");
    return ss.str().substr(2);
}

void ncPlayback::FileCheck()
{
    vector<string> vec_path;
    for (const auto& file : std::filesystem::directory_iterator(s_dir_)) {
        vec_path.emplace_back(file.path());
    }
    sort(vec_path.begin(), vec_path.end(), std::greater<>());

    float f_totla_mb = 0;

    for (int i = 0; i < vec_path.size(); i++) {
        f_totla_mb += float(std::filesystem::file_size(vec_path[i])) / 1000000;
        // NLOG(info) << "vec_path[i] " << vec_path[i] << " size " << std::filesystem::file_size(vec_path[i]) << " total " << f_totla_mb;
        if (f_totla_mb > f_bag_size_) {
            std::filesystem::remove(vec_path[i]);
            NLOG(info) << "delete " << vec_path[i];
        }
    }
    NLOG(info) << "f_bag_size_ allow " << f_bag_size_ << "MB nowsize " << f_totla_mb << "MB";
}

void ncPlayback::RecordLoop()
{
    bool b_record_on_pre = false;
    while (!b_terminate_) {
        if (!f_use_)
            break;
        std::chrono::duration<double> sec_cmdvel = std::chrono::steady_clock::now() - tp_cmd_vel_;
        if (b_record_on_ == true && sec_cmdvel.count() > f_move_sec_)  // cmd_vel이 안들어온지 좀 됐으면 종료
        {
            b_record_on_ = false;
        }

        std::chrono::duration<double> sec_since_last_err = std::chrono::steady_clock::now() - tp_err_pub_time_;
        if (sec_since_last_err.count() > 1)
            b_error_start_ = false;  // 알람 마지막 발생으로부터 1초 이후에 에러 상태 초기화

        if (b_record_on_ == true && b_record_on_pre == false) {  // record on
            std::lock_guard<std::mutex> lock(mtx_record_);
            s_dir_record_ = s_dir_ + MakeFileName();
            bag_record_.open(s_dir_record_ + ".active", rosbag::bagmode::Write);
            s_status_history_ = "";
        }

        if (b_record_on_ == false && b_record_on_pre == true) {  // record off
            std::lock_guard<std::mutex> lock(mtx_record_);
            bag_record_.close();
            string s_cmd = "mv " + s_dir_record_ + ".active " + s_dir_record_ + "_" + s_status_history_ + ".bag";

            std::chrono::duration<double> sec_start_time = std::chrono::steady_clock::now() - tp_start_time_;
            NLOG(info) << "sec_start_time.count() " << sec_start_time.count();

            if (sec_start_time.count() < 10) {
                // navi 처음 시작시 저장
                s_cmd = "mv " + s_dir_record_ + ".active " + s_dir_record_ + "_" + "start_navigation" + ".bag";
            }

            int n_result = std::system(s_cmd.c_str());
            NLOG(info) << "mv n_result " << n_result;
            s_dir_record_ = "";

            FileCheck();
        }

        b_record_on_pre = b_record_on_;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

void ncPlayback::PlayLoop()
{
    string s_play_cmd_pre = "";
    ros::Time rt_time_pre;
    ros::Time rt_computer_pre;
    while (!b_terminate_) {
        // NLOG(info) << "PlayLoop " << s_play_cmd_ << " / " << s_play_cmd_pre;
        if (f_start_time_ != 0) {
            std_msgs::String time_msg;
            time_msg.data = to_string(f_start_time_);
            time_pub.publish(time_msg);
        }
        if (s_play_cmd_ == "play" && b_play_open_) {
            b_play_open_ = false;
            NLOG(info) << "Play Start";
            if (bag_play_.isOpen()) {
                bag_play_.close();
                delete bag_view_;
            }
            try {
                f_speed_ = 1;
                bag_play_.open(s_dir_play_, rosbag::bagmode::Read);
                bag_view_ = new rosbag::View;
                bag_view_->addQuery(bag_play_);

                view_ = bag_view_->begin();
                rt_time_pre = view_->getTime();
                rt_computer_pre = ros::Time::now();

                bag_begin_time_ = bag_view_->getBeginTime();
                bag_end_time_ = bag_view_->getEndTime();
            }
            catch (const std::exception& e) {
                std::lock_guard<std::mutex> lock(mtx_play_);
                s_play_cmd_ = "";
                NLOG(error) << e.what() << '\n';
            }
        }
        if (s_play_cmd_ == "" && b_play_close_) {
            b_play_close_ = false;
            NLOG(info) << "Play Stop";
            try {
                bag_play_.close();
                delete bag_view_;
            }
            catch (const std::exception& e) {
                NLOG(error) << e.what() << '\n';
            }
        }
        else if (s_play_cmd_ == "play" && f_start_time_ != 0) {
            if (f_start_time_ > bag_begin_time_.toSec() && f_start_time_ < bag_end_time_.toSec()) {
                float f_gap = f_start_time_ - bag_begin_time_.toSec();
                NLOG(info) << "f_gap time " << f_gap;

                view_ = bag_view_->begin();
                int n_count = 0;
                while (view_ != bag_view_->end()) {
                    ros::Time rt_time = view_->getTime();
                    view_++;
                    n_count++;
                    double d_bag_time = rt_time.toSec();

                    if (f_start_time_ - d_bag_time < 0)  // 넘어가는 시간을 찾음.
                    {
                        NLOG(info) << "find seek point " << n_count;
                        break;
                    }
                }
            }
            f_start_time_ = 0;
        }

        if (s_play_cmd_ == "play" && bag_play_.isOpen()) {
            // NLOG(info) << "Play on";
            try {
                view_++;
                if (view_ == bag_view_->end()) {
                    NLOG(info) << "Play End";
                    std::lock_guard<std::mutex> lock(mtx_play_);
                    s_play_cmd_ = "";
                    continue;
                }
                else {
                    string s_topic = view_->getTopic();
                    ros::Time rt_time = view_->getTime();

                    double f_time_gap = rt_time.toSec() - rt_time_pre.toSec();
                    double f_computer_gap = ros::Time::now().toSec() - rt_computer_pre.toSec();

                    // NLOG(info) << " topic :" << s_topic << " gap " << f_time_gap << " computergap " << f_computer_gap;
                    if (s_topic == "/PLAYBACK/navifra/info")
                        navifrainfo_pub.publish(*(view_->instantiate<core_msgs::NavicoreStatus>()));
                    else if (s_topic == "/PLAYBACK/motor_info")
                        motorinfo_pub.publish(*(view_->instantiate<core_msgs::MotorInfo>()));
                    else if (s_topic == "/PLAYBACK/front_cloud_global")
                        scan_front_pub.publish(*(view_->instantiate<sensor_msgs::PointCloud>()));
                    else if (s_topic == "/PLAYBACK/rear_cloud_global")
                        scan_rear_pub.publish(*(view_->instantiate<sensor_msgs::PointCloud>()));
                    else if (s_topic == "/PLAYBACK/left_cloud_global")
                        scan_left_pub.publish(*(view_->instantiate<sensor_msgs::PointCloud>()));
                    else if (s_topic == "/PLAYBACK/right_cloud_global")
                        scan_right_pub.publish(*(view_->instantiate<sensor_msgs::PointCloud>()));
                    else if (s_topic == "/PLAYBACK/v2v_cloud_global")
                        scan_v2v_pub.publish(*(view_->instantiate<sensor_msgs::PointCloud>()));
                    else if (s_topic == "/PLAYBACK/camera_cloud_global")
                        scan_camera_pub.publish(*(view_->instantiate<sensor_msgs::PointCloud>()));
                    else if (s_topic == "/PLAYBACK/NaviFra/visualize/ui_global_path")
                        global_path_pub.publish(*(view_->instantiate<nav_msgs::Path>()));
                    else if (s_topic == "/PLAYBACK/NaviFra/visualize/ui_local_path")
                        local_path_pub.publish(*(view_->instantiate<nav_msgs::Path>()));
                    else if (s_topic == "/PLAYBACK/NaviFra/visualize/robot_collision")
                        robot_col_pub.publish(*(view_->instantiate<geometry_msgs::PolygonStamped>()));
                    else if (s_topic == "/PLAYBACK/NaviFra/visualize/robot_collision_predict")
                        robot_col_pre_pub.publish(*(view_->instantiate<geometry_msgs::PolygonStamped>()));
                    else if (s_topic == "/PLAYBACK/NaviFra/visualize/ui_obstacle_pos")
                        robot_obs_pos_pub.publish(*(view_->instantiate<geometry_msgs::PolygonStamped>()));
                    else if (s_topic == "/PLAYBACK/play_time")
                        time_pub.publish(*(view_->instantiate<std_msgs::String>()));
                    else if (s_topic == "/PLAYBACK/etc_custom_message")
                        robot_uimsg_pub.publish(*(view_->instantiate<std_msgs::String>()));

                    double d_wait_time_sec = f_time_gap - f_computer_gap;
                    // NLOG(info) << "d_wait_time_sec " << d_wait_time_sec;
                    if (d_wait_time_sec > 0) {
                        int wait_milli = int(d_wait_time_sec * float(1000) / f_speed_);
                        if (wait_milli > 1000)
                            wait_milli = 10;
                        // NLOG(info) << "wait_milli " << wait_milli;
                        std::this_thread::sleep_for(std::chrono::milliseconds(wait_milli));
                    }
                    rt_time_pre = rt_time;
                    rt_computer_pre = ros::Time::now();
                }
            }
            catch (const std::exception& e) {
                NLOG(error) << e.what() << '\n';
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        else {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        s_play_cmd_pre = s_play_cmd_;
    }
}

void ncPlayback::DownloadLoop()
{
    while (!b_terminate_) {
        if (s_download_date_.size() > 0 && s_download_start_.size() > 0 && s_download_end_.size() > 0) {
            try {
                NLOG(info) << "s_download_date_ " << s_download_date_;

                string s_logs = s_home + "/navifra_solution/navicore/logs/";
                vector<string> vec_dir;

                std::filesystem::path filepath = std::string(s_logs);
                bool filepathExists = std::filesystem::is_directory(filepath);
                // logs
                if (filepathExists) {
                    for (const fs::directory_entry& entry : fs::recursive_directory_iterator(s_logs)) {
                        // NLOG(info) << "vec_dir " << entry.path();
                        string s_dir = entry.path();
                        if (s_dir.find(".log") != string::npos && s_dir.find("error") == string::npos) {
                            vec_dir.emplace_back(s_dir);
                        }
                    }
                }
                NLOG(info) << "vec_dir size " << vec_dir.size();

                string s_log_file = s_home + "/navicore_temp.log";
                std::filesystem::path p(s_log_file);
                if (std::filesystem::exists(p)) {
                    fs::remove(p);
                    NLOG(info) << "remove navicore_temp.log";
                }
                string s_log_file2 = s_home + "/navicore.log";
                std::filesystem::path p2(s_log_file2);
                if (std::filesystem::exists(p2)) {
                    fs::remove(p2);
                    NLOG(info) << "remove navicore.log";
                }

                // s_download_start_
                // s_download_end_
                string s_log_level_ = "INFO";

                NLOG(info) << "s_download_start_ " << s_download_start_;
                NLOG(info) << "s_download_end_ " << s_download_end_;
                ofstream file(s_log_file);
                if (file.is_open()) {
                    try {
                        for (const auto& dir : vec_dir) {
                            ifstream infile(dir);
                            if (!infile.is_open())
                                continue;

                            string sLine;
                            bool b_read_first = false;

                            while (getline(infile, sLine)) {
                                vector<string> vec_data = Split(sLine, ' ');
                                if (vec_data.size() <= 1)
                                    continue;

                                string s_date = vec_data[0].substr(0, 6);
                                string s_time = vec_data[1].substr(0, 8);
                                
                                if(s_date.size() < 6) // s_date size check
                                    continue;

                                if (s_date[4] != s_download_date_[2] || s_date[5] != s_download_date_[3] ||
                                    s_date[1] != s_download_date_[4] || s_date[2] != s_download_date_[5]) {
                                    // NLOG(info)<<"s_date break / " << s_date << " / " << s_download_date_;
                                    break;
                                }

                                bool time_in_range = (s_time >= s_download_start_ && s_time <= s_download_end_);
                                bool level_matches = (s_log_level_ == "INFO") ||
                                    (s_log_level_ == "WARN" && sLine.find("WARN") != string::npos) ||
                                    (s_log_level_ == "ERROR" && sLine.find("ERROR") != string::npos);

                                if (time_in_range && level_matches) {
                                    b_read_first = true;
                                    file << sLine << "\n";
                                }
                                else if (b_read_first) {
                                    NLOG(info) << "read stop / " << dir;
                                    break;
                                }
                            }
                            infile.close();
                        }
                    }
                    catch (const std::exception& e) {
                        NLOG(warning) << e.what();
                    }
                }
                file.close();

                string s_cmd_sort = "sort " + s_log_file + " > " + s_home + "/navicore.log";
                int n_rt = std::system(s_cmd_sort.c_str());
                string s_gz_file = s_home + "/" + s_file_ + ".gz";
                try
                {
                    string s_cmd_gzip = "tar -czf " + s_gz_file + " " + s_home + "/navicore.log" + " " + s_home + "/navicore.bag" + " " + s_home + "/navicore.json";
                    n_rt = std::system(s_cmd_gzip.c_str());

                    float f_gz_size = float(std::filesystem::file_size(s_gz_file)) / 1000000;
                    NLOG(info) << "f_gz_size " << f_gz_size << "MB";
                    // if(f_gz_size > 5)
                    // {
                    //     NLOG(info)<<"only log...";
                    //     string s_cmd_gzip2 = "gzip -c " + s_home + "/navicore.log" + " > " + s_gz_file;
                    //     n_rt = std::system(s_cmd_gzip2.c_str());
                    // }
                }
                catch(const std::exception& e)
                {
                    string s_cmd_gzip = "gzip -c " + s_home + "/navicore.log" + " > " + s_gz_file;
                    n_rt = std::system(s_cmd_gzip.c_str());
                    NLOG(error) << e.what();
                }

                // string s_cmd_gzip = "gzip -c " + s_home + "/navicore.log" + " > " + s_gz_file;
                // n_rt = std::system(s_cmd_gzip.c_str());

                APIConnect();
                std::filesystem::path p3(s_gz_file);
                if (std::filesystem::exists(p3)) {
                    fs::remove(p3);
                    NLOG(info) << "remove " << s_gz_file;
                }
            }
            catch (const std::exception& e) {
                NLOG(error) << e.what();
            }

            s_download_date_ = "";
            s_download_start_ = "";
            s_download_end_ = "";
            s_file_ = "";
            s_uuid_ = "";
            s_type_ = "";
        }
        else if(s_download_date_full_.size()) {
            try {
                NLOG(info) << "s_download_date_full_ " << s_download_date_full_;

                string s_logs = s_home + "/navifra_solution/navicore/logs/";
                vector<string> vec_dir;

                std::filesystem::path filepath = std::string(s_logs);
                bool filepathExists = std::filesystem::is_directory(filepath);
                // logs
                if (filepathExists) {
                    for (const fs::directory_entry& entry : fs::recursive_directory_iterator(s_logs)) {
                        string s_dir = entry.path();
                        if (s_dir.find(".log") != string::npos && s_dir.find("error") == string::npos) {
                            vec_dir.emplace_back(s_dir);
                        }
                    }
                }
                NLOG(info) << "vec_dir size " << vec_dir.size();

                string s_log_file = s_home + "/navicore_temp.log";
                std::filesystem::path p(s_log_file);
                if (std::filesystem::exists(p)) {
                    fs::remove(p);
                    NLOG(info) << "remove navicore_temp.log";
                }
                string s_log_file2 = s_home + "/navicore.log";
                std::filesystem::path p2(s_log_file2);
                if (std::filesystem::exists(p2)) {
                    fs::remove(p2);
                    NLOG(info) << "remove navicore.log";
                }
                string s_log_level_ = "INFO";
                ofstream file(s_log_file);
                if (file.is_open()) {
                    try {
                        for (const auto& dir : vec_dir) {
                            ifstream infile(dir);
                            if (!infile.is_open())
                                continue;

                            string sLine;
                            bool b_read_first = false;

                            while (getline(infile, sLine)) {
                                vector<string> vec_data = Split(sLine, ' ');
                                if (vec_data.size() <= 1)
                                    continue;

                                string s_date = vec_data[0].substr(0, 6);
                                
                                if(s_date.size() < 6) // s_date size check
                                    continue;

                                if (s_date[4] != s_download_date_full_[2] || s_date[5] != s_download_date_full_[3] ||
                                    s_date[1] != s_download_date_full_[4] || s_date[2] != s_download_date_full_[5]) {
                                    // NLOG(info)<<"s_date break / " << s_date << " / " << s_download_date_full_;
                                    break;
                                }

                                bool level_matches = (s_log_level_ == "INFO") ||
                                    (s_log_level_ == "WARN" && sLine.find("WARN") != string::npos) ||
                                    (s_log_level_ == "ERROR" && sLine.find("ERROR") != string::npos);

                                if (level_matches) {
                                    b_read_first = true;
                                    file << sLine << "\n";
                                }
                                else if (b_read_first) {
                                    NLOG(info) << "read stop / " << dir;
                                    break;
                                }
                            }
                            infile.close();
                        }
                    }
                    catch (const std::exception& e) {
                        NLOG(warning) << e.what();
                    }
                }
                file.close();

                string s_cmd_sort = "sort " + s_log_file + " > " + s_home + "/navicore.log";
                int n_rt = std::system(s_cmd_sort.c_str());
                string s_gz_file = s_home + "/" + s_file_ + ".gz";
                string s_cmd_gzip = "gzip -c " + s_home + "/navicore.log" + " > " + s_gz_file;
                n_rt = std::system(s_cmd_gzip.c_str());
                
                APIConnect();
                std::filesystem::path p3(s_gz_file);
                if (std::filesystem::exists(p3)) {
                    fs::remove(p3);
                    NLOG(info) << "remove " << s_gz_file;
                }
            }
            catch (const std::exception& e) {
                NLOG(error) << e.what();
            }

            s_download_date_full_ = "";
            s_file_ = "";
            s_uuid_ = "";
            s_type_ = "";
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

std::string ncPlayback::getToken()
{
    const char* HOME_DIR = std::getenv("HOME");
    auto s_robot_ip = std::getenv("ROBOT_IP");
    std::string homeDIR = (HOME_DIR != NULL) ? HOME_DIR : "/root";
    std::string extention = ".json";
    std::string default_file_name = "nc_brain_agent_develop.json";
    std::string filepath_ = homeDIR + "/navifra_solution/navicore/configs/configs/";
    std::string filename_ = filepath_ + "nc_brain_agent_develop" + "_" + (s_robot_ip ? s_robot_ip : "localhost") + extention;
    NLOG(info) << "filename_ " << filename_;

    Object::Ptr config = loadJSON(filename_);
    return config->getObject("env")->get("client_token").convert<std::string>();
}

Poco::JSON::Object::Ptr ncPlayback::loadJSON(std::string filename)
{
    if (File(filename).exists()) {
        std::ostringstream ostr;
        Poco::FileInputStream fis(filename);
        Poco::StreamCopier::copyStream(fis, ostr);
        Poco::JSON::Parser parser;
        Poco::Dynamic::Var result = parser.parse(ostr.str());
        Poco::JSON::Object::Ptr data = result.extract<Poco::JSON::Object::Ptr>();

        return std::move(data);
    }
    return nullptr;
}

void ncPlayback::APIConnect()
{
    std::vector<std::string> fileds;
    fileds.push_back("DEVICE");

    std::string backend_host = getBackendHostFromEnv();
    std::string backend_port = getBackendPortFromEnv();

    apiClient_.reset(new NcRESTAPIClient(backend_host, backend_port));

    NLOG(info) << backend_host.c_str() << ":" << backend_port.c_str();

    std::string strToken = getToken();
    // std::string strToken =
    //     "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJpZCI6IjEzZjBhNzI2LTM4NzMtNGU3NC04ZTg4LWRmMDdkNzczMjkxMCIsImFjY291bnQiOiLslYTrgpjtgqgg7Iqk7Lm07J207JuM7LukIiwibW9kZWxfdHlwZSI6IkQxMDAwMSIsIm5hbWUiOiJRRC1Gcmstcm9ib3QxIiwiZGVzYyI6IiIsImxldmVsIjo1LCJpcF9hZGRyIjoicm9ib3QxIiwidG9rZW5fdHlwZSI6ImRldmljZSIsImlhdCI6MTcxMzc2OTUzNywiZXhwIjozMzI0OTc2OTUzN30.MvdDyQ-c1SH-imXN7mPhZhYnmvPJeE_H11uTrcmIil4";
    LOG_INFO("Extracted Data: %s", strToken.c_str());

    std::ostringstream stream1;
    Poco::DeflatingOutputStream gzipper(stream1, Poco::DeflatingStreamBuf::STREAM_GZIP);

    std::string s_log_path = s_home + "/" + s_file_ + ".gz";
    NLOG(info) << "s_log_path " << s_log_path;
    Poco::Net::HTMLForm pocoForm;
    pocoForm.setEncoding(Poco::Net::HTMLForm::ENCODING_MULTIPART);
    pocoForm.add("id", s_uuid_);
    pocoForm.add("key", s_file_ + ".gz");
    pocoForm.add("type", s_type_);
    pocoForm.addPart("log_file_gz", new Poco::Net::FilePartSource(Poco::Path(s_log_path).toString(), "application/gzip"));

    LOG_INFO("Agent - remoteFileNameMsg - Passed");
    std::tuple<std::string, Poco::Net::HTTPResponse::HTTPStatus, std::string> res =
        apiClient_->post_file("/file-logs/upload", pocoForm, strToken);  // brain map upload post

    std::string is = std::get<0>(res);
    HTTPResponse::HTTPStatus status = std::get<1>(res);
    std::string reason = std::get<2>(res);

    if (status == Poco::Net::HTTPResponse::HTTPStatus::HTTP_OK) {
        LOG_INFO("HTTP OK");
    }
    else {
        LOG_ERROR("/file-logs/upload status error : Status %d Reason %s", (int)status, reason.c_str());
    }
}

std::string ncPlayback::getBackendHostFromEnv()
{
    const char* BACKEND_HOST = std::getenv("BACKEND_HOST");
    std::string backend_host = (BACKEND_HOST != NULL) ? BACKEND_HOST : "127.0.0.1";
    return backend_host;
}
std::string ncPlayback::getBackendPortFromEnv()
{
    const char* BACKEND_PORT = std::getenv("BACKEND_PORT");
    std::string backend_port = (BACKEND_PORT != NULL) ? BACKEND_PORT : "5000";
    return backend_port;
}

void ncPlayback::RecvCmdVel(const geometry_msgs::Twist::ConstPtr& msg)
{
    tp_cmd_vel_ = std::chrono::steady_clock::now();
    static bool b_first = true;
    static std::chrono::steady_clock::time_point tp_change_last = std::chrono::steady_clock::now();
    static std::chrono::steady_clock::time_point tp_move_last = std::chrono::steady_clock::now();

    std::chrono::duration<double> sec_error_period = std::chrono::steady_clock::now() - tp_err_start_time_;
    float f_linear_ms = hypot(msg->linear.x, msg->linear.y);
    float f_angular_rads = fabs(msg->angular.z);
    if (f_linear_ms > 0.001 || f_angular_rads > 0.01 ||
        sec_error_period.count() < f_move_sec_)  // 움직임이 있거나 에러 한지 5초 이내라면 상태라면
    {
        tp_move_last = std::chrono::steady_clock::now();
    }

    std::chrono::duration<double> sec_move = std::chrono::steady_clock::now() - tp_move_last;
    std::chrono::duration<double> sec_change = std::chrono::steady_clock::now() - tp_change_last;

    if (sec_change.count() > f_change_sec_ && sec_move.count() < f_move_sec_ && !b_record_on_) {
        tp_change_last = std::chrono::steady_clock::now();
        b_record_on_ = true;
        NLOG(info) << "START RECORD";
        n_global_path_ = 0;
        n_local_path_ = 0;
    }
    if (sec_change.count() > f_change_sec_ && sec_move.count() > f_move_sec_ && b_record_on_) {
        tp_change_last = std::chrono::steady_clock::now();
        b_record_on_ = false;
        NLOG(info) << "STOP RECORD";
    }
    if (s_dir_record_.size() > 0 && b_record_on_ == true) {
        try {
            float f_active_size = float(std::filesystem::file_size(s_dir_record_ + ".active")) / 1000000;
            // NLOG(info) << "f_active_size " << f_active_size;
            if (f_active_size > f_bag_onefile_size_)  // active 가 100MB를 넘으면 f_bag_onefile_size_
            {
                b_record_on_ = false;
            }
        }
        catch (const std::exception& e) {
            NLOG(error) << e.what();
        }
    }
}

vector<string> ncPlayback::Split(const string& data, char c_st)
{
    vector<string> words;
    stringstream sstream(data);
    string word;

    while (getline(sstream, word, c_st)) {
        words.push_back(word);
        // NLOG(info) << "word " << word;
    }
    return words;
}

string ncPlayback::MakeDate(const string& num_date)
{
    string result = "";
    NLOG(info) << "make " << num_date;
    if (num_date.size() == 6) {
        result += num_date[0];
        result += num_date[1];
        result += "-";
        string s_mon = num_date.substr(2, 2);
        NLOG(info) << "s_mon " << s_mon;
        if (s_mon == "01")
            result += "Jan";
        if (s_mon == "02")
            result += "Feb";
        if (s_mon == "03")
            result += "Mar";
        if (s_mon == "04")
            result += "Apr";
        if (s_mon == "05")
            result += "May";
        if (s_mon == "06")
            result += "Jun";
        if (s_mon == "07")
            result += "Jul";
        if (s_mon == "08")
            result += "Aug";
        if (s_mon == "09")
            result += "Sep";
        if (s_mon == "10")
            result += "Oct";
        if (s_mon == "11")
            result += "Nov";
        if (s_mon == "12")
            result += "Dec";
        result += "-";
        result += num_date[4];
        result += num_date[5];
    }
    return result;
}

void ncPlayback::RecvCmd(const std_msgs::String::ConstPtr& msg)
{
    LOG_INFO("RecvCmd %s", msg->data.c_str());
    if (msg->data == "record_start") {
        b_record_on_ = true;
    }
    if (msg->data == "record_stop") {
        b_record_on_ = false;
    }

    vector<string> words;
    stringstream sstream(msg->data);
    string word;

    while (getline(sstream, word, '/')) {
        words.push_back(word);
        NLOG(info) << "word " << word;
    }
    NLOG(info) << "words.size() " << words.size();

    if (words.size() >= 1) {
        std::lock_guard<std::mutex> lock(mtx_play_);
        if (words[0] == "play" && words.size() >= 2) {
            NLOG(info) << "words[0] " << words[0] << " words[1] " << words[1];
            if (s_play_cmd_ == "play" && s_dir_play_ == s_dir_ + words[1])  // 같은파일이라면
            {
                NLOG(info) << "play same file";
                return;
            }
            s_dir_play_ = s_dir_ + words[1];
            s_play_cmd_ = "play";
            NLOG(info) << "s_dir_play_ " << s_dir_play_;
            b_play_open_ = true;
        }
        else if (words[0] == "stop" && s_play_cmd_ == "play") {
            NLOG(info) << "words[0] " << words[0];
            s_play_cmd_ = "stop";
        }
        else if (words[0] == "cancel") {
            NLOG(info) << "words[0] " << words[0];
            s_dir_play_ = "";
            s_play_cmd_ = "";
            b_play_close_ = true;
        }
        else if (words[0] == "speed" && words.size() >= 2) {
            NLOG(info) << "words[0] " << words[0] << " words[1] " << words[1];
            try {
                float f_speed = stof(words[1]);
                f_speed_ = f_speed;
            }
            catch (const std::exception& e) {
                NLOG(error) << e.what() << '\n';
            }
        }
        else if (words[0] == "seek" && words.size() >= 2 && s_play_cmd_ == "play") {
            NLOG(info) << "words[0] " << words[0] << " words[1] " << words[1];
            try {
                f_start_time_ = stod(words[1]);
            }
            catch (const std::exception& e) {
                NLOG(error) << e.what() << '\n';
            }
        }

        else if (words[0] == "download" && words.size() == 4)  // for log download...
        {
            string s_rosbag = s_dir_ + words[1];
            s_file_ = words[1];
            s_uuid_ = words[2];
            s_type_ = words[3];
            NLOG(info) << "download s_rosbag " << s_rosbag << " s_uuid_ " << s_uuid_ << " s_type_ " << s_type_;

            try {
                if (s_type_ == "playback_log") {
                    rosbag::Bag bag_down;
                    bag_down.open(s_rosbag, rosbag::bagmode::Read);

                    rosbag::View bag_view(bag_down);
                    ros::Time start_time = bag_view.getBeginTime();
                    ros::Time end_time = bag_view.getEndTime();
                    bag_down.close();

                    try
                    {
                        string s_cp_cmd = "cp " + s_rosbag + " " + s_home + "/navicore.bag";
                        int n_result = std::system(s_cp_cmd.c_str());
                        NLOG(info) << s_cp_cmd<<" mv n_result " << n_result;

                        string s_cp_cmd2 = "cp " + s_home + "/navifra_solution/navicore/configs/map/latest/map.json" + " " + s_home + "/navicore.json";
                        int n_result2 = std::system(s_cp_cmd2.c_str());
                        NLOG(info) << s_cp_cmd2<<" mv n_result2 " << n_result2;
                    }
                    catch(const std::exception& e)
                    {
                        NLOG(error) << e.what() << '\n';
                    }

                    int duration_sec = int((end_time - start_time).toSec());

                    s_download_date_ = words[1].substr(0, 8);
                    s_download_date_.erase(find(s_download_date_.begin(), s_download_date_.end(), '-'));
                    s_download_date_.erase(find(s_download_date_.begin(), s_download_date_.end(), '-'));
                    NLOG(info) << "s_download_date_ " << s_download_date_;
                    s_download_start_ = words[1].substr(9, 8);
                    NLOG(info) << " s_download_start_ " << s_download_start_;

                    int n_hour = stoi(s_download_start_.substr(0, 2));
                    int n_min = stoi(s_download_start_.substr(3, 2));
                    int n_sec = stoi(s_download_start_.substr(6, 2));
                    NLOG(info) << "duration_sec " << duration_sec;
                    NLOG(info) << "before n_hour " << n_hour << " n_min " << n_min << " n_sec " << n_sec;

                    int n_hour_start = n_hour;
                    int n_min_start = n_min;
                    int n_sec_start = n_sec;

                    n_sec_start -= 5;
                    if (n_sec_start < 0) {
                        n_sec_start += 60;
                        n_min_start -= 1;
                    }
                    if (n_min_start < 0) {
                        n_min_start += 60;
                        n_hour_start -= 1;
                    }
                    if (n_hour_start < 0) {
                        n_hour_start = 0;
                        n_min_start = 0;
                        n_sec_start = 0;
                    }
                    string s_start_hour, s_start_min, s_start_sec;
                    if (to_string(n_hour_start).size() == 1)
                        s_start_hour = "0" + to_string(n_hour_start);
                    else
                        s_start_hour = to_string(n_hour_start);
                    if (to_string(n_min_start).size() == 1)
                        s_start_min = "0" + to_string(n_min_start);
                    else
                        s_start_min = to_string(n_min_start);
                    if (to_string(n_sec_start).size() == 1)
                        s_start_sec = "0" + to_string(n_sec_start);
                    else
                        s_start_sec = to_string(n_sec_start);

                    int n_add_h = duration_sec / 3600;
                    int n_add_m = (duration_sec % 3600) / 60;
                    int n_add_s = (duration_sec % 3600) % 60;

                    n_sec += n_add_s;
                    if (n_sec >= 60) {
                        n_sec -= 60;
                        n_add_m += 1;
                    }
                    n_min += n_add_m;
                    if (n_min >= 60) {
                        n_min -= 60;
                        n_add_h += 1;
                    }
                    n_hour += n_add_h;
                    NLOG(info) << "after n_hour " << n_hour << " n_min " << n_min << " n_sec " << n_sec;

                    string s_end_hour, s_end_min, s_end_sec;
                    if (to_string(n_hour).size() == 1)
                        s_end_hour = "0" + to_string(n_hour);
                    else
                        s_end_hour = to_string(n_hour);
                    if (to_string(n_min).size() == 1)
                        s_end_min = "0" + to_string(n_min);
                    else
                        s_end_min = to_string(n_min);
                    if (to_string(n_sec).size() == 1)
                        s_end_sec = "0" + to_string(n_sec);
                    else
                        s_end_sec = to_string(n_sec);

                    s_download_start_ = s_start_hour + ":" + s_start_min + ":" + s_start_sec;
                    s_download_end_ = s_end_hour + ":" + s_end_min + ":" + s_end_sec;
                    NLOG(info) << "s_download_start_ " << s_download_start_;
                    NLOG(info) << "s_download_end_ " << s_download_end_;
                }
                else if (s_type_ == "file_log") {
                    s_download_date_full_ = words[1].substr(2, 10);
                    s_download_date_full_.erase(find(s_download_date_full_.begin(), s_download_date_full_.end(), '-'));
                    s_download_date_full_.erase(find(s_download_date_full_.begin(), s_download_date_full_.end(), '-'));
                    NLOG(info) << "s_download_date_full_ " << s_download_date_full_;
                }
            }
            catch (const std::exception& e) {
                NLOG(error) << e.what() << '\n';
            }
        }
    }
}

void ncPlayback::RecvAlarm(const core_msgs::NaviAlarm::ConstPtr& msg)
{
    if (msg->alarm == core_msgs::NaviAlarm::GOAL_ARRIVED && b_record_on_ && s_status_history_.size() <= 10) {
        s_status_history_ += "A";
    }
    else if (msg->alarm == core_msgs::NaviAlarm::CANCELED && b_record_on_ && s_status_history_.size() <= 10) {
        s_status_history_ += "C";
    }
    else if (msg->alarm == core_msgs::NaviAlarm::MOVE_PAUSED && b_record_on_ && s_status_history_.size() <= 10) {
        s_status_history_ += "P";
    }
    else if (msg->alarm == core_msgs::NaviAlarm::MOVE_RESUME && b_record_on_ && s_status_history_.size() <= 10) {
        s_status_history_ += "R";
    }
    else if (msg->alarm >= 2000) {
        tp_err_pub_time_ = std::chrono::steady_clock::now();
        // 에러 발생 시 녹화 시작
        // 에러 정상화 시 RecvCmdVel 콜백이 걸리면서 녹화 종료
        if (!b_error_start_) {
            b_error_start_ = true;
            tp_err_start_time_ = std::chrono::steady_clock::now();
        }

        std::chrono::duration<double> sec_error_period = std::chrono::steady_clock::now() - tp_err_start_time_;
        if (sec_error_period.count() < 5) {
            if (!b_record_on_)
                b_record_on_ = true;
            tp_cmd_vel_ = std::chrono::steady_clock::now();
        }
        // NLOG(info) << "START RECORD by Alarm";
    }
}

void ncPlayback::RecordInfo(const core_msgs::NavicoreStatus::ConstPtr& msg)
{
    std::lock_guard<std::mutex> lock(mtx_record_);
    if (s_dir_record_.size() > 0) {
        try {
            bag_record_.write("/PLAYBACK/navifra/info", ros::Time::now(), *msg);
        }
        catch (const std::exception& e) {
            NLOG(error) << e.what() << '\n';
        }

        try {
            std_msgs::String time_msg;
            time_msg.data = to_string(ros::Time::now().toSec());
            bag_record_.write("/PLAYBACK/play_time", ros::Time::now(), time_msg);
        }
        catch (const std::exception& e) {
            NLOG(error) << e.what() << '\n';
        }
        if (s_dir_record_.size() > 0 && n_global_path_ <= 10)
            n_global_path_++;
        if (s_dir_record_.size() > 0 && n_local_path_ <= 10)
            n_local_path_++;

        if (n_global_path_ > 10 && n_global_path_ < 100) {
            n_global_path_ = 100;
            try {
                bag_record_.write("/PLAYBACK/NaviFra/visualize/ui_global_path", ros::Time::now(), o_global_path_);
            }
            catch (const std::exception& e) {
                NLOG(error) << e.what() << '\n';
            }
        }
        if (n_local_path_ > 10 && n_local_path_ < 100) {
            n_local_path_ = 100;
            try {
                bag_record_.write("/PLAYBACK/NaviFra/visualize/ui_local_path", ros::Time::now(), o_local_path_);
            }
            catch (const std::exception& e) {
                NLOG(error) << e.what() << '\n';
            }
        }
    }
    static string status_pre = msg->s_status;
    // NLOG(info) << "b_record_on_ " << b_record_on_ << " msg->s_status " << msg->s_status << " s_status_history_ " << s_status_history_;
    if (b_record_on_ && s_status_history_.size() == 0 && msg->s_status.size() > 0) {
        s_status_history_ += msg->s_status[0];
    }
    if (b_record_on_ && msg->s_status.size() > 0 && status_pre != msg->s_status && s_status_history_.size() <= 10) {
        s_status_history_ += msg->s_status[0];
    }

    status_pre = msg->s_status;
}

void ncPlayback::RecordMotorInfo(const core_msgs::MotorInfo::ConstPtr& msg)
{
    static int n_count = 0;
    std::lock_guard<std::mutex> lock(mtx_record_);
    n_count++;
    if (s_dir_record_.size() > 0 && n_count >= n_lidar_skip_count_) {
        n_count = 0;

        try {
            bag_record_.write("/PLAYBACK/motor_info", ros::Time::now(), *msg);
        }
        catch (const std::exception& e) {
            NLOG(error) << e.what() << '\n';
        }
    }
}

void ncPlayback::RecordFront(const sensor_msgs::PointCloud::ConstPtr& msg)
{
    static int n_count = 0;
    std::lock_guard<std::mutex> lock(mtx_record_);
    n_count++;
    if (s_dir_record_.size() > 0 && n_count >= n_lidar_skip_count_) {
        n_count = 0;
        try {
            bag_record_.write("/PLAYBACK/front_cloud_global", ros::Time::now(), *msg);
        }
        catch (const std::exception& e) {
            NLOG(error) << e.what() << '\n';
        }
    }
}

void ncPlayback::RecordRear(const sensor_msgs::PointCloud::ConstPtr& msg)
{
    static int n_count = 0;
    std::lock_guard<std::mutex> lock(mtx_record_);
    n_count++;
    if (s_dir_record_.size() > 0 && n_count >= n_lidar_skip_count_) {
        n_count = 0;
        try {
            bag_record_.write("/PLAYBACK/rear_cloud_global", ros::Time::now(), *msg);
        }
        catch (const std::exception& e) {
            NLOG(error) << e.what() << '\n';
        }
    }
}

void ncPlayback::RecordLeft(const sensor_msgs::PointCloud::ConstPtr& msg)
{
    static int n_count = 0;
    std::lock_guard<std::mutex> lock(mtx_record_);
    n_count++;
    if (s_dir_record_.size() > 0 && n_count >= n_lidar_skip_count_) {
        n_count = 0;
        try {
            bag_record_.write("/PLAYBACK/left_cloud_global", ros::Time::now(), *msg);
        }
        catch (const std::exception& e) {
            NLOG(error) << e.what() << '\n';
        }
    }
}

void ncPlayback::RecordRight(const sensor_msgs::PointCloud::ConstPtr& msg)
{
    static int n_count = 0;
    std::lock_guard<std::mutex> lock(mtx_record_);
    n_count++;
    if (s_dir_record_.size() > 0 && n_count >= n_lidar_skip_count_) {
        n_count = 0;
        try {
            bag_record_.write("/PLAYBACK/right_cloud_global", ros::Time::now(), *msg);
        }
        catch (const std::exception& e) {
            NLOG(error) << e.what() << '\n';
        }
    }
}

void ncPlayback::RecordV2V(const sensor_msgs::PointCloud::ConstPtr& msg)
{
    std::lock_guard<std::mutex> lock(mtx_record_);
    if (s_dir_record_.size() > 0) {
        try {
            bag_record_.write("/PLAYBACK/v2v_cloud_global", ros::Time::now(), *msg);
        }
        catch (const std::exception& e) {
            NLOG(error) << e.what() << '\n';
        }
    }
}

void ncPlayback::RecordCamera(const sensor_msgs::PointCloud::ConstPtr& msg)
{
    std::lock_guard<std::mutex> lock(mtx_record_);
    if (s_dir_record_.size() > 0) {
        try {
            bag_record_.write("/PLAYBACK/camera_cloud_global", ros::Time::now(), *msg);
        }
        catch (const std::exception& e) {
            NLOG(error) << e.what() << '\n';
        }
    }
}

void ncPlayback::RecordGPath(const nav_msgs::Path::ConstPtr& msg)
{
    std::lock_guard<std::mutex> lock(mtx_record_);
    o_global_path_ = *msg;
    if (s_dir_record_.size() > 0) {
        try {
            bag_record_.write("/PLAYBACK/NaviFra/visualize/ui_global_path", ros::Time::now(), *msg);
        }
        catch (const std::exception& e) {
            NLOG(error) << e.what() << '\n';
        }
    }
}

void ncPlayback::RecordLPath(const nav_msgs::Path::ConstPtr& msg)
{
    std::lock_guard<std::mutex> lock(mtx_record_);
    o_local_path_ = *msg;
    if (s_dir_record_.size() > 0) {
        try {
            bag_record_.write("/PLAYBACK/NaviFra/visualize/ui_local_path", ros::Time::now(), *msg);
        }
        catch (const std::exception& e) {
            NLOG(error) << e.what() << '\n';
        }
    }
}

void ncPlayback::RecordC(const geometry_msgs::PolygonStamped::ConstPtr& msg)
{
    std::lock_guard<std::mutex> lock(mtx_record_);
    if (s_dir_record_.size() > 0) {
        try {
            bag_record_.write("/PLAYBACK/NaviFra/visualize/robot_collision", ros::Time::now(), *msg);
        }
        catch (const std::exception& e) {
            NLOG(error) << e.what() << '\n';
        }
    }
}

void ncPlayback::RecordCP(const geometry_msgs::PolygonStamped::ConstPtr& msg)
{
    std::lock_guard<std::mutex> lock(mtx_record_);
    if (s_dir_record_.size() > 0) {
        try {
            bag_record_.write("/PLAYBACK/NaviFra/visualize/robot_collision_predict", ros::Time::now(), *msg);
        }
        catch (const std::exception& e) {
            NLOG(error) << e.what() << '\n';
        }
    }
}

void ncPlayback::RecordOP(const geometry_msgs::PolygonStamped::ConstPtr& msg)
{
    std::lock_guard<std::mutex> lock(mtx_record_);
    if (s_dir_record_.size() > 0) {
        try {
            bag_record_.write("/PLAYBACK/NaviFra/visualize/ui_obstacle_pos", ros::Time::now(), *msg);
        }
        catch (const std::exception& e) {
            NLOG(error) << e.what() << '\n';
        }
    }
}

void ncPlayback::RecordMSG(const std_msgs::String::ConstPtr& msg)
{
    std::lock_guard<std::mutex> lock(mtx_record_);
    if (s_dir_record_.size() > 0) {
        try {
            bag_record_.write("/PLAYBACK/etc_custom_message", ros::Time::now(), *msg);
        }
        catch (const std::exception& e) {
            NLOG(error) << e.what() << '\n';
        }
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "nc_playback_cpp");
    LOG_INFO("NaviFra nc_playback_cpp node is launched!!", 1);

    ncPlayback o_node;
    ros::AsyncSpinner spinner(0);
    spinner.start();
    ros::waitForShutdown();
    return 0;
}
