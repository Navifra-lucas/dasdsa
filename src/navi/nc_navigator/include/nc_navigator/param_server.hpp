/*
 * @file	: param_server.hpp
 * @date	: Dec 12, 2021
 * @author	: "Nate"
 * @brief	: navigtaion에 사용되는 파라미터
 * @remark	:
 * @warning	:
 * 	Copyright(C) 2021 NaviFra Coperation.
 * 	All Rights are Reserved.
 */

#ifndef NAVIFRA_PARAM_SERVER_H_
#define NAVIFRA_PARAM_SERVER_H_

#include "param_repository/param_repository.hpp"

#include <boost/any.hpp>

#include <sstream>

using namespace NaviFra;
class ParamServer {
public:
    ParamServer();
    virtual ~ParamServer(){};

    /**
     * @brief parameter get && update
     *
     */
    void UpdateParam();

    void GetParam();

    void ParamCmd(const std_msgs::String::ConstPtr& msg);

    void CheckState(const std_msgs::String::ConstPtr& msg);

    bool Notify(const std::string& str_cbf_name, const boost::any& any_type_var);

    /**
     * @brief 외부에서 Callback fuction을 등록하는 함수
     *
     * @param str_cbf_name
     * @param pt_func_
     * @return bool true,flase
     */
    bool RegisteCallbackFunc(const std::string& str_cbf_name, const std::function<void(const boost::any&)>& pt_func_);

    vector<string> split(string input, char delimiter);

    bool b_robot_state_;

    bool b_robot_init_ = false;

private:
    ros::NodeHandle node_handle_;  // 노드 핸들러
    ros::Publisher pub_param_data;

    ros::Subscriber param_update_sub_;  // 파라미터 업데이트
    ros::Subscriber navi_state_sub_;

    NaviFra::Parameters_t st_param_;  // 파라미터

    std::map<std::string, std::function<void(const boost::any&)>> map_callback_pt_;
};

#endif
