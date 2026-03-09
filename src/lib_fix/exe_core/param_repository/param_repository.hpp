/*
 * @file	: param_repository.hpp
 * @date	: Jan. 1, 2022
 * @author	: "Joongtae, Park(brain)" (founder@navifra.com)"
 * @brief	: 주행 관련 파라미터 정보를 저장해주는 class
 * @remark	: 싱글톤 형태로 만들어짐
 * @warning	:
 * 	Copyright(C) 2022 NaviFra Coperation. All Rights are Reserved.
 */

#ifndef NAVIFRA_PARAM_REPOSITORY_H_
#define NAVIFRA_PARAM_REPOSITORY_H_

#include "msg_information/parameter_information/param_msg.hpp"
#include "util/cbfunc_register.hpp"

#include "util/singleton_generator.hpp"

#include <boost/any.hpp>

#include <functional>
#include <iostream>

namespace NaviFra {
class ParamRepository : public SingletonGenerator<ParamRepository> {
public:
    ParamRepository(){};
    virtual ~ParamRepository(){};

    /**
     * @brief ParamRepository에 등록하고 싶은 콜백함수를 설정하는 함수
     *
     * @param n_cb_name
     * @param pt_func_
     */
    void RegisterCbFunc(const std::string& str_cbf_name, const std::function<void(const boost::any&)>& pt_func);

    /**
     * @brief 로봇 모션에 관려된 param값을 초기화 하는 루틴
     *
     */
    void NotifyParam(const NaviFra::Parameters_t& st_param_);

private:
    std::map<std::string, std::function<void(const boost::any&)>> map_callback_pt_;
};
}  // namespace NaviFra

#endif