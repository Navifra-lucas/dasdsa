
/*
 * @file	: cbfunc_register.hpp
 * @date	: Jan. 1, 2022
 * @author	: "Joongtae, Park(brain)" (founder@navifra.com)"
 * @brief	: 콜백을 원하는 함수를 저장하고 실행시켜주는 싱클톤 class
 * @remark	: 싱글톤 형태로 만들어짐
 * @warning	:
 * 	Copyright(C) 2022 NaviFra Coperation. All Rights are Reserved.
 */

#ifndef CALL_BACK_FUNC_REGISTER_H_
#define CALL_BACK_FUNC_REGISTER_H_

#include "util/util.hpp"

#include <boost/any.hpp>
#include <boost/thread/thread.hpp>

#include <chrono>
#include <condition_variable>
#include <functional>
#include <iostream>
#include <map>
#include <mutex>
#include <thread>
#include <typeinfo>
using namespace std;

namespace NaviFra {
class CBFuncRegister {
public:
  CBFuncRegister();
  virtual ~CBFuncRegister();

  bool RegistCbFunc(const std::string &str_cb_name,
                    std::function<void(const boost::any &)> pt_func_);
  bool RegistCbFunc(const int &n_cb_name,
                    std::function<void(const boost::any &)> pt_func_);
  bool RegistCbFunc(const int &n_cb_name,
                    std::function<void(const boost::any &, const boost::any &,
                                       const boost::any &)>
                        pt_func_);

  bool notify(const std::string &str_cb_name, const boost::any &any_type_var);
  bool notify(const int &n_cb_name, const boost::any &any_type_var);
  bool notify(const int &n_cb_name, const boost::any &any_type_var1,
              const boost::any &any_type_var2, const boost::any &any_type_var3);

private:
  std::map<std::string, std::function<void(const boost::any &)>>
      string_map_callback_pt_;
  std::map<int, std::function<void(const boost::any &)>> map_callback_pt_;
  std::map<int, std::function<void(const boost::any &, const boost::any &,
                                   const boost::any &)>>
      map_callback_pt_3arg_;
};

} // namespace NaviFra
#endif
