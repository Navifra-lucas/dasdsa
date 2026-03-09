#include "param_repository.hpp"

#include "util/logger.hpp"

namespace NaviFra {
void ParamRepository::RegisterCbFunc(const std::string& str_cbf_name, const std::function<void(const boost::any&)>& pt_func)
{
    if (map_callback_pt_.find(str_cbf_name) == map_callback_pt_.end()) {
        map_callback_pt_[str_cbf_name] = pt_func;
        LOG_INFO("[param register cbfunc : %s]", str_cbf_name.c_str());
    }
}

void ParamRepository::NotifyParam(const NaviFra::Parameters_t& st_param_)
{
    std::map<std::string, std::function<void(const boost::any&)>>::iterator iter;
    // 전체 순회하며 등록된 함수로 모두 param set
    int n_set_func_num = 0;
    for (iter = map_callback_pt_.begin(); iter != map_callback_pt_.end(); iter++) {
        LOG_INFO("[param notify : %s]", iter->first.c_str());
        iter->second(st_param_);
        n_set_func_num++;
    }
    LOG_INFO("n_set_func_num : %d", n_set_func_num);
}
}  // namespace NaviFra
