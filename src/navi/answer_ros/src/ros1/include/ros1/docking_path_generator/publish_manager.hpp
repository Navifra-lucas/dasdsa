#ifndef PUBLISH_MANAGER_HPP_
#define PUBLISH_MANAGER_HPP_

#include "utils/publish_callback.hpp"

namespace NVFR {

namespace PCM {

template <typename ClassType, typename... Args>
inline bool RegistCbFunc(int n_key, void (ClassType::*func)(const Args& ...), ClassType* obj)
{
    return PublishCb<Args ...>::GetInstance()->RegistCbFunc(n_key, [obj, func](const Args& ...args) { (obj->*func)(args...); });
}

template <typename... Args>
inline void Publish(int n_key, const Args& ...args)
{
    PublishCb<Args ...>::GetInstance()->Publish(n_key, args...);
}

namespace KEY {

inline constexpr int DOCKING_PATH = 0x0001;
inline constexpr int RETURN_PATH = 0x0002;

}  // namespace KEY

}  // namespace PCM

}  // namespace NVFR

#endif
