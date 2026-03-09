
/*
 * @file	: singleton_generator.hpp
 * @date	: Jan. 1, 2022
 * @author	: "Joongtae, Park(brain)" (founder@navifra.com)"
 * @brief	: 상속을 통해 싱글톤 클래스를 만드는 템플릿
 * @remark	: Modern C++ Thread-Safe Singleton Template 참고
 * @warning	:
 * 	Copyright(C) 2022 NaviFra Coperation. All Rights are Reserved.
 */

#ifndef NAVIFRA_SINGLETON_GENERATOR_H_
#define NAVIFRA_SINGLETON_GENERATOR_H_

#include <iostream>
#include <memory>
#include <mutex>
#include <typeinfo>

namespace NaviFra {

template <typename T>
class SingletonGenerator {
protected:
    SingletonGenerator<T>() = default;

public:
    virtual ~SingletonGenerator<T>() = default;

    SingletonGenerator<T>(const SingletonGenerator<T>&) = delete;
    SingletonGenerator<T>& operator=(const SingletonGenerator<T>&) = delete;

    // 원시 포인터 -> unique_ptr 변경, 로그 제거
    static T* GetInstance()
    {
        static std::once_flag onceFlag;
        static std::unique_ptr<T> instance;
        std::call_once(onceFlag, []() { instance.reset(new T{}); });
        return instance.get();
    }
};

}  // namespace NaviFra
#endif