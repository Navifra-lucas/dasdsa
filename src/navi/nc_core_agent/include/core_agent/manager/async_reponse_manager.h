#ifndef NAVIFRA_ASYNC_RESPONSE_MANAGER_H
#define NAVIFRA_ASYNC_RESPONSE_MANAGER_H

#include <Poco/SingletonHolder.h>
#include <boost/any.hpp>
#include <boost/variant.hpp>

#include <chrono>
#include <future>
#include <map>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <typeindex>

namespace NaviFra {

// ID 타입을 int 또는 std::string으로 허용하기 위한 타입 정의
using IDType = boost::variant<int, std::string>;

class AsyncResponseManager {
public:
    // 싱글톤 인스턴스 반환
    static AsyncResponseManager& instance()
    {
        static Poco::SingletonHolder<AsyncResponseManager> sh;
        return *sh.get();
    }

    // 특정 타입 T의 특정 ID에 대한 promise와 future를 생성하는 함수
    template <typename T>
    void createPromise(const IDType& id)
    {
        std::unique_lock<std::mutex> lock(mutex_);  // 락을 걸어 동기화
        auto promise = std::make_shared<std::promise<T>>();  // std::shared_ptr로 promise 생성
        auto future = promise->get_future();  // promise로부터 future 가져오기
        futures_[{id, std::type_index(typeid(T))}] = std::make_shared<std::future<T>>(std::move(future));  // shared_ptr로 future 저장
        promises_[{id, std::type_index(typeid(T))}] = promise;  // shared_ptr로 promise 저장
    }

    // 특정 타입 T의 특정 ID에 대한 promise에 값을 설정하는 함수
    template <typename T>
    void setPromiseValue(const IDType& id, const T& value)
    {
        std::unique_lock<std::mutex> lock(mutex_);  // 락을 걸어 동기화
        auto key = std::make_pair(id, std::type_index(typeid(T)));
        auto it = promises_.find(key);
        if (it != promises_.end()) {
            // shared_ptr로 저장된 promise에 값 설정
            std::static_pointer_cast<std::promise<T>>(it->second)->set_value(value);
            promises_.erase(it);  // 사용된 promise 삭제
        }
    }

    // 특정 타입 T의 특정 ID에 대한 응답을 대기하는 함수
    template <typename T>
    T waitForResponse(const IDType& id, int timeout_ms = 1000)
    {
        std::shared_ptr<std::future<T>> future;
        {  // future를 가져올 때까지만 락을 걸어 동기화
            std::unique_lock<std::mutex> lock(mutex_);
            auto key = std::make_pair(id, std::type_index(typeid(T)));
            auto it = futures_.find(key);
            if (it != futures_.end()) {
                future = std::static_pointer_cast<std::future<T>>(it->second);  // shared_ptr로 저장된 future 가져오기
            }
            else {
                throw std::runtime_error("Future not found for the given ID and type.");
            }
        }  // 락을 해제하여 다른 스레드가 future에 접근할 수 있도록 함

        auto status = future->wait_for(std::chrono::milliseconds(timeout_ms));
        if (status == std::future_status::ready) {
            T value = future->get();  // future로부터 값 가져오기
            {  // future 삭제를 위해 다시 락을 걸어 동기화
                std::unique_lock<std::mutex> lock(mutex_);
                futures_.erase({id, std::type_index(typeid(T))});
            }
            return value;
        }
        else {
            throw std::runtime_error("Timeout: Response not received within the given period.");
        }
    }

    // 특정 타입 T의 특정 ID에 대한 promise와 future를 삭제하는 함수
    template <typename T>
    void remove(const IDType& id)
    {
        std::unique_lock<std::mutex> lock(mutex_);  // 락을 걸어 동기화
        auto key = std::make_pair(id, std::type_index(typeid(T)));
        promises_.erase(key);
        futures_.erase(key);
    }

    // 특정 타입 T의 특정 ID에 대한 promise가 존재하는지 확인하는 함수
    template <typename T>
    bool exists(const IDType& id)
    {
        std::unique_lock<std::mutex> lock(mutex_);  // 락을 걸어 동기화
        auto key = std::make_pair(id, std::type_index(typeid(T)));
        return promises_.find(key) != promises_.end();
    }

private:
    // 각 ID와 타입별로 promise와 future를 저장하기 위한 맵
    std::map<std::pair<IDType, std::type_index>, std::shared_ptr<void>> promises_;  // 각 ID와 타입별로 shared_ptr로 저장된 promise
    std::map<std::pair<IDType, std::type_index>, std::shared_ptr<void>> futures_;  // 각 ID와 타입별로 shared_ptr로 저장된 future
    std::mutex mutex_;  // 동기화를 위한 mutex
};

}  // namespace NaviFra

#endif  // NAVIFRA_ASYNC_RESPONSE_MANAGER_H
