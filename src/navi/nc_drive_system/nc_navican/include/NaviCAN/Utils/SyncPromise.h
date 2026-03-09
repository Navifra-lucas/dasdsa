#ifndef NAVIFRA_NAVICAN_SYNC_PROMISE_H
#define NAVIFRA_NAVICAN_SYNC_PROMISE_H

#include <lely/coapp/master.hpp>

#include <atomic>
#include <condition_variable>
#include <future>
#include <memory>
#include <mutex>
#include <string>

namespace NaviFra {
namespace NaviCAN {
namespace Utils {

/**
 * @brief Thread-safe Promise wrapper
 *
 * Promise/Future 패턴을 스레드 안전하게 관리하는 유틸리티 클래스입니다.
 * 여러 스레드에서 안전하게 promise를 설정하고 future를 가져올 수 있습니다.
 *
 * @tparam T Promise가 반환할 타입
 *
 * @example
 * SyncPromise<int> sync_promise;
 * auto future = sync_promise.getFuture();
 * sync_promise.setValue(42);  // 다른 스레드에서
 * int value = future.get();
 */
template<typename T>
class SyncPromise {
public:
    SyncPromise() = default;

    // Non-copyable
    SyncPromise(const SyncPromise&) = delete;
    SyncPromise& operator=(const SyncPromise&) = delete;

    // Movable
    SyncPromise(SyncPromise&&) noexcept = default;
    SyncPromise& operator=(SyncPromise&&) noexcept = default;

    /**
     * @brief 새로운 future를 생성하고 반환
     *
     * 기존 promise를 리셋하고 새로운 future를 반환합니다.
     * 이 함수는 스레드 안전합니다.
     *
     * @return std::future<T>
     */
    std::future<T> getFuture() {
        std::scoped_lock lock(mutex_);
        is_set_ = false;
        promise_ = std::promise<T>();
        return promise_.get_future();
    }

    /**
     * @brief Promise에 값을 설정
     *
     * is_set flag를 체크하여 이미 설정되었다면 무시합니다.
     * try_lock을 사용하여 lock 경합을 최소화합니다.
     *
     * @param value 설정할 값
     * @return true 값이 설정됨, false 이미 설정되었거나 lock 실패
     */
    bool setValue(T value) {
        if (is_set_.exchange(true)) {
            return false;  // 이미 설정됨
        }

        std::unique_lock lock(mutex_, std::defer_lock);
        if (lock.try_lock()) {
            try {
                promise_.set_value(std::move(value));
                return true;
            } catch (...) {
                is_set_ = false;
                throw;
            }
        }

        is_set_ = false;
        return false;  // Lock 실패
    }

    /**
     * @brief Promise에 예외를 설정
     *
     * @param eptr 예외 포인터
     * @return true 예외가 설정됨, false 이미 설정되었거나 lock 실패
     */
    bool setException(std::exception_ptr eptr) {
        if (is_set_.exchange(true)) {
            return false;
        }

        std::unique_lock lock(mutex_, std::defer_lock);
        if (lock.try_lock()) {
            try {
                promise_.set_exception(eptr);
                return true;
            } catch (...) {
                is_set_ = false;
                throw;
            }
        }

        is_set_ = false;
        return false;
    }

    /**
     * @brief 값이 설정되었는지 확인
     */
    bool isSet() const {
        return is_set_.load();
    }

    /**
     * @brief Promise 리셋
     */
    void reset() {
        std::scoped_lock lock(mutex_);
        is_set_ = false;
        promise_ = std::promise<T>();
    }

private:
    std::promise<T> promise_;
    std::atomic<bool> is_set_{false};
    mutable std::mutex mutex_;
};

/**
 * @brief SDO 동기화를 위한 헬퍼 클래스
 *
 * SDO read/write 작업의 동기화를 관리합니다.
 * mutex + condition_variable + running flag를 캡슐화합니다.
 */
class SDOSync {
public:
    SDOSync() = default;

    // Non-copyable
    SDOSync(const SDOSync&) = delete;
    SDOSync& operator=(const SDOSync&) = delete;

    /**
     * @brief SDO 작업 시작 대기
     *
     * 이미 실행 중인 작업이 있다면 완료될 때까지 대기합니다.
     * 작업 시작 시 호출해야 합니다.
     */
    void waitAndLock() {
        std::unique_lock lock(mutex_);
        while (running_) {
            cond_.wait(lock);
        }
        running_ = true;
    }

    /**
     * @brief SDO 작업 완료 알림
     *
     * 작업 완료 시 호출하여 대기 중인 다른 작업을 깨웁니다.
     */
    void unlock() {
        std::scoped_lock lock(mutex_);
        running_ = false;
        cond_.notify_one();
    }

    /**
     * @brief RAII 스타일 락 헬퍼
     */
    class ScopedLock {
    public:
        explicit ScopedLock(SDOSync& sync) : sync_(sync) {
            sync_.waitAndLock();
        }

        ~ScopedLock() {
            sync_.unlock();
        }

        // Non-copyable, non-movable
        ScopedLock(const ScopedLock&) = delete;
        ScopedLock& operator=(const ScopedLock&) = delete;
        ScopedLock(ScopedLock&&) = delete;
        ScopedLock& operator=(ScopedLock&&) = delete;

    private:
        SDOSync& sync_;
    };

    /**
     * @brief 현재 실행 중인지 확인
     */
    bool isRunning() const {
        return running_;
    }

private:
    std::mutex mutex_;
    std::condition_variable cond_;
    bool running_ = false;
};

/**
 * @brief Boot 프로세스 동기화 헬퍼
 *
 * CANopen 노드의 boot 프로세스를 관리합니다.
 */
class BootSync {
public:
    struct BootInfo {
        lely::canopen::NmtState state;
        char status;
        std::string what;
    };

    BootSync() = default;

    // Non-copyable
    BootSync(const BootSync&) = delete;
    BootSync& operator=(const BootSync&) = delete;

    /**
     * @brief Boot 완료 설정
     *
     * @param state NMT 상태
     * @param status Boot 상태 (0 = 성공)
     * @param what 에러 메시지 (있다면)
     */
    void setBoot(lely::canopen::NmtState state, char status, const std::string& what) {
        std::scoped_lock lock(mutex_);
        boot_info_.state = state;
        boot_info_.status = status;
        boot_info_.what = what;

        if (status == 0) {
            booted_.store(true);
        }

        cond_.notify_all();
    }

    /**
     * @brief Boot 완료 대기
     *
     * @param timeout 대기 시간
     * @return true Boot 완료, false 타임아웃
     */
    template<typename Duration>
    bool waitForBoot(Duration timeout) {
        std::unique_lock lock(mutex_);
        return cond_.wait_for(lock, timeout, [this] { return booted_.load(); });
    }

    /**
     * @brief Boot 완료 여부 확인
     */
    bool isBooted() const {
        return booted_.load();
    }

    /**
     * @brief 최근 boot 정보 가져오기
     */
    BootInfo getBootInfo() const {
        std::scoped_lock lock(mutex_);
        return boot_info_;
    }

    /**
     * @brief Boot 상태 리셋
     */
    void reset() {
        std::scoped_lock lock(mutex_);
        booted_.store(false);
        boot_info_ = {};
    }

private:
    std::atomic<bool> booted_{false};
    BootInfo boot_info_;
    mutable std::mutex mutex_;
    std::condition_variable cond_;
};

}  // namespace Utils
}  // namespace NaviCAN
}  // namespace NaviFra

#endif  // NAVIFRA_NAVICAN_SYNC_PROMISE_H
