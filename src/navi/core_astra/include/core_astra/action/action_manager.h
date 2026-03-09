#ifndef NAVIFRA_ACTION_MANAGER_H
#define NAVIFRA_ACTION_MANAGER_H

#include "core_astra/action/action_base.h"
#include "util/logger.hpp"

#include <Poco/SingletonHolder.h>

#include <atomic>
#include <functional>
#include <mutex>
#include <unordered_map>

namespace NaviFra {

class ActionManager {
public:
    ActionManager() = default;
    static ActionManager& instance()
    {
        static Poco::SingletonHolder<ActionManager> sh;
        return *sh.get();
    }
    // 등록된 액션 로드 (+ keepAlive=true면 즉시 인스턴스 생성)
    void initialize(NaviFra::ActionType type)
    {
        std::lock_guard<std::mutex> lock(mtx_);
        if (shutting_down_) {
            LOG_WARNING("ActionManager::initialize called during shutdown; ignored");
            return;
        }

        const auto& registeredActions = ActionRegistry::instance().getActions();
        auto it = registeredActions.find(type);
        if (it == registeredActions.end())
            return;

        for (const auto& [name, factory] : it->second) {
            registry_[name] = factory;

            // 기본 정책: keepAlive = true (필요 시 setKeepAlive로 바꾸세요)
            if (keepAliveFlags_.find(name) == keepAliveFlags_.end()) {
                keepAliveFlags_[name] = true;
            }

            // keepAlive이면 즉시 생성해서 보관
            if (keepAliveFlags_[name]) {
                if (actions_.find(name) == actions_.end()) {
                    try {
                        actions_[name] = factory();
                        LOG_INFO("Action keepAlive instantiated: %s", name.c_str());
                    }
                    catch (const std::exception& e) {
                        LOG_ERROR("Failed to instantiate action %s: %s", name.c_str(), e.what());
                    }
                }
            }
        }
    }

    void setKeepAlive(const std::string& name, bool keepAlive)
    {
        std::lock_guard<std::mutex> lock(mtx_);
        if (shutting_down_)
            return;

        keepAliveFlags_[name] = keepAlive;

        // true로 바뀌면 즉시 인스턴스 생성
        if (keepAlive) {
            auto itf = registry_.find(name);
            if (itf != registry_.end() && actions_.find(name) == actions_.end()) {
                try {
                    actions_[name] = itf->second();
                    LOG_INFO("Action keepAlive instantiated by setKeepAlive: %s", name.c_str());
                }
                catch (const std::exception& e) {
                    LOG_ERROR("Failed to instantiate action %s: %s", name.c_str(), e.what());
                }
            }
        }
        else {
            // false로 바뀌면 보관 중인 인스턴스 해제(선택)
            actions_.erase(name);
        }
    }

    ActionBase::Ptr getOrCreate(const std::string& name)
    {
        std::lock_guard<std::mutex> lock(mtx_);
        if (shutting_down_)
            return nullptr;

        // 이미 살아 있으면 반환
        auto ita = actions_.find(name);
        if (ita != actions_.end())
            return ita->second;

        // 등록된 액션인지 확인
        auto itf = registry_.find(name);
        if (itf == registry_.end()) {
            LOG_WARNING("getOrCreate: action not registered: %s", name.c_str());
            return nullptr;
        }

        // keepAlive 여부에 따라 생성/보관
        bool keepAlive = false;
        if (auto itk = keepAliveFlags_.find(name); itk != keepAliveFlags_.end())
            keepAlive = itk->second;

        auto inst = itf->second();
        if (keepAlive)
            actions_[name] = inst;
        return inst;
    }

    bool hasAction(const std::string& name) const
    {
        std::lock_guard<std::mutex> lock(mtx_);
        // actions_ 또는 registry_에 있으면 존재로 판단
        return actions_.count(name) || registry_.count(name);
    }

    ActionBase::Ptr getAction(const std::string& name)
    {
        std::lock_guard<std::mutex> lock(mtx_);
        auto ita = actions_.find(name);
        if (ita != actions_.end())
            return ita->second;
        // 보관된 인스턴스가 없으면 nullptr (필요 시 getOrCreate 사용)
        return nullptr;
    }

    void printActions()
    {
        std::lock_guard<std::mutex> lock(mtx_);
        for (auto& [name, inst] : actions_) {
            LOG_INFO("Action : [ %s ], Handler [ %s ]", name.c_str(), inst ? inst->name().c_str() : "(null)");
        }
    }

    // === Graceful Shutdown ===================================================
    // - 여러 번 호출되어도 한 번만 수행 (idempotent)
    // - 액션 인스턴스 해제(소멸자에 정리 로직이 있어야 함)
    // - 필요 시 ActionBase에 onShutdown()/stop() 같은 훅이 있으면 TODO 위치에서 호출
    void shutdown()
    {
        std::unique_lock<std::mutex> lock(mtx_);
        if (shutting_down_) {
            LOG_INFO("ActionManager::shutdown already in progress or done.");
            return;
        }
        shutting_down_ = true;
        LOG_INFO("ActionManager::shutdown begin");

        // 1) 인스턴스들에 사용자 정의 종료 훅 호출 (있다면)
        for (auto& [name, inst] : actions_) {
            if (!inst)
                continue;
            try {
                // TODO: 프로젝트에 맞게 정리 훅을 호출하세요.
                // 예) inst->onShutdown();  또는 inst->stop();  등
                // 기본 구현에선 포인터 해제(소멸자)로 정리에 맡깁니다.
            }
            catch (const std::exception& e) {
                LOG_WARNING("Action %s shutdown hook failed: %s", name.c_str(), e.what());
            }
            catch (...) {
                LOG_WARNING("Action %s shutdown hook threw unknown exception", name.c_str());
            }
        }

        // 2) keepAlive 해제 (새 생성 방지)
        for (auto& kv : keepAliveFlags_) {
            kv.second = false;
        }

        // 3) 인스턴스 해제
        actions_.clear();

        // 4) 레지스트리 정리 (필요 시 남겨도 됨)
        registry_.clear();

        LOG_INFO("ActionManager::shutdown done");
    }

    bool isShuttingDown() const { return shutting_down_.load(); }
    // ========================================================================

private:
    ActionManager(const ActionManager&) = delete;
    ActionManager& operator=(const ActionManager&) = delete;

    mutable std::mutex mtx_;
    std::unordered_map<std::string, std::function<ActionBase::Ptr()>> registry_;
    std::unordered_map<std::string, bool> keepAliveFlags_;
    std::unordered_map<std::string, ActionBase::Ptr> actions_;
    std::atomic<bool> shutting_down_{false};
};

}  // namespace NaviFra

#endif  // NAVIFRA_ACTION_MANAGER_H
