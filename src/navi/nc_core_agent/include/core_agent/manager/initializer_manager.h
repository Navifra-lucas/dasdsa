#ifndef NAVIFRA_INITIALIZER_MANAGER_H
#define NAVIFRA_INITIALIZER_MANAGER_H

#include <Poco/SingletonHolder.h>

#include <algorithm>
#include <atomic>
#include <mutex>
#include <vector>

namespace NaviFra {

class Initializer {
public:
    virtual void initialize() = 0;
    virtual int priority() const = 0;  // 낮을수록 먼저
    // ↓ 여기: 기본 빈 구현으로 변경 (순수 가상 X)
    virtual void finalize() {}  // 선택적으로 오버라이드
    virtual ~Initializer() = default;
};

class InitializerManager {
public:
    InitializerManager() = default;
    static InitializerManager& instance()
    {
        static Poco::SingletonHolder<InitializerManager> sh;
        return *sh.get();
    }

    void registerInitializer(Initializer* init);
    void run();
    void finalize();

    bool isRunning() const { return ran_.load(); }
    bool isFinalized() const { return finalized_.load(); }

private:
    mutable std::mutex mtx_;
    std::vector<Initializer*> initializers_;
    std::vector<Initializer*> init_order_;
    std::atomic<bool> ran_{false};
    std::atomic<bool> finalized_{false};
};

}  // namespace NaviFra

// 자동 등록 매크로
#define REGISTER_INITIALIZER(CLASSNAME)                                             \
    class CLASSNAME##Registrar {                                                    \
    public:                                                                         \
        CLASSNAME##Registrar()                                                      \
        {                                                                           \
            static CLASSNAME instance;                                              \
            NaviFra::InitializerManager::instance().registerInitializer(&instance); \
        }                                                                           \
    };                                                                              \
    static CLASSNAME##Registrar global_##CLASSNAME##_registrar;

#endif  // NAVIFRA_INITIALIZER_MANAGER_H
