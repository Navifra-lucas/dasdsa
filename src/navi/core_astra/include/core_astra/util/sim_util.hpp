#ifndef NAVIFRA_SIM_UTIL_HPP
#define NAVIFRA_SIM_UTIL_HPP

#pragma once
#include "core_agent/util/config.h"
#include "core_astra/action/action_base.h"
#include "util/logger.hpp"

#include <atomic>
#include <chrono>
#include <cstdlib>
#include <random>
#include <thread>

namespace NaviFra {

// 환경변수에서 1회 로드 (프로세스 시작 시 호출 권장)
inline void init_from_env()
{
    auto getenv_or = [](const char* k, const char* defv) {
        const char* v = std::getenv(k);
        return (v && *v) ? v : defv;
    };
    int min_ms = std::atoi(getenv_or("ASTRA_SIM_MIN_DELAY_MS", "0"));
    int max_ms = std::atoi(getenv_or("ASTRA_SIM_MAX_DELAY_MS", "0"));
    double drop = std::atof(getenv_or("ASTRA_SIM_DROP_PROB", "0.0"));

    if (min_ms < 0)
        min_ms = 0;
    if (max_ms < min_ms)
        max_ms = min_ms;

    Config::instance().setInt("min_ms", min_ms);
    Config::instance().setInt("max_ms", max_ms);
    Config::instance().setDouble("drop", drop);
}

// 액션 컨텍스트 인터페이스 최소 요구 (cancelled 포인터/ok/error/ name())
template <typename Ctx>
inline bool simulate_delay_or_drop(Ctx& ctx, const char* action_name)
{
    const int min_ms = Config::instance().getInt("min_ms", 0);
    const int max_ms = Config::instance().getInt("max_ms", 0);
    const double drop = Config::instance().getDouble("drop", 0.0);

    if ((min_ms == 0 && max_ms == 0) && drop == 0.0)
        return false;  // 아무 것도 안 함

    // RNG는 스레드당 1회 초기화
    static thread_local std::mt19937 rng{std::random_device{}()};
    std::uniform_int_distribution<int> dly(min_ms, max_ms);
    std::uniform_real_distribution<double> prob(0.0, 1.0);

    const int delay_ms = dly(rng);
    const bool do_drop = (prob(rng) < drop);

    // 딜레이: 50ms 슬라이스로 끊어 취소 체크
    int slept = 0;
    while (slept < delay_ms) {
        if (ctx.cancelled && *ctx.cancelled) {
            LOG_WARNING("[%s] cancelled during simulated delay (%d/%d ms)", action_name, slept, delay_ms);
            ctx.error(action_name, NaviFra::ActionError::Cancelled, "Request cancelled");
            return true;  // 여기서 처리 종료
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        slept += 50;
    }
    if (delay_ms > 0) {
        LOG_INFO("[%s] simulated delay: %d ms", action_name, delay_ms);
    }

    if (do_drop) {
        LOG_WARNING("[%s] simulated DROP: no reply will be sent", action_name);
        return true;  // 드롭: 호출 측에서 그냥 return 하면 됨
    }

    return false;  // 시뮬레이션만 하고 정상 흐름 계속
}

}  // namespace NaviFra

#endif  // NAVIFRA_SIM_UTIL_HPP