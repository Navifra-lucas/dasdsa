#include "core_agent/manager/initializer_manager.h"
#include "core_agent/manager/publish_manager.h"
#include "core_astra/action/action_manager.h"
#include "core_astra/core_astra_node.h"
#include "core_astra/util/sim_util.hpp"
#include "util/logger.hpp"

#include <ros/ros.h>

#include <atomic>
#include <csignal>
#include <string>
#include <unordered_map>

using namespace NaviFra;

// ---- Graceful shutdown helpers --------------------------------------------
namespace {
std::atomic<bool> g_shutdown_requested{false};

void signalHandler(int sig)
{
    // 여러 번 들어와도 한 번만 처리
    bool expected = false;
    if (g_shutdown_requested.compare_exchange_strong(expected, true)) {
        LOG_INFO("[graceful] signal {} received → request ROS shutdown", sig);
        // ROS 스레드/스핀을 깨우는 표준 진입점
        ros::requestShutdown();
    }
}

void installSignalHandlers()
{
    struct sigaction sa;
    sa.sa_handler = signalHandler;
    sigemptyset(&sa.sa_mask);
    sa.sa_flags = 0;

    // Ctrl+C(SIGINT), 시스템 종료(SIGTERM) 모두 처리
    sigaction(SIGINT, &sa, nullptr);
    sigaction(SIGTERM, &sa, nullptr);
}

// 종료 시 공통 정리 루틴 (atexit에서도 불리도록 별도 함수)
void cleanup()
{
    fprintf(stderr, "[graceful] stopping managers...\n");

    try {
        // 퍼블리셔/스레드 정지
        try {
            PublisherManager::instance().stop();  // TODO: 없으면 프로젝트 메서드명으로 변경
        }
        catch (...) {
            fprintf(stderr, "[graceful] PublisherManager stop failed (ignored) \n");
        }

        // 액션 쪽 리소스(스레드/큐/소켓) 정리
        try {
            ActionManager::instance().shutdown();  // TODO: 없으면 clear()/finalize() 등으로 변경
        }
        catch (...) {
            fprintf(stderr, "[graceful] ActionManager shutdown failed (ignored) \n");
        }

        // 초기화 리소스 해제
        try {
            InitializerManager::instance().finalize();  // TODO: 없는 경우 생략 가능
        }
        catch (...) {
            fprintf(stderr, "[graceful] InitializerManager finalize failed (ignored) \n");
        }
    }
    catch (...) {
        fprintf(stderr, "[graceful] unexpected error while cleaning up \n");
    }

    fprintf(stderr, "[graceful] managers stopped. \n");

    // 2) defaultPool의 모든 쓰레드가 끝날 때까지 대기
    try {
        Poco::ThreadPool::defaultPool().joinAll();
    }
    catch (...) {
        // 여기서는 LOG_* 대신 fprintf 권장 (로거 파괴 순서 문제 회피)
        fprintf(stderr, "[graceful] defaultPool joinAll() failed (ignored)\n");
    }
}
}  // namespace
// ---------------------------------------------------------------------------

int main(int argc, char** argv)
{
    try {
        // ROS가 자체 SIGINT 핸들러를 설치하지 않도록 해서 우리 핸들러로 일원화
        ros::init(argc, argv, "core_astra", ros::init_options::NoSigintHandler);
        init_from_env();

        // 신호 핸들러 설치 (SIGINT/SIGTERM)
        installSignalHandlers();

        // 프로세스 정상 종료시 백업 안전장치
        // std::atexit([]() { cleanup(); });

        CoreAstraNode node;

        // 초기화 및 시작
        InitializerManager::instance().run();
        PublisherManager::instance().start();

        auto& mgr = ActionManager::instance();
        mgr.initialize(ActionType::DEFAULT);
        mgr.printActions();

        // 멀티스레드 스핀으로 콜백 처리 (필요 스레드 수 조정)
        ros::AsyncSpinner spinner(std::max(2u, std::thread::hardware_concurrency() / 4));
        spinner.start();

        LOG_INFO("[core_astra] node started. Waiting for shutdown...");

        // 신호 → requestShutdown() 후 여기서 블록 해제
        ros::waitForShutdown();

        LOG_INFO("[core_astra] ROS shutdown requested. Cleaning up...");
        cleanup();  // 안전하게 한 번 더 호출 (중복 호출은 내부에서 허용/무시)

        LOG_INFO("[core_astra] bye.");

        ros::requestShutdown();
    }
    catch (const Poco::Exception& ex) {
        LOG_ERROR("%s", ex.displayText().c_str());
    }

    return 0;
}
