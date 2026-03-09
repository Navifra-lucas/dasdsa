#include "NaviCAN/MotorDriver/MotorDriver.h"
#include "NaviCAN/NaviCANCore.h"

#include <termios.h>
#include <unistd.h>

#include <csignal>

using namespace std::chrono_literals;
using namespace lely;
using namespace NaviFra;

std::shared_ptr<NaviCANCore> core;

// 비차단 입력 설정 함수 (Linux 환경)
void enableNonBlockingInput()
{
    struct termios t;
    tcgetattr(STDIN_FILENO, &t);
    t.c_lflag &= ~ICANON;  // 비캐논 모드
    t.c_lflag &= ~ECHO;  // 입력 문자 숨김
    tcsetattr(STDIN_FILENO, TCSANOW, &t);
}

// 비차단 입력 복구 함수
void disableNonBlockingInput()
{
    struct termios t;
    tcgetattr(STDIN_FILENO, &t);
    t.c_lflag |= ICANON;  // 캐논 모드 복구
    t.c_lflag |= ECHO;  // 입력 문자 표시
    tcsetattr(STDIN_FILENO, TCSANOW, &t);
}

std::atomic<bool> run_ = false;

// 사용자 입력 처리 함수
void handleUserInput()
{
    enableNonBlockingInput();  // 비차단 입력 활성화
    double speed = 0;

    while (true) {
        if (std::cin.peek() != EOF) {  // 입력 버퍼에 데이터가 있는지 확인
            char command = std::cin.get();

            if (command == 'o') {  // "on" 명령
                std::cout << "\nAttempting to turn servo on..." << std::endl;
                if (core->enable(1)) {
                    std::cout << "Servo is now ON." << std::endl;
                }
                else {
                    std::cerr << "Failed to turn servo on." << std::endl;
                }
            }
            else if (command == 'f') {  // "off" 명령
                std::cout << "\nAttempting to turn servo off..." << std::endl;
                // if (core.handleShutdown()) {
                //     std::cout << "Servo is now OFF." << std::endl;
                // }
                // else {
                //     std::cerr << "Failed to turn servo off." << std::endl;
                // }
            }
            else if (command == 'w') {
                speed += 10000;
                core->setTarget(1, speed);
                std::cout << "Speed increased to: " << speed << std::endl;
            }
            else if (command == 's') {
                speed -= 10000;
                core->setTarget(1, speed);
                std::cout << "Speed decreased to: " << speed << std::endl;
            }
            else if (command == 'q') {  // "quit" 명령
                std::cout << "\nExiting user input thread." << std::endl;
                disableNonBlockingInput();  // 비차단 입력 복구
                std::exit(0);
                break;
            }
            else if (command == 'm') {
                // core->SwitchMode(1, IMotorDriver::PROFILED_VELOCITY);
                // if (motor.enterModeAndWait(IMotorDriver::PROFILED_VELOCITY)) {
                //     std::cout << "set velocity mode Ok" << std::endl;
                // };
            }
            else if (std::cin.peek() == '\033') {  // 특수 키 (ESC)
                std::cin.get();  // Skip ESC
                char nextChar = std::cin.get();  // Skip '['

                if (nextChar == '[') {
                    char arrowKey = std::cin.get();
                    if (arrowKey == 'A') {  // Up arrow
                        speed += 10;
                        // motor.setTarget(speed);
                        std::cout << "Speed increased to: " << speed << " (Up arrow)" << std::endl;
                    }
                    else if (arrowKey == 'B') {  // Down arrow
                        speed -= 10;
                        // motor.setTarget(speed);
                        std::cout << "Speed decreased to: " << speed << " (Down arrow)" << std::endl;
                    }
                }
            }
            else {
                std::cerr << "\nUnknown command. Use 'o' (on), 'f' (off), 'w' (increase speed), 's' (decrease speed), 'q' (quit)."
                          << std::endl;
            }
        }

        // 화살표 키 입력 처리
        if (std::cin.peek() == '\033') {  // 특수 키 (ESC)
            std::cin.get();  // Skip ESC
            char nextChar = std::cin.get();  // Skip '['

            if (nextChar == '[') {
                char arrowKey = std::cin.get();
                if (arrowKey == 'A') {  // Up arrow
                    speed += 10;
                    // motor.setTarget(speed);
                    std::cout << "Speed increased to: " << speed << " (Up arrow)" << std::endl;
                }
                else if (arrowKey == 'B') {  // Down arrow
                    speed -= 10;
                    // motor.setTarget(speed);
                    std::cout << "Speed decreased to: " << speed << " (Down arrow)" << std::endl;
                }
            }
        }

        //   std::cout << "  currnet Motor Speed " << motor.getSpeed() << std::endl;

        std::this_thread::sleep_for(std::chrono::milliseconds(100));  // 딜레이
    }
    disableNonBlockingInput();
}

int main()
{
    try {
        core.reset(new NaviCANCore());
        core->initialize();

        // core->startThread();
        std::thread input_thread(handleUserInput);

        std::this_thread::sleep_for(2000ms);
        // core->enable(1);
        // core->SwitchMode(1, IMotorDriver::PROFILED_VELOCITY);

        while (true) {
            std::this_thread::sleep_for(100ms);
        }

        input_thread.join();

        core->finalize();
    }
    catch (...) {
    }

    return 0;
}