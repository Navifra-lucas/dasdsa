#ifndef _NAVIFRA_TIMER_H
#define _NAVIFRA_TIMER_H

#include <Poco/Stopwatch.h>

namespace NaviFra {
namespace Util {
class Timer {
public:
    Timer(long timeoutInMilliseconds)
        : timeoutInMilliseconds_(timeoutInMilliseconds)
        , stopwatch_()
    {
    }

    void start(long timeoutInMilliseconds)
    {
        timeoutInMilliseconds_ = timeoutInMilliseconds;
        stopwatch_.restart();
    }

    bool isTimeout()
    {
        return stopwatch_.elapsed() >= timeoutInMilliseconds_ * 1000;  // 밀리초 단위로 변환
    }

private:
    long timeoutInMilliseconds_;
    Poco::Stopwatch stopwatch_;
};
}  // namespace Util
}  // namespace Navifra
#endif