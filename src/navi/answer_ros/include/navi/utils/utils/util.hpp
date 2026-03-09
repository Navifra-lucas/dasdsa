#ifndef NAVIFRA_UTIL_HPP_
#define NAVIFRA_UTIL_HPP_

#include <math.h>

#include <chrono>
#include <fstream>
#include <iostream>
#include <thread>
#include <vector>

namespace NaviFra {
class Util {
public:
    Util();
    virtual ~Util();
    static void SyncExecIntervalTime_(const std::chrono::steady_clock::time_point& chrono_time, const float& f_interval_time_ms)
    {
        std::chrono::duration<double> sec = std::chrono::steady_clock::now() - chrono_time;
        float f_exe_time = sec.count() * 1000;
        if (f_interval_time_ms > f_exe_time) {
            int n_sleep_time_ms = (f_interval_time_ms - f_exe_time);
            std::this_thread::sleep_for(std::chrono::milliseconds(n_sleep_time_ms));
        }
        else {
            std::this_thread::yield();
        }
    }

    // Normalizes the angle in radians between [-pi and pi).
    template <typename T>
    static T NormalizeAngle(const T& angle_radians)
    {
        T two_pi(2.0 * M_PI);
        return angle_radians - two_pi * std::floor((angle_radians + T(M_PI)) / two_pi);
    }
};

}  // namespace NaviFra
#endif
