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
    static void SyncExecIntervalTime_(const std::chrono::steady_clock::time_point& chrono_time, const float& f_interval_time_ms);

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
