#ifndef NAVIFRA_SCAN_STATE_H
#define NAVIFRA_SCAN_STATE_H

#include <string>

namespace NaviFra {
class RobotScanStatus {
public:
    RobotScanStatus();
    ~RobotScanStatus();

    const static std::string KEY;

    enum STATE_SCAN
    {
        STATE_SCAN_ON = 0,
        STATE_SCAN_OFF
    };

public:
    void On();
    void Off();
    STATE_SCAN const getState() { return state_; }

private:
    virtual std::string implName() { return "RobotScanStatus"; }
    void setState(STATE_SCAN state);

private:
    STATE_SCAN state_;
};
}  // namespace NaviFra

#endif