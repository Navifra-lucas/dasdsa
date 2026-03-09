#ifndef NAVIFRA_ROBOT_CALIBRATION_H
#define NAVIFRA_ROBOT_CALIBRATION_H

#include <string>
#include <vector>

namespace NaviFra {
class RobotCalibration {
public:
    RobotCalibration();
    virtual ~RobotCalibration();

    const static std::string KEY;

public:
    std::string currentType();
    void setCurrentType(std::string type);
    std::string calibrationCommand(const std::string type);
    void appendResult(float res);
    void clear();
    size_t size();
    float at(size_t index);

private:
    std::string currnt_type_;
    std::vector<float> result_;
};
}  // namespace NaviFra
#endif