#ifndef NAVIFRA_BATTERY_STATUS_H
#define NAVIFRA_BATTERY_STATUS_H

#include <Poco/JSON/Array.h>

namespace NaviFra {
class BatteryStatus {
public:
    BatteryStatus();
    ~BatteryStatus();

    Poco::JSON::Array::Ptr toArray() const;

    const static std::string KEY;

    // Getter / Setter
    float getVoltage() const;
    void setVoltage(float v);

    float getCurrent() const;
    void setCurrent(float c);

    float getCapacity() const;
    void setCapacity(float c);

    int getEqCycle() const;
    void setEqCycle(int c);

    int getPackOvervoltage() const;
    void setPackOvervoltage(int v);

    int getPackUndervoltage() const;
    void setPackUndervoltage(int v);

    int getCellOvervoltage() const;
    void setCellOvervoltage(int v);

    int getCellUndervoltage() const;
    void setCellUndervoltage(int v);

    int getOverTemperature() const;
    void setOverTemperature(int v);

    int getUnderTemperature() const;
    void setUnderTemperature(int v);

    int getCellVoltageDiffFault() const;
    void setCellVoltageDiffFault(int v);

    int getCellTempDiffFault() const;
    void setCellTempDiffFault(int v);

    int getOvercurrent() const;
    void setOvercurrent(int v);

    int getChargeCompleteSignal() const;
    void setChargeCompleteSignal(int v);

    int getChargingSignal() const;
    void setChargingSignal(int v);

    float getCellMaxVoltage() const;
    void setCellMaxVoltage(float v);

    float getCellMinVoltage() const;
    void setCellMinVoltage(float v);

    float getCellAvgVoltage() const;
    void setCellAvgVoltage(float v);

    int getCellMaxVoltagePos() const;
    void setCellMaxVoltagePos(int v);

    int getCellMinVoltagePos() const;
    void setCellMinVoltagePos(int v);

    float getCellMaxTemp() const;
    void setCellMaxTemp(float v);

    float getCellMinTemp() const;
    void setCellMinTemp(float v);

    float getCellAvgTemp() const;
    void setCellAvgTemp(float v);

    int getMaxTempPos() const;
    void setMaxTempPos(int v);

    int getMinTempPos() const;
    void setMinTempPos(int v);

    float getPeekVoltage() const;
    void setPeekVoltage(float v);

    float getPeekCurrent() const;
    void setPeekCurrent(float v);

    float getPeekTemperature() const;
    void setPeekTemperature(float v);

private:
    float voltage;  // [0] 전압 (V)
    float current;  // [1] 전류 (A)
    float capacity;  // [2] 배터리 용량 (%)
    int eq_cycle;  // [3] Eq_cycle (cycle number)

    int pack_overvoltage;  // [4] 0~3: Normal, Warning, Cut-off, Fault
    int pack_undervoltage;  // [5]
    int cell_overvoltage;  // [6]
    int cell_undervoltage;  // [7]
    int over_temperature;  // [8]
    int under_temperature;  // [9]
    int cell_voltage_diff_fault;  // [10]
    int cell_temp_diff_fault;  // [11]
    int overcurrent;  // [12]
    int charge_complete_signal;  // [13] 0: Normal, 1: Complete
    int charging_signal;  // [14] 0: Normal, 1: Charging

    float cell_max_voltage;  // [15] V
    float cell_min_voltage;  // [16] V
    float cell_avg_voltage;  // [17] V
    int cell_max_voltage_pos;  // [18]
    int cell_min_voltage_pos;  // [19]

    float cell_max_temp;  // [20] ℃
    float cell_min_temp;  // [21] ℃
    float cell_avg_temp;  // [22] ℃
    int max_temp_pos;  // [23]
    int min_temp_pos;  // [24]

    float peek_voltage;  // [25] V
    float peek_current;  // [26] A
    float peek_temperature;  // [27] ℃
};

}  // namespace NaviFra
#endif  // NAVIFRA_BATTERY_STATUS_H