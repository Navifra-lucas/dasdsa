#include "nc_wia_agent/data/battery_status.h"

using namespace NaviFra;

const std::string BatteryStatus::KEY = "BatteryStatus";

BatteryStatus::BatteryStatus()
    : voltage(0)
    , current(0)
    , capacity(0)
    , eq_cycle(0)
    , pack_overvoltage(0)
    , pack_undervoltage(0)
    , cell_overvoltage(0)
    , cell_undervoltage(0)
    , over_temperature(0)
    , under_temperature(0)
    , cell_voltage_diff_fault(0)
    , cell_temp_diff_fault(0)
    , overcurrent(0)
    , charge_complete_signal(0)
    , charging_signal(0)
    , cell_max_voltage(0)
    , cell_min_voltage(0)
    , cell_avg_voltage(0)
    , cell_max_voltage_pos(0)
    , cell_min_voltage_pos(0)
    , cell_max_temp(0)
    , cell_min_temp(0)
    , cell_avg_temp(0)
    , max_temp_pos(0)
    , min_temp_pos(0)
    , peek_voltage(0)
    , peek_current(0)
    , peek_temperature(0)
{
}

BatteryStatus::~BatteryStatus()
{
}

Poco::JSON::Array::Ptr BatteryStatus::toArray() const
{
    Poco::JSON::Array::Ptr arr = new Poco::JSON::Array;

    arr->add(voltage);
    arr->add(current);
    arr->add(capacity);
    arr->add(eq_cycle);

    arr->add(pack_overvoltage);
    arr->add(pack_undervoltage);
    arr->add(cell_overvoltage);
    arr->add(cell_undervoltage);
    arr->add(over_temperature);
    arr->add(under_temperature);
    arr->add(cell_voltage_diff_fault);
    arr->add(cell_temp_diff_fault);
    arr->add(overcurrent);
    arr->add(charge_complete_signal);
    arr->add(charging_signal);

    arr->add(cell_max_voltage);
    arr->add(cell_min_voltage);
    arr->add(cell_avg_voltage);
    arr->add(cell_max_voltage_pos);
    arr->add(cell_min_voltage_pos);

    arr->add(cell_max_temp);
    arr->add(cell_min_temp);
    arr->add(cell_avg_temp);
    arr->add(max_temp_pos);
    arr->add(min_temp_pos);

    arr->add(peek_voltage);
    arr->add(peek_current);
    arr->add(peek_temperature);

    return arr;
}

// Getters
float BatteryStatus::getVoltage() const
{
    return voltage;
}
void BatteryStatus::setVoltage(float v)
{
    voltage = v;
}

float BatteryStatus::getCurrent() const
{
    return current;
}
void BatteryStatus::setCurrent(float c)
{
    current = c;
}

float BatteryStatus::getCapacity() const
{
    return capacity;
}
void BatteryStatus::setCapacity(float c)
{
    capacity = c;
}

int BatteryStatus::getEqCycle() const
{
    return eq_cycle;
}
void BatteryStatus::setEqCycle(int c)
{
    eq_cycle = c;
}

int BatteryStatus::getPackOvervoltage() const
{
    return pack_overvoltage;
}
void BatteryStatus::setPackOvervoltage(int v)
{
    pack_overvoltage = v;
}

int BatteryStatus::getPackUndervoltage() const
{
    return pack_undervoltage;
}
void BatteryStatus::setPackUndervoltage(int v)
{
    pack_undervoltage = v;
}

int BatteryStatus::getCellOvervoltage() const
{
    return cell_overvoltage;
}
void BatteryStatus::setCellOvervoltage(int v)
{
    cell_overvoltage = v;
}

int BatteryStatus::getCellUndervoltage() const
{
    return cell_undervoltage;
}
void BatteryStatus::setCellUndervoltage(int v)
{
    cell_undervoltage = v;
}

int BatteryStatus::getOverTemperature() const
{
    return over_temperature;
}
void BatteryStatus::setOverTemperature(int v)
{
    over_temperature = v;
}

int BatteryStatus::getUnderTemperature() const
{
    return under_temperature;
}
void BatteryStatus::setUnderTemperature(int v)
{
    under_temperature = v;
}

int BatteryStatus::getCellVoltageDiffFault() const
{
    return cell_voltage_diff_fault;
}
void BatteryStatus::setCellVoltageDiffFault(int v)
{
    cell_voltage_diff_fault = v;
}

int BatteryStatus::getCellTempDiffFault() const
{
    return cell_temp_diff_fault;
}
void BatteryStatus::setCellTempDiffFault(int v)
{
    cell_temp_diff_fault = v;
}

int BatteryStatus::getOvercurrent() const
{
    return overcurrent;
}
void BatteryStatus::setOvercurrent(int v)
{
    overcurrent = v;
}

int BatteryStatus::getChargeCompleteSignal() const
{
    return charge_complete_signal;
}
void BatteryStatus::setChargeCompleteSignal(int v)
{
    charge_complete_signal = v;
}

int BatteryStatus::getChargingSignal() const
{
    return charging_signal;
}
void BatteryStatus::setChargingSignal(int v)
{
    charging_signal = v;
}

float BatteryStatus::getCellMaxVoltage() const
{
    return cell_max_voltage;
}
void BatteryStatus::setCellMaxVoltage(float v)
{
    cell_max_voltage = v;
}

float BatteryStatus::getCellMinVoltage() const
{
    return cell_min_voltage;
}
void BatteryStatus::setCellMinVoltage(float v)
{
    cell_min_voltage = v;
}

float BatteryStatus::getCellAvgVoltage() const
{
    return cell_avg_voltage;
}
void BatteryStatus::setCellAvgVoltage(float v)
{
    cell_avg_voltage = v;
}

int BatteryStatus::getCellMaxVoltagePos() const
{
    return cell_max_voltage_pos;
}
void BatteryStatus::setCellMaxVoltagePos(int v)
{
    cell_max_voltage_pos = v;
}

int BatteryStatus::getCellMinVoltagePos() const
{
    return cell_min_voltage_pos;
}
void BatteryStatus::setCellMinVoltagePos(int v)
{
    cell_min_voltage_pos = v;
}

float BatteryStatus::getCellMaxTemp() const
{
    return cell_max_temp;
}
void BatteryStatus::setCellMaxTemp(float v)
{
    cell_max_temp = v;
}

float BatteryStatus::getCellMinTemp() const
{
    return cell_min_temp;
}
void BatteryStatus::setCellMinTemp(float v)
{
    cell_min_temp = v;
}

float BatteryStatus::getCellAvgTemp() const
{
    return cell_avg_temp;
}
void BatteryStatus::setCellAvgTemp(float v)
{
    cell_avg_temp = v;
}

int BatteryStatus::getMaxTempPos() const
{
    return max_temp_pos;
}
void BatteryStatus::setMaxTempPos(int v)
{
    max_temp_pos = v;
}

int BatteryStatus::getMinTempPos() const
{
    return min_temp_pos;
}
void BatteryStatus::setMinTempPos(int v)
{
    min_temp_pos = v;
}

float BatteryStatus::getPeekVoltage() const
{
    return peek_voltage;
}
void BatteryStatus::setPeekVoltage(float v)
{
    peek_voltage = v;
}

float BatteryStatus::getPeekCurrent() const
{
    return peek_current;
}
void BatteryStatus::setPeekCurrent(float v)
{
    peek_current = v;
}

float BatteryStatus::getPeekTemperature() const
{
    return peek_temperature;
}
void BatteryStatus::setPeekTemperature(float v)
{
    peek_temperature = v;
}