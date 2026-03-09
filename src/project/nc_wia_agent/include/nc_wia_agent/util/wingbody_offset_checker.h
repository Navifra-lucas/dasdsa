#ifndef NAVIFRA_WINGBODY_OFFSET_H
#define NAVIFRA_WINGBODY_OFFSET_H

#include <Poco/Timestamp.h>

#include <vector>
#include "nc_wia_agent/data/robot_basic_status.h"

namespace NaviFra {

class WingbodyOffsetChecker {
public:
    WingbodyOffsetChecker();
    ~WingbodyOffsetChecker();

    bool isNeedCompare() const;

    bool compare();

    void reset();

    void update(float x, float y, float theta);
    
    bool isTimeout() const;
    
    void getData(float& x, float& y, float& theta) const;

private:
    
    float f_received_x_;
    float f_received_y_;
    float f_received_theta_;
    
    bool b_has_data_;
    
    Poco::Timestamp last_publish_time_;
    static const int TIMEOUT_MS = 5000;
};

}  // namespace NaviFra

#endif  // NAVIFRA_WINGBODY_OFFSET_H
