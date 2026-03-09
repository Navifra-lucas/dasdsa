int64 NONE=0
int64 POI=1
int64 SPIN_TURN=2

int64 POLY1=1
int64 POLY2=2
int64 POLY3=3
int64 POLY4=4
int64 POLY5=5

int64 WAYPOINT=0
int64 CHARGER=1

string id
string name
int64 type
int64 mission_type
geometry_msgs/Point position
geometry_msgs/Quaternion orientation

bool alarm_flag
float32 alarm_mm
float32 alarm_deg
int64 polytype
float32 barcode
string action
string custom