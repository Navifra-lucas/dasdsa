#범퍼 & EMS 관련 버튼식
bool front_bumper_switch
bool rear_bumper_switch
bool emergency

bool ossd_signal
bool sto_signal
bool brake_release_feedback
bool manual_charge_on
bool quick_stop_status

bool reset_switch
bool brake_release_switch

#오토, 매뉴얼 관련
bool io_mode_select_1 # 자동
bool io_mode_select_2 # 수동

bool drive_power_relay_signal

#릴레이가 켜졌다 -> 충전준비가 됐다. 
bool rear_charge_relay_signal

#리프트 상태 관련
bool front_up_px
bool front_down_px
bool rear_up_px
bool rear_down_px

bool front_motor_state
bool rear_motor_state
bool front_motor_alarm
bool rear_motor_alarm

uint8 safety_input_io_ib0
uint8 safety_input_io_qb6
uint8 safety_input_io_qb12
uint8 safety_input_io_qb18
uint8 safety_input_io_ib24
uint8 safety_input_io_ib25
uint8 safety_input_io_qb26

#버전정보
uint8 version_major
uint8 version_minor
uint8 version_patch
string plc_version
