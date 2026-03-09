# Voltage and Current
float32 f32_cum_volt
float32 f32_gather_volt	
float32 f32_current
float32 f32_pack_volt
float32 f32_pack_current

# State of Charge / Health
float32 f32_soc
float32 f32_soh

# Firmware
uint16 n_firmware_version

# Cell Voltages
float32[24] f32_cell_voltage

# Max/Min/Avg per pack
float32 f32_max_cell_v
float32 f32_min_cell_v
float32 f32_avg_cell_v

int8[8] n_temp
float32 f32_temp
float32 f32_max_temp
float32 f32_min_temp
float32 f32_avg_temp

# Cell index
uint8 un8_max_cell_no
uint8 un8_min_cell_no
uint8 un8_max_v_cell_no
uint8 un8_min_v_cell_no
uint8 un8_max_temp_pos
uint8 un8_min_temp_pos

# Relay & charging states
bool b_main_relay
bool b_sleep_request
bool b_sleep_command
bool b_breaker_off
bool b_shutdown_request
bool b_sleep_flag
bool b_charging
bool b_charge_complete

# Fault flags
bool b_relay_fail
bool b_fuse_fail
bool b_relay_fusing_fail
bool b_eeprom_fail
bool b_precharge_fail
bool b_AFIC_fail
bool b_ct_mismatch
bool b_pvs_mismatch
bool b_ct_fail
bool b_pvs_fail
bool b_temp_sensor_fail
bool b_cell_volt_sensor_fail
bool b_ct_zeroset_fail
bool b_pvs_zeroset_fail

# Error flags
uint8 n_pov_flag
uint8 n_puv_flag
uint8 n_cov_flag
uint8 n_cuv_flag
uint8 n_osoc_flag
uint8 n_usoc_flag
uint8 n_ot_flag
uint8 n_ut_flag
uint8 n_odv_flag
uint8 n_odt_flag
uint8 n_oc_flag
uint8 n_upv_flag
uint8 n_ocv_flag
uint8 n_ucv_flag
uint8 n_packovervoltage_flag
