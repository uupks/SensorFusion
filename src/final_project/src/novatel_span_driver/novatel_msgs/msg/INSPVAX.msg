# message 1465
novatel_msgs/CommonHeader header

# Table 29 in the SPAN on OEM6 manual:
# See: http://www.novatel.com/assets/Documents/Manuals/OM-20000144UM.pdf#page=121
uint32 ins_status
uint32 INS_STATUS_INACTIVE=0
uint32 INS_STATUS_ALIGNING=1
uint32 INS_STATUS_HIGH_VARIANCE=2
uint32 INS_STATUS_SOLUTION_GOOD=3
uint32 INS_STATUS_SOLUTION_FREE=6
uint32 INS_STATUS_ALIGNMENT_COMPLETE=7
uint32 INS_STATUS_DETERMINING_ORIENTATION=8
uint32 INS_STATUS_WAITING_INITIALPOS=9

# Table 30 in the SPAN on OEM6 manual:
# See: http://www.novatel.com/assets/Documents/Manuals/OM-20000144UM.pdf#page=124
uint32 position_type
uint32 POSITION_TYPE_NONE=0
uint32 POSITION_TYPE_SBAS=52
uint32 POSITION_TYPE_PSEUDORANGE_SINGLE_POINT=53
uint32 POSITION_TYPE_PSEUDORANGE_DIFFERENTIAL=54
uint32 POSITION_TYPE_RTK_FLOAT=55
uint32 POSITION_TYPE_RTK_FIXED=56
uint32 POSITION_TYPE_OMNISTAR=57
uint32 POSITION_TYPE_OMNISTAR_HP=58
uint32 POSITION_TYPE_OMNISTAR_XP=59
uint32 POSITION_TYPE_PPP_CONVERGING=73
uint32 POSITION_TYPE_PPP=74

float64 latitude
float64 longitude
float64 altitude

float32 undulation

float64 north_velocity
float64 east_velocity
float64 up_velocity

float64 roll
float64 pitch
float64 azimuth

float32 latitude_std
float32 longitude_std
float32 altitude_std

float32 north_velocity_std
float32 east_velocity_std
float32 up_velocity_std

float32 roll_std
float32 pitch_std
float32 azimuth_std

uint32 extended_status
uint32 EXTENDED_STATUS_POSITION_UPDATE_APPLIED=1
uint32 EXTENDED_STATUS_PHASE_UPDATE_APPLIED=2
uint32 EXTENDED_STATUS_ZUPT_APPLIED=4
uint32 EXTENDED_STATUS_WHEEL_SENSOR_APPLIED=8
uint32 EXTENDED_STATUS_HEADING_UPDATE_APPLIED=16
uint32 EXTENDED_STATUS_INS_SOLUTION_CONVERGED=64

uint16 seconds_since_update
