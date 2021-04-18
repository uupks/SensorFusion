# message 812
# Contains the RAWIMU data corrected for gravity, 
# the earth's rotation and estimated sensor errors.
novatel_msgs/CommonHeader header

int32 week
float64 gpssec

# Gyro values are instantaneous and in radians, must
# multiply by datarate to get rad/s.
float64 pitch_rate
float64 roll_rate
float64 yaw_rate

float64 x_accel
float64 y_accel
float64 z_accel
