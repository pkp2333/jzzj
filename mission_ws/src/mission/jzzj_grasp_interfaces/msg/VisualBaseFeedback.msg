std_msgs/Header header
string session_id
int64 track_id
# Translation estimate from a stationary fruit and a rigid body-mounted camera.
# This is NOT full odometry: yaw and independent motion of the fruit are unobserved.
float64 x
float64 y
float64[3] fruit_position_m
bool valid
string diagnostic
