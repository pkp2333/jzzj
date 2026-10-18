std_msgs/Header header
string session_id
uint64 command_id
uint8 ACQUIRE=0
uint8 MOVE=1
uint8 HOLD=2
uint8 LOCK=3
uint8 RELEASE=4
uint8 operation
float64 vx
float64 vy
# Only explicit operator recovery may release a locked/faulted base.
bool operator_confirmed
