# One atomic request for the exploration adapter. In target mode the target
# pose and START bit are delivered in the same callback, so a task can never
# begin global planning before its mission target has been stored.
std_msgs/Header header

uint64 task_epoch
string task_id
uint8 mission_mode
bool has_target
geometry_msgs/PoseStamped target
bool start

uint8 MODE_COVERAGE=0
uint8 MODE_TARGET=1
