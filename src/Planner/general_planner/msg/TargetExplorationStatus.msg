std_msgs/Header header
uint64 task_epoch
string task_id

# Only SUCCEEDED and READY permit dispatching a new target task.
uint8 RUNNING=0
uint8 SUCCEEDED=1
uint8 FAILED=2
uint8 READY=3
uint8 result
string reason
