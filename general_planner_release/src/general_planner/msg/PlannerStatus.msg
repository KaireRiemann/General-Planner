std_msgs/Header header
uint64 transition_id
uint64 task_epoch
uint64 world_epoch
uint64 map_revision
uint64 topo_revision
uint64 accepted_request_id

string task_id
uint8 active_mode
uint8 requested_mode
uint8 phase
uint16 mode_state
uint8 task_result

# Human-readable mirrors of the enums above for rostopic echo / upper layers.
string active_mode_str
string requested_mode_str
string phase_str
string mode_state_str
string command_owner_str
string task_result_str

bool stable_hover
bool ready_for_new_task
bool odom_valid
bool map_ready
bool topology_ready
uint8 command_owner
float32 speed_mps
float32 yaw_rate_rps
string reason

uint8 MODE_HOLD=0
uint8 MODE_STATE2STATE=1
uint8 MODE_EXPLORATION=2
uint8 MODE_EMERGENCY_STOP=3
uint8 MODE_TARGET_EXPLORATION=4
uint8 MODE_GATE=5

uint8 PHASE_BOOT=0
uint8 PHASE_WAITING_INPUT=1
uint8 PHASE_PLANNING=2
uint8 PHASE_EXECUTING=3
uint8 PHASE_BRAKING=4
uint8 PHASE_HOLD_VERIFY=5
uint8 PHASE_STABLE_HOLD=6
uint8 PHASE_FAILED=7
uint8 PHASE_EMERGENCY=8

uint8 RESULT_NONE=0
uint8 RESULT_SUCCEEDED=1
uint8 RESULT_FAILED=2
uint8 RESULT_CANCELED=3
uint8 RESULT_BLOCKED=4

uint8 OWNER_HOLD=0
uint8 OWNER_STATE2STATE=1
uint8 OWNER_EXPLORATION=2
uint8 OWNER_GATE=3

uint16 MODE_STATE_UNKNOWN=0
uint16 MODE_STATE_S2S_WAIT_GOAL=1
uint16 MODE_STATE_S2S_GENERATE_TRAJ=2
uint16 MODE_STATE_S2S_FOLLOW_TRAJ=3
uint16 MODE_STATE_S2S_YAWING=4
uint16 MODE_STATE_S2S_EMER_STOP=5
uint16 MODE_STATE_EXP_INIT=10
uint16 MODE_STATE_EXP_WAIT_TRIGGER=11
uint16 MODE_STATE_EXP_PLAN_TRAJ=12
uint16 MODE_STATE_EXP_EXEC_TRAJ=13
uint16 MODE_STATE_EXP_REORIENT=14
uint16 MODE_STATE_EXP_CAUTION=15
uint16 MODE_STATE_EXP_PAUSING=16
uint16 MODE_STATE_EXP_PAUSED=17
uint16 MODE_STATE_EXP_FINISH=18
uint16 MODE_STATE_EXP_WAIT_TARGET=19
uint16 MODE_STATE_HOLD_IDLE=20
uint16 MODE_STATE_GATE_WAIT_START=30
uint16 MODE_STATE_GATE_EXECUTING=31
uint16 MODE_STATE_GATE_END_VERIFY=32
uint16 MODE_STATE_GATE_COMPLETE=33
