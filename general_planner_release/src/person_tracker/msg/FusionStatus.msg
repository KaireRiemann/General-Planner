uint8 WAITING=0
uint8 LOCKED=1
uint8 SUSPECT=2
uint8 LOST=3

uint8 UNKNOWN=0
uint8 OUT_OF_VIEW=1
uint8 CONSISTENT=2
uint8 CONFLICT=3

std_msgs/Header header
uint8 fusion_state
string fusion_state_text
uint8 semantic_evidence
string semantic_evidence_text
bool camera_projection_valid
bool predicted_in_fov
bool yolo_target_present
bool yolo_lidar_component_valid
bool automatic_reselection_eligible
bool reselection_ready
uint32 conflict_count
uint32 confirmation_count
float64 semantic_distance
float64 semantic_age
int32 target_id
string reason
