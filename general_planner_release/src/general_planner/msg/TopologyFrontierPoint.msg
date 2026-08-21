# A public ACTIVE topology node attached to a current KNOWN_FREE-to-UNKNOWN
# map boundary. `position` is the safe topology anchor; `boundary_position`
# is the final known-free sample directly before UNKNOWN.

uint64 id
geometry_msgs/Point position
geometry_msgs/Point boundary_position
geometry_msgs/Vector3 boundary_normal
float32 clearance
float32 boundary_distance
uint32 degree
uint8 state
uint8 unknown_direction_mask
uint8 unknown_direction_count
uint64 node_revision
uint64 last_observed_revision

uint8 STATE_ACTIVE=0
uint8 STATE_HISTORICAL=1

uint8 FACE_NEG_X=1
uint8 FACE_POS_X=2
uint8 FACE_NEG_Y=4
uint8 FACE_POS_Y=8
uint8 FACE_NEG_Z=16
uint8 FACE_POS_Z=32
