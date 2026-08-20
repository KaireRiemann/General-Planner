# A persistent global-topology node exposed as a frontier-equivalent point.
# `is_expandable` means that at least one selected portal face has no active
# topology edge to an adjacent region. It is a topology-growth candidate, not
# a guarantee that the corresponding map direction is traversable.

uint64 id
geometry_msgs/Point position
float32 clearance
uint32 degree
uint8 state
bool is_expandable
uint8 portal_mask
uint8 expansion_mask
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
