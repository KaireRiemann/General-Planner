# Semantic frontier cluster exposed to upper layers / release consumers.
# This is not a voxel-center cloud; it mirrors ClusterInfo used by exploration.

int32 id

geometry_msgs/Point center
geometry_msgs/Vector3 normal
geometry_msgs/Pose best_viewpoint
geometry_msgs/Point box_min
geometry_msgs/Point box_max

uint8 state
bool is_reachable
bool is_dormant
bool is_new_cluster

float32 visible_gain
float32 score
float32 distance

uint8 STATE_ACTIVE=0
uint8 STATE_SELECTED=1
uint8 STATE_PENDING_RETRY=2
uint8 STATE_SUSPENDED=3
uint8 STATE_DEFERRED_BRANCH=4
uint8 STATE_VISITED=5
uint8 STATE_BLACKLISTED=6
