# Selected-target accepted observation, before filter update but after frontend constraints.
std_msgs/Header header
int32 target_id
geometry_msgs/Point accepted_position
geometry_msgs/Vector3 velocity_before_update
geometry_msgs/Vector3 velocity_after_update
geometry_msgs/Vector3 velocity_after_outer_rules
bool stop_hypothesis
bool far_reacquisition
bool position_snapped
bool semantic_reset
