# Latched, full snapshot of frontier-equivalent persistent topology points.

std_msgs/Header header
uint64 world_epoch
uint64 map_revision
uint64 topology_revision
uint32 active_count
uint32 expandable_count
TopologyExpansionPoint[] points
