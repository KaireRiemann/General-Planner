# Latched full snapshot. Every point is an active topology node with at least
# one evidence-backed KNOWN_FREE-to-UNKNOWN direction.

std_msgs/Header header
uint64 world_epoch
uint64 map_revision
uint64 topology_revision
uint32 topology_node_count
uint32 frontier_count
TopologyFrontierPoint[] points
