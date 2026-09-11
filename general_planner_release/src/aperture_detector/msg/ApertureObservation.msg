# Geometry only: validity does not certify vehicle passage or free exit space.
# stamp is the newest contributing cloud stamp, never a publication timestamp.
std_msgs/Header header
bool geometry_valid
string reason
geometry_msgs/Point center
geometry_msgs/Vector3 normal
geometry_msgs/Polygon boundary
float64 plane_area
float64 hole_area
float64 area_ratio
bool visual_roi_used
bool depth_known
float64 depth
