# All positions are expressed in header.frame_id. horizontal_valid explicitly
# gates x/y tracking so height-only commands cannot activate placeholder x/y.
std_msgs/Header header
string[4] foot_names
geometry_msgs/Point[4] lift_off_positions
geometry_msgs/Point[4] touch_down_positions
float64[4] minimum_apex_heights
bool[4] valid
bool[4] horizontal_valid
# Paper-style convex inner approximation of the selected steppable region.
# Vertices are counter-clockwise in header.frame_id.  The plane origin and
# normal define the touchdown surface; an empty polygon keeps legacy point
# target behavior.
geometry_msgs/Point[4] plane_origins
geometry_msgs/Vector3[4] plane_normals
geometry_msgs/Polygon[4] convex_regions
bool[4] region_valid
