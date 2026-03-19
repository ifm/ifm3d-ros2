# GetRackRequest.msg

# Selection of the horizontal drop setting. Options: ['left', 'center', 'right']
string horizontal_drop_position "left"

string HORIZONTAL_DROP_POSITION_LEFT="left"
string HORIZONTAL_DROP_POSITION_CENTER="center"
string HORIZONTAL_DROP_POSITION_RIGHT="right"

# Selection of the vertical drop setting. Options: ['interior', 'floor']
string vertical_drop_position "interior"

string VERTICAL_DROP_POSITION_INTERIOR="interior"
string VERTICAL_DROP_POSITION_FLOOR="floor"

# Estimated distance between rack and coordinate system center in meters
float64 depth_hint 1.8

# Estimated z-coordinate of the rack shelf in meters
float64 z_hint -0.4

# Bounding box of VOI
ifm3d_ros2/BoundingBox clearing_volume