# PalletDetection.msg

std_msgs/Header header

# Position and size of the center beam, the left pocket and the right pocket respectively.
# For each, the width and height is provided, as well as the coordinates of the center of
# the pocket or beam along the X, Y and Z axis.
# For single pocket pallets, the center, left and right results are all the same.
geometry_msgs/PoseStamped center_position
float64 center_height
float64 center_width
geometry_msgs/PoseStamped left_position
float64 left_height
float64 left_width
geometry_msgs/PoseStamped right_position
float64 right_height
float64 right_width