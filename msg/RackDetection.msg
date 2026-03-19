# RackDetection.msg

std_msgs/Header header

# Position of the origin of the rack coordinate system.
geometry_msgs/PoseStamped position

# Type of the rack. Either “right” or “left”, indicating beam side.
string side

# Volume in which the algorithm searches for an horizontal beam.
ifm3d_ros2/BoundingBoxStamped horizontal_beam_voi
	
# Volume in which the algorithm searches for a vertical upright.
ifm3d_ros2/BoundingBoxStamped upright_voi
	
# The number of pixels in the volume of interest.
# Typically, this is used to verify that the rack is empty before dropping a load.
uint64 number_of_pixels

# The horizontal beam of the rack grid location could not be segmented.
bool no_beam_flag
# Multiple horizontal beam candidates were segmented, the most plausible was selected.
bool multiple_beams_flag
# The threshold of pixel coverage over the surface area of the beam was not met.
bool beam_coverage_flag
# A vertical upright was not detected, the rack frame was established based on the segmented beam,
# sweeping volume, and (optionally) the floor.
bool no_upright_flag
# Multiple upright candidates (on the anchor side of interest) were segmented, the most plausible
# was selected.
bool multiple_uprights_flag
# The threshold of pixel coverage over the surface area of the upright was not met.
bool upright_coverage_flag
# The relative position of the upright candidate and the horizontal beam is implausible.
bool no_join_flag
# The origin of the computed rack frame is outside of an expected tolerance (indicative of
# a beam-only localization anchoring to an obstacle).
bool bad_transform_flag
# An obstacle was detected within the shelf sweeping volume with respect to the established rack frame.
bool shelf_obstacle_flag