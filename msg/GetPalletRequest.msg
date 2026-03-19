# GetPalletRequest.msg

# Estimated distance between pallet and calibrated coordinate system center in meters.
# Set to <=0 for automatic detection.
float64 depth_hint 1.5

# Index of the pallet parameters.
# Options 0-4 are predefined, options 5-9 can be set by the user.
uint8 pallet_index 0

uint8 BLOCK_PALLET=0
uint8 STRINGER_PALLET=1
uint8 EPAL_SIDE_PALLET=2
uint8 WIRE_CAGE_PALLET_FRONT=3
uint8 WIRE_CAGE_PALLET_SIDE=4

# Set the order in the list of detected pallets.
# Options: ['scoreDescending', 'zDescending', 'zAscending', 'yDescending', 'yAscending']
string pallet_order "scoreDescending"
string SCORE_DESCENDING="scoreDescending"
string Z_DESCENDING="zDescending"
string Z_ASCENDING="zAscending"
string Y_DESCENDING="yDescending"
string Y_ASCENDING="yAscending"