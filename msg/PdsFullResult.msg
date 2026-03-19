# PdsFullResult.msg

string raw_json

int32 pds_version_major
int32 pds_version_minor
int32 pds_version_patch

string timestamp

string last_command
string last_command_input

# results, only one will be populated based on lastCommand
ifm3d_ros2/PalletDetectionArray pallet_detection_array
ifm3d_ros2/RackDetection rack_detection
ifm3d_ros2/VolumeCheck volume_check
