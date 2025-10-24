std_msgs/Header header

# https://api.ifm3d.com/stable/cpp_api/classifm3d_1_1ODSExtrinsicCalibrationCorrectionV1.html

uint32 version # version of the calibration correction (uint32)

float32 completion_rate # completion rate of the calibration (float)

float32[3] rot_delta_value # rotation delta values (array of 3 floats)

uint8[3] rot_delta_valid # rotation delta validity (array of 3 uint8)

float32[3] rot_head_to_user # rotation head to user transformation (array of 3 floats)