# ODS transforms

ODS data is published with reference to the `"ifm_base_link"`, which, in general, corresponds to the robot's `"base_link"`. 

The reference coordinate system for ODS is constrained such that:
- The center of the ODS coordinate system is at floor level,
- The X axis is pointing in the direction of forward movement,
- The Z axis is pointing upwards.

The cameras used by ODS must be calibrated with reference to the ODS coordinate system, as part of the initial ODS configuration (see [the ODS configuration documentation](./ods_configuration.md)).

The origin of the ODS coordinate system corresponds to the center of the occupancy grid. 

Publishing the tf from the robot's `"base_link"` to the `"ifm_base_link"` should be done by the user.
