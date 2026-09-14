uint8 PHASE_PROCESS_POSITIVE=0
uint8 PHASE_RETURN_FROM_POSITIVE=1
uint8 PHASE_PROCESS_NEGATIVE=2
uint8 PHASE_RETURN_FROM_NEGATIVE=3
uint8 PHASE_LAYER_TRANSITION=4

# Desired TCP pose. On input it is expressed in SprayPath.header.frame_id;
# on output it is expressed in the robot base frame.
geometry_msgs/Pose tcp_pose

# Rounded physical nozzle end used for bottom-clearance checks. This is
# calculated geometry, not the spraying TCP or a separately calibrated point.
# On transformed output it is expressed in the robot base frame.
geometry_msgs/Point safety_point
bool safety_point_valid

# Center pose of the physical side outlet. It coincides with tcp_pose; its +Z
# axis is the spray direction.
# On transformed output it is expressed in the robot base frame.
geometry_msgs/Pose spray_outlet_pose
bool spray_outlet_pose_valid

# Link6/flange pose calculated from tcp_pose and the configured tool transform.
geometry_msgs/Pose flange_pose
bool flange_pose_valid

int32 sequence_index
int32 layer_index
uint8 motion_phase
bool plasma_enabled
float64 joint6_geometric_deg

# Surface point associated with this pose, in the same frame as tcp_pose.
geometry_msgs/Point surface_target
