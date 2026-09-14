std_msgs/Header header
string path_id
string tool_frame

# Robot state captured with the source point cloud. Joint names are preferred;
# if names are empty, positions must follow the URDF chain order.
sensor_msgs/JointState capture_joint_state

# Optional direct ^base T_gripper capture pose. When true this takes precedence
# over capture_joint_state and is interpreted using base_frame/gripper_frame.
geometry_msgs/Pose capture_gripper_pose
bool capture_gripper_pose_valid
string base_frame
string gripper_frame

bool transform_valid
bool execution_permitted
string calibration_id
string status_message

# Cavity approach geometry. The rounded-tip mouth pose is the physical opening
# target used for non-contact reach validation. TCP poses remain side-outlet
# motion targets and use the same frame as points[] on input and the robot base
# frame on transformed output. cavity_axis points from the cavity interior
# toward free space through the opening.
geometry_msgs/Pose cavity_mouth_tip_pose
bool cavity_mouth_tip_pose_valid
geometry_msgs/Pose cavity_entry_tcp_pose
bool cavity_entry_tcp_pose_valid
geometry_msgs/Pose pre_entry_tcp_pose
bool pre_entry_tcp_pose_valid
geometry_msgs/Vector3 cavity_axis
bool cavity_axis_valid
int32 cavity_entry_slice_index
float64 pre_entry_distance_m
# Axial distance kept between the physical rounded tip and the detected mouth
# at the reviewed entry stop. Positive values place the tip outside the cavity.
float64 entry_tip_standoff_m

# Link6 poses derived by the transform node from the configured TCP matrix.
geometry_msgs/Pose cavity_entry_flange_pose
bool cavity_entry_flange_pose_valid
geometry_msgs/Pose pre_entry_flange_pose
bool pre_entry_flange_pose_valid

plasma_robot_interfaces/SprayPathPoint[] points
