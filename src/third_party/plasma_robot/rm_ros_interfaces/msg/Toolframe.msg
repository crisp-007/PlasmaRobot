string frame_name                       # Controller tool-frame name
geometry_msgs/Pose pose                 # Flange-to-tool pose; position in m
geometry_msgs/Vector3 euler_rad         # Rx/Ry/Rz in rad
float32 payload_kg                      # Configured payload in kg
geometry_msgs/Point center_of_mass_mm   # Payload center of mass in mm
bool success
int32 error_code                        # RealMan SDK return code
