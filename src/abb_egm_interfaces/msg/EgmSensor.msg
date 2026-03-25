std_msgs/Header header

uint8 msg_type
uint8 MSGTYPE_UNDEFINED=0
uint8 MSGTYPE_COMMAND=1
uint8 MSGTYPE_DATA=2
uint8 MSGTYPE_CORRECTION=3
uint8 MSGTYPE_PATH_CORRECTION=4

uint8 mode
uint8 MODE_UNDEFINED=0
uint8 MODE_JOINTS=1
uint8 MODE_CARTESIAN=2

sensor_msgs/JointState planned_joints
geometry_msgs/PoseStamped planned_pose
sensor_msgs/JointState planned_ext_joints

float64[] speed_ref_joints
float64[] speed_ref_cartesian
float64[] speed_ref_ext_joints

bool send_rapid_data
bool rapid_dig_val
float64[] rapid_dnum