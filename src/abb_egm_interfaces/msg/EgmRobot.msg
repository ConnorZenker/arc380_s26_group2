std_msgs/Header header

uint8 msg_type
uint8 MSGTYPE_UNDEFINED=0
uint8 MSGTYPE_COMMAND=1
uint8 MSGTYPE_DATA=2
uint8 MSGTYPE_CORRECTION=3
uint8 MSGTYPE_PATH_CORRECTION=4

sensor_msgs/JointState feedback_joints
geometry_msgs/PoseStamped feedback_pose
sensor_msgs/JointState feedback_ext_joints

sensor_msgs/JointState planned_joints
geometry_msgs/PoseStamped planned_pose
sensor_msgs/JointState planned_ext_joints

uint8 motor_state
uint8 MOTORS_UNDEFINED=0
uint8 MOTORS_ON=1
uint8 MOTORS_OFF=2

uint8 mci_state
uint8 MCI_UNDEFINED=0
uint8 MCI_ERROR=1
uint8 MCI_STOPPED=2
uint8 MCI_RUNNING=3

bool mci_convergence_met
float64[] test_signals

uint8 rapid_exec_state
uint8 RAPID_UNDEFINED=0
uint8 RAPID_STOPPED=1
uint8 RAPID_RUNNING=2

bool measured_force_active
float64[] measured_force

float64 utilization_rate
uint32 move_index

bool collision_triggered
float64[] collision_quota

bool rapid_dig_val
float64[] rapid_dnum