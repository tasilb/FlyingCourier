# Operation states
uint8 STATUS_FAULT=0
uint8 STATUS_INIT=1
uint8 STATUS_READY=2
uint8 STATUS_ACTUATION=3
uint8 STATUS_LANDING=4
uint8 STATUS_SETPOINT_NAV=5

# Setpoint types
uint8 SETPOINT_POS=0

uint8 status
uint8 setpoint_type
mavros_msgs/State mavros_state
mavros_msgs/BatteryStatus battery
geometry_msgs/PoseStamped local_pose
geometry_msgs/PoseStamped setpoint_pose
