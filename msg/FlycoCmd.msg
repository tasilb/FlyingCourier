# Operation states
uint8 STATUS_FAULT=0
uint8 STATUS_INIT=1
uint8 STATUS_READY=2
uint8 STATUS_ACUTATION=3
uint8 STATUS_LANDING=4
uint8 STATUS_SETPOINT_NAV=5

# Commands
uint8 CMD_FAILSAFE=0
uint8 CMD_LAND=1
uint8 CMD_SETPOINT_POS=2
uint8 CMD_ACTUATE=3

uint8 cmd
geometry_msgs/PoseStamped setpoint_pose
