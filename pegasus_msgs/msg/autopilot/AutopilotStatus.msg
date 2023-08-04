std_msgs/Header header
uint8 state


# -----------------------------------------
# Header for the Autopilot different states
# -----------------------------------------
uint8 DISARMED = 0
uint8 ARMED_LANDED = 1
uint8 TAKE_OFF = 2
uint8 LAND = 3
uint8 HOLD = 4
uint8 WAYPOINT = 5