std_msgs/Header header
uint32 num_satellites   # Number of sattelites
uint8 fix_type          # Fix type

# ---------------------------------
# Constants Definition for the fix-type
# --------------------------------- 
uint8 NO_GPS = 0
uint8 NO_FIX = 1
uint8 FIX_2D = 2
uint8 FIX_3D = 3
uint8 FIX_DGPS = 4
uint8 RTK_FLOAT = 5
uint8 RTK_FIXED = 6