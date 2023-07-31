std_msgs/Header header
float64 altitude_monotonic          # Altitude in meters is initialized on system boot and monotonic
float64 altitude_amsl               # Altitude AMSL (above mean sea level) in meters
float64 altitude_local              # Local altitude in meters
float64 altitude_relative_home      # Altitude above home position in meters
float64 altitude_relative_terrain   # Altitude above terrain in meter
float64 bottom_clearance            # This is not the altitude, but the clear space below the system according to the fused clearance estimate in meters