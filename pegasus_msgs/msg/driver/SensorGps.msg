std_msgs/Header header
float64 latitude_deg		    # Latitude in degrees, allows centimeter level RTK precision
float64 longitude_deg		    # Longitude in degrees, allows centimeter level RTK precision
float64 altitude_msl		    # Altitude above MSL, meters
float64 altitude_ellipsoid  	# Altitude above Ellipsoid, meters
float64 hdop                    # GPS HDOP horizontal dilution of position in meters
float64 vdop                    # GPS VDOP horizontal dilution of position in meters
float64 velocity                # GPS ground velocity in meters per second
float64 heading                 # Compass heading in degrees, 0.0..360.0 degrees. If unknown, set to: 361.0
float64 cog_deg                 # Course over ground (NOT heading, but direction of movement) in degrees. 0.0..360.0 degrees
float64 horizontal_uncertainty  # Horizontal position uncertainty in meters
float64 vertical_uncertainty    # Vertical position uncertainty in meters
float64 velocity_uncertainty    # Velocity uncertainty in meters per second
float64 heading_uncertainty     # Heading uncertainty in degrees