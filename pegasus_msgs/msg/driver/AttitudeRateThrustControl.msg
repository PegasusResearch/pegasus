# Message for expressing a vehicle desired attitude-rate (roll-rate, pitch-rate and yaw-rate) and thrust to apply upwards 
float64[3] attitude_rate    # Roll-rate, pitch-rate and yaw-rate (expressed in degrees/s and following the Z-Y-X convention)
float64 thrust              # Total thrust to apply, scalled between (0, 100%)