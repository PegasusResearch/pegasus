# Statistics for each individual PID controller at the acceleration tracking level
pegasus_msgs/PidStatistic[3] pid_statistics
# Statistics for the overall acceleration tracking
float64[3] rotation_error
# Desired angular rate
float64[3] desired_angular_rate
# Total Thrust asked to the system (in N)
float64 thrust_reference
# Attitude rate asked to the system (in deg/s)
float64[3] attitude_rate_reference