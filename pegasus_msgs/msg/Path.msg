# - The parametric value used to evaluate the parametric curve
float64 gamma  
# - The curve position, velocity and acceleration evaluated at the parametric value
float64[3] pd # Desired position [x, y, z]
float64[3] d_pd # Desired velocity [x_dot, y_dot, z_dot]
float64[3] dd_pd # Desired acceleration [x_ddot, y_ddot, z_ddot]
float64 yaw # Desired yaw in degrees (usually is the same as tangent_angle)
# - Other curve information used mainly by fixed-wing vehicles and AUVs
float64 curvature 
float64 torsion
float64 tangent_angle # The tangent angle to the path 
float64 derivative_norm
# - Desired progression speed for the given parametric curve section
float64 vehicle_speed # The desired vehicle inertial speed
float64 vd # The desired speed progression for the parametric value
float64 d_vd # The desired acceleration for the parametric value
# - The limits of the parametric curve
float64 gamma_min
float64 gamma_max