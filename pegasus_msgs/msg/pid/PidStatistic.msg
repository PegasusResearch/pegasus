# -- The last time step in seconds taken between the current and last iteration
float64 dt
float64 reference
# -- Errors and references not affected by the gains
float64 error_p # The proportional error fed into the p_term
float64 error_d # The derivative error fed into the d_term
float64 integral # The integral value fed into the i_term
float64 ff_ref # The fead-forward value fed into the ff_term
# -- PID terms already afftected by the gains
float64 p_term # The scaled proportional error (kp * error_p)
float64 d_term # The scaled derivative error (kd * error_d)
float64 i_term # The scaled integral value (i_term <=> iterm += ki * (error_p - prev_error_p))
float64 ff_term # The scaled feed-forward value (kff * ff_ref)
# -- Outputs of the control loop
float64 anti_windup_discharge # The amount that was discharged from the integral via the anti-windup mechanism
float64 output_pre_sat # The output of the PID before being saturated
float64 output # The actual output of the PID