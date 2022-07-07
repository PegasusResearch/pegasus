#!/usr/bin/env python3
from pegasus_api import Drone
import numpy as np
from math import cos, sin, asin, atan
from numpy.linalg import norm
import time

mass_aero = 1.350
mass_snap = 0.600
mass_iris = 1.500

def thrust_curve_iris(force: float):
    a = 34.03
    b = 7.151
    c = -0.012
    return (-b + np.sqrt(np.power(b, 2) - (4 * a * (c - force)))) / (2 * a) * 100.0

def thrust_curve_iris2(force: float, vel: np.ndarray):
    return ((force / (1.0 - vel / 25.0) - 1.52 * 9.80665) / (2 * 34.068 * 0.561 + 7.1202) + 0.561) * 100.0

def deg_to_rad(angle):
    return angle * np.pi / 180.0

def rad_to_deg(angle):
    return angle * 180.0 / np.pi

def controller(u, mass):

    g = 9.8066
    e3 = np.array([0, 0, 1])

    TRe3 = mass * g * e3 - u 
    T = np.linalg.norm(TRe3)
    Re3 = TRe3 / T
    att = np.array([asin(-Re3[1]), atan(Re3[0]/Re3[2]), 0.0])

    return (att, T)

def main(args=None):
     
    # Create the drone
    drone1 = Drone(id='1')
    
    # Arm the vehicle
    print("Arming the vehicle")
    drone1.arm()
    
    # Use automatic takeoff
    drone1.takeoff(1.5)
    time.sleep(15)
    
    # Get the current time
    initial_time = time.time()
    curr_time = 0
    
    kp = np.diag([4.0, 4.0, 4.0])
    kd = np.diag([4.0, 4.0, 4.0])
    ki = np.diag([0.15, 0.15, 0.5])
    
    integral = np.array([0.0, 0.0, 0.0])
    
    mass = mass_iris
    
    dt = 0.05
    
    # Control the drone at approximately 20 Hz
    while(curr_time < 30.0):
        
        p = drone1.pos
        v = drone1.inertial_vel
        
        t = curr_time
        p_ref = np.array([2.6*sin(2*0.55*t), 1.6*sin(3*0.55*t+np.pi/2), -1.2+0.6*cos(0.35*t)])
        v_ref = np.array([2*0.55*2.6*cos(2*0.55*t), 3*0.55*1.6*cos(3*0.55*t+np.pi/2), -0.35*0.6*sin(0.35*t)])		
        a_ref = np.array([-2*0.55*2*0.55*2.6*sin(2*0.55*t), -3*0.55*3*0.55*1.6*sin(3*0.55*t+np.pi/2), -0.35*0.35*0.6*cos(0.35*t)])
       
        u = -np.dot(kp, (p-p_ref)) -np.dot(kd, (v-v_ref)) -np.dot(ki, integral) + mass*a_ref
        
        att, T = controller(u, mass)
        thrust = thrust_curve_iris2(T, np.linalg.norm(v))
        
        integral += (p-p_ref) * dt
        
        # Set the deisred attitude and thrust to apply
        drone1.set_attitude_thrust(roll=att[0], pitch=att[1], yaw=att[2], thrust=thrust)
      
        # Update the current elapsed time  
        curr_time = time.time() - initial_time
        
        time.sleep(dt)
    
    # Autoland the drone and disarm
    drone1.land()
    drone1.disarm()
        
    
if __name__ == '__main__':
    main()