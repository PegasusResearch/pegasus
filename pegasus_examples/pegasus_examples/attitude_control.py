#!/usr/bin/env python3
from pegasus_api import Drone
import numpy as np
import time

mass_aero = 1.350
mass_snap = 0.600
mass_iris = 1.500

def thrust_curve_aero(force: float):
    a = 8.84
    b = 2.955
    c = -1.478
    d = 10.37
    return (np.tan((force - d) / a) - c) / b * 100.0

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


class PIDController:
    """
    A simple PID controller to demonstrate the Drone API
    """
    g = np.array([0.0, 0.0, 9.8])
    
    def __init__(self, kp: np.ndarray, kd: np.ndarray, ki: np.ndarray, mass: float):
        """
        Constructor for the PID position controller
        """
        self.kp = kp
        self.kd = kd
        self.ki = ki
        self.integral = np.zeros((3,))
        self.prev_time = None
        
        print('Controller gains')
        print(self.kp)
        print(self.kd)
        print(self.ki)
        
        # Constants
        self.mass = mass
        
        print('Vehicle mass')
        print(self.mass)
        
    def compute_control(self, pos, vel, pos_ref, vel_ref, accel_ref, yaw):
        """
        Compute the desired thrust and attitude to apply to the vehicle
        """
        
        # Compute the desired acceleration in the inertial frame
        des_accel = self.compute_acceleration(pos, vel, pos_ref, vel_ref, accel_ref)
        
        # Compute the desired thrust and atttitude to request to the inner-loops
        return self.aceleration_to_attitude(des_accel, yaw)
    
    def compute_acceleration(self, pos, vel, pos_ref, vel_ref, accel_ref):
        """
        Compute the desired acceleration using a PID control law
        """
        # Compute the PID control law
        pos_error = pos_ref - pos
        vel_error = vel_ref - vel
        
        u = -self.g + (self.kp * pos_error) + (self.kd *  vel_error) + (self.ki * self.integral)
    
        current_time = time.time()
        
        # Integrate the position error
        # TODO - add anti-windup here
        if self.prev_time is not None:
            self.integral += pos_error / (current_time - self.prev_time)
        
        self.prev_time = current_time
        
        return u

    def aceleration_to_attitude(self, u: np.ndarray, yaw: float):
        """
        Compute the desired attitude for the vehicle, based on the desired acceleration 
        vector expressed in the inertial frame of reference (in NED)
        """
        
        # Compute the total thrust in N to apply to the vehicle and the direction vector
        T = self.mass * np.linalg.norm(u)
        r3d = - u / np.linalg.norm(u)
        
        # Define the rotation matrix about the z-axis
        Rz_T = np.array([[ np.cos(yaw), np.sin(yaw), 0.0],
                         [-np.sin(yaw), np.cos(yaw), 0.0],
                         [         0.0,         0.0, 1.0]])
        
        u_bar = np.dot(Rz_T, r3d)
        
        # Compute the desired roll and pitch angles
        roll = np.arcsin(-u_bar[1])
        pitch = np.arctan2(u_bar[0], u_bar[2])
        
        att = np.array([roll, pitch, yaw])
        
        return (att, T)
    

def main(args=None):
     
    # Create the drone
    drone1 = Drone(id='1')
    
    # Create the PID controller
    pid = PIDController(
        kp=np.array([4.0, 4.0, 4.0]), 
        kd=np.array([4.0, 4.0, 4.0]), 
        ki=np.array([0.15, 0.15, 0.5]), 
        mass=mass_iris)
    
    # Arm the vehicle
    print("Arming the vehicle")
    drone1.arm()
    
    # Use automatic takeoff
    drone1.takeoff(2.0)
    time.sleep(15)
    
    # Get the current time
    initial_time = time.time()
    curr_time = 0
    
    # Control the drone at approximately 20 Hz
    while(curr_time < 30.0):
        
        curr_pos = drone1.pos
        print(curr_pos)
        curr_vel = drone1.inertial_vel
        
        desired_pos = np.array([0.0, 0.0, -3.0])
        desired_vel = np.zeros((3,))
        desired_accel = np.zeros((3,))        
        desired_yaw = 0.0       # Radians
        
        # Compute the desired control law to apply
        attitude, thrust = pid.compute_control(curr_pos, curr_vel, desired_pos, desired_vel, desired_accel, desired_yaw)
        
        # Convert the thrust in Newton to percentage (0-100.0%)
        normalized_thrust = thrust_curve_iris2(thrust, np.linalg.norm(drone1.inertial_vel))
        #print(normalized_thrust)
        
        # Set the deisred attitude and thrust to apply
        drone1.set_attitude_thrust(roll=rad_to_deg(attitude[0]), pitch=rad_to_deg(attitude[1]), yaw=rad_to_deg(desired_yaw), thrust=normalized_thrust)
      
        # Update the current elapsed time  
        curr_time = time.time() - initial_time
        
        time.sleep(0.005)
    
    # Autoland the drone and disarm
    drone1.land()
    drone1.disarm()
        
    
if __name__ == '__main__':
    main()