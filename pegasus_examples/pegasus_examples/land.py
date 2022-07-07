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

def deg_to_rad(angle):
    return angle * 180.0 / np.pi

def rad_to_deg(angle):
    return angle * np.pi / 180.0


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
        
        desired_acceleration = -self.g + (self.kp *  (pos_ref - pos))
            #+ np.dot(self.kd, vel_ref - vel) \
            #+ np.dot(self.ki, self.integral) +
    
        current_time = time.time()
        
        # Integrate the position error
        # TODO - add anti-windup here
        #if self.prev_time is not None:
        #    self.integral += (pos_ref - pos) / (current_time - self.prev_time)
        
        self.prev_time = current_time
        
        return desired_acceleration

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
    
    # Autoland the drone and disarm
    drone1.land()
    drone1.disarm()
        
    
if __name__ == '__main__':
    main()