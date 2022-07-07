import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
import time
import numpy as np
import sys

from pegasus_msgs.msg import State, AttitudeThrustControl
from pegasus_msgs.action import VehicleArm, VehicleLand, VehicleTakeOff

def controller(u, mass, yaw):
 
    T = mass * np.linalg.norm(u)
    r3d = - u / np.linalg.norm(u)
    
    RzT = np.array([[ np.cos(yaw), np.sin(yaw), 0.0],
                    [-np.sin(yaw), np.cos(yaw), 0.0],
                    [         0.0,         0.0, 1.0]])
    
    u_bar = np.dot(RzT, r3d)
    att = np.array([np.arcsin(-u_bar[1]), np.arctan2(u_bar[0], u_bar[2]), yaw])

    return (att, T)

def thrust_curve_iris(force: float):
    a = 34.03
    b = 7.151
    c = -0.012
    return (-b + np.sqrt(np.power(b, 2) - (4 * a * (c - force)))) / (2 * a) * 100.0

def thrust_curve_snap(force: float):
    return 0.23 / 0.345 * ((0.1377 * np.exp(0.02976 * force) * np.sqrt(force)) + (0.003759 * force) - 0.05973) * 100.0

def rad_to_deg(angle):
    return angle * 180.0 / np.pi

def deg_to_rad(angle):
    return angle * np.pi / 180.0

class PIDNode(Node):

    def __init__(self):
        super().__init__('pid_node')
        
        # Drone and controller parameters
        self.mass = 0.560 #1.5
        self.kp = np.diag([4.0, 4.0, 4.0])
        self.kd = np.diag([4.0, 4.0, 4.0])
        self.ki = np.diag([0.5, 0.5, 0.5])
        self.integral = np.array([0.0, 0.0, 0.0])
        
        # Current state of the vehicle
        self.landed_pos = None
        self.landed_yaw = None
        self.p = np.zeros((3,))
        self.v = np.zeros((3,))
        
        # Initialize the publishers and subscribers
        self.initialize_subscribers()
        self.initialize_publishers()
        
        # Make the drone arm
        print('Arming the drone')
        self.arm()
        time.sleep(2)
        #self.takeoff(2.0)
        #time.sleep(10)
        
        # Start the callback timer that implements the controller
        timer_period = 0.05
        self.initial_time = time.time()
        self.t = 0
        print('Starting manual control')
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
    def initialize_subscribers(self):
        
        # Subscribe to the state of the vehicle
        self.state_sub = self.create_subscription(State, '/drone6/nav/state', self.state_callback, 1)

        # Client for arming/disarming the vehicle
        self.arm_client = ActionClient(self, VehicleArm, '/drone6/arm')
        
        self.takeoff_client = ActionClient(self, VehicleTakeOff, '/drone6/takeoff')
        
        self.land_client = ActionClient(self, VehicleLand, '/drone6/land')
    
    def initialize_publishers(self):
        self.control_pub = self.create_publisher(AttitudeThrustControl, '/drone6/control/attitude_thrust', 1)
    
    def arm(self):
        """
        Arm the vehicle
        """
        arm_msg = VehicleArm.Goal()
        arm_msg.arm = True
        self.arm_client.wait_for_server()
        return self.arm_client.send_goal_async(arm_msg)
        
    def disarm(self):
        """
        Disarm the vehicle
        """
        arm_msg = VehicleArm.Goal()
        arm_msg.arm = False
        self.arm_client.wait_for_server()
        return self.arm_client.send_goal_async(arm_msg)
    
    def takeoff(self, height=1.0):
        """
        Perform a takeoff
        """
        takeoff_msg = VehicleTakeOff.Goal()
        takeoff_msg.height = height
        self.takeoff_client.wait_for_server()
        return self.takeoff_client.send_goal_async(takeoff_msg)
    
    def land(self):
        """
        Perform a land
        """
        land_msg = VehicleLand.Goal()
        land_msg.land = True
        self.land_client.wait_for_server()
        return self.land_client.send_goal_async(land_msg)
        
    def state_callback(self, msg: State):
        
        # Update the position of the vehicle expressed in NED
        self.p[0] = msg.pose.pose.position.x
        self.p[1] = msg.pose.pose.position.y
        self.p[2] = msg.pose.pose.position.z
        
        # Update the inertial velocity expressed in NED 
        self.v[0] = msg.inertial_vel.vector.x
        self.v[1] = msg.inertial_vel.vector.y
        self.v[2] = msg.inertial_vel.vector.z
        
        # Update the initial position of the vehicle
        if self.landed_pos is None:
            self.landed_pos = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z])
            self.landed_yaw = msg.rpy.vector.z

    def timer_callback(self):
        
        curr_time = time.time() - self.initial_time
        
        dt = curr_time - self.t
    
        # Compute the references
        if self.t < 15.0:
            # First - send a waypoint for the vehicle to climb
            p_ref = np.array([self.landed_pos[0], self.landed_pos[1], -1.0])
            v_ref = 0.0
            a_ref = 0.0
            yaw_ref = self.landed_yaw
        elif 15.0 <= self.t: #<= 35.0 :
            # Second - perform a circle
            print('Starting Circle')
            p_ref = np.zeros((3,))
            p_ref[0] = 1.5 * np.cos(self.t - 15.0) + 0.0
            p_ref[1] = 1.5 * np.sin(self.t - 15.0) + 0.0
            p_ref[2] = -1.5
            
            v_ref = np.zeros((3,))
            v_ref[0] = -1.5 * np.sin(self.t - 15.0)
            v_ref[1] = 1.5 * np.cos(self.t - 15.0)
            v_ref[2] = 0.0
            
            yaw_ref = rad_to_deg(np.arctan2(v_ref[1], v_ref[0])) + 90.0
            if(yaw_ref >= 500.0):
                yaw_ref = 0.0
            
            a_ref = np.zeros((3,))
            a_ref[0] = -1.5 * np.cos(self.t - 15.0)
            a_ref[1] = -1.5 * np.sin(self.t - 15.0)
            a_ref[2] = 0.0
            
        #elif self.t >= 35.0:
            # Third - perform a bernoulli
        #    print('Starting a Bernoulli')
        #    z = 1 + np.sin(self.t) * np.sin(self.t)
        #    p_ref = np.zeros((3,))
        #    p_ref[0] = 1.5 * np.cos(self.t) / z + 0.0
        #    p_ref[1] = 1.5 * np.sin(self.t) * np.cos(self.t) / z + 0.0
        #    p_ref[2] = -1.5
        #    yaw_ref = 45.0 # deg
            
        #    v_ref = np.zeros((3,))
        #    v_ref[0] = (1.5 * np.sin(self.t) * (np.sin(self.t) * np.sin(self.t) - 3)) / (z * z)
        #    v_ref[1] = -(1.5 * (3 * np.sin(self.t) * np.sin(self.t) - 1)) / np.power((np.sin(self.t) * np.sin(self.t) + 1), 2)
        #    v_ref[2] = 0.0
            
        #    a_ref = np.zeros((3,))
            
        # Compute the control law
        u = np.array([0.0, 0.0, -9.8]) -np.dot(self.kp, (self.p-p_ref)) -np.dot(self.kd, (self.v-v_ref)) -np.dot(self.ki, self.integral) + self.mass*a_ref
        att, T = controller(u, self.mass, yaw=deg_to_rad(yaw_ref))
        
        # Compute the normalized thrust between 0 - 100%
        T = np.maximum(np.minimum(thrust_curve_snap(T), 100.0), 0.0)
		
        self.integral += (self.p-p_ref) * dt
        self.integral = np.maximum(np.minimum(self.integral, 2.0), -2.0)
        
        # Publish the attitude and thrust
        msg = AttitudeThrustControl()
        msg.attitude = np.array([rad_to_deg(att[0]), rad_to_deg(att[1]), yaw_ref]).reshape((3,))
        msg.thrust = T
        print(self.p-p_ref)
        self.control_pub.publish(msg)
        
        # Update the last update time
        self.t = time.time() - self.initial_time 
        
        # Finish execution after 15 seconds
        if self.t > 50.0:
            print("Finishing maneuvre")
            self.land()   
            time.sleep(6)
            self.disarm()
            self.destroy_node()
            rclpy.shutdown()
            sys.exit(0)

def main(args=None):
    rclpy.init(args=args)
    pid_node = PIDNode()

    rclpy.spin(pid_node)
    pid_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()