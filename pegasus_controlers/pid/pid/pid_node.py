import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
import time
import numpy as np
import sys

from pegasus_msgs.msg import State, AttitudeThrustControl
from pegasus_msgs.action import VehicleArm, VehicleLand, VehicleTakeOff

def controller(u, mass):

	g = 9.8066
	e3 = np.array([0, 0, 1])
	
	TRe3 = mass * g * e3 - u 
	T = np.linalg.norm(TRe3)
	Re3 = TRe3 / T
	att = np.array([np.arcsin(-Re3[1]), np.arctan2(Re3[0], Re3[2]), 0])

	return (att, T)

def thrust_curve_iris(force: float):
    a = 34.03
    b = 7.151
    c = -0.012
    return (-b + np.sqrt(np.power(b, 2) - (4 * a * (c - force)))) / (2 * a) * 100.0

def thrust_curve_snap(force: float):
    a = 0.1377
    b = 0.003759
    c = -0.02976
    d = -0.05973
    return ((a * np.exp(-c * force) * np.sqrt(force)) + (b * force) + d) * 100.0

def rad_to_deg(angle):
    return angle * 180.0 / np.pi

class PIDNode(Node):

    def __init__(self):
        super().__init__('pid_node')
        
        # Drone and controller parameters
        self.mass = 0.560
        self.kp = np.diag([1.0, 1.0, 1.0])
        self.kd = np.diag([4.0, 4.0, 4.0])
        self.ki = np.diag([0.0, 0.0, 0.0])
        self.integral = np.array([0.0, 0.0, 0.0])
        
        # Current state of the vehicle
        self.p = np.zeros((3,))
        self.v = np.zeros((3,))
        
        # Initialize the publishers and subscribers
        self.initialize_subscribers()
        self.initialize_publishers()
        
        # Make the drone arm
        print('Arming the drone')
        #self.arm()
        print('Auto-takeoff')
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

    def timer_callback(self):
        
        curr_time = time.time() - self.initial_time
        
        dt = curr_time - self.t
    
        # Compute the references
        if self.t < 15.0:
            p_ref = np.array([0.0, 0.0, -1.0])
        elif self.t >= 15.0:
            p_ref = np.array([2.0, 1.0, -1.5])
            
        v_ref = 0.0
        a_ref = 0.0
        # Compute the control law
        u = np.array([0.0, 0.0, -9.8]) -np.dot(self.kp, (self.p-p_ref)) -np.dot(self.kd, (self.v-v_ref)) -np.dot(self.ki, self.integral) + self.mass*a_ref
        att, T = controller(u, self.mass)
        
        # Compute the normalized thrust between 0 - 100%
        T = np.maximum(np.minimum(thrust_curve_snap(T), 100.0), 0.0)
		
        self.integral += (self.p-p_ref) * dt
        
        # Publish the attitude and thrust
        msg = AttitudeThrustControl()
        msg.attitude = np.array([rad_to_deg(att[0]), rad_to_deg(att[1]), rad_to_deg(att[2])]).reshape((3,))
        msg.thrust = T
        print(msg.thrust)
        #self.control_pub.publish(msg)
        
        # Update the last update time
        self.t = time.time() - self.initial_time 
        
        # Finish execution after 15 seconds
        if self.t > 45.0:
            print("Finishing maneuvre")
            self.land()   
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