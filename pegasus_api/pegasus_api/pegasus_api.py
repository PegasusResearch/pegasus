import rclpy
import threading
from rclpy.node import Node
from rclpy.action import ActionClient
from pegasus_msgs.msg import State, BodyVelocityControl, AttitudeThrustControl, AttitudeRateThrustControl, ActuatorControl, PositionControl, InertialAccelerationControl, Status
from pegasus_msgs.action import VehicleArm, VehicleLand, VehicleTakeOff

import numpy as np

class Drone:
    """
    A class that represents a drone being controled via ROS2
    """
    def __init__(self, namespace='drone', id='1'):
        
        # Declare the state of the vehicle (expressed in NED frame)
        self.pos = np.zeros((3,))
        self.orientation = np.zeros((3,))
        self.quaternion = np.array([0.0, 0.0, 0.0, 1.0])
        self.body_vel = np.zeros((3,))
        self.inertial_vel = np.zeros((3,))
        self.ang_vel = np.zeros((3,))
        
        # Declare the battery percentage
        self.battery = 100.0
        self.armed = False
        
        # Initialize the ROS node
        rclpy.init(args=None)
        
        # Create the ROS node that will receive the vehicle data
        self.wrapper = self.RosWrapper(self.pos, self.orientation, self.quaternion, self.body_vel, self.inertial_vel, self.ang_vel, namespace, id)
        
        # Start the node in a separate thread
        executor = rclpy.executors.MultiThreadedExecutor()
        executor.add_node(self.wrapper)
        self.executor_thread = threading.Thread(target=executor.spin, daemon=True)
        self.executor_thread.start()
        
    def shutdown(self):
        rclpy.shutdown()
        self.executor_thread.join()
        
    def set_body_velocity(self, u: float, v: float, w: float, yaw_rate: float):
        """
        Control the body velocity of the vehicle
        """
        msg = BodyVelocityControl()
        msg.velocity_body = np.array([u, v, w])
        msg.yaw_rate_deg = yaw_rate
        self.wrapper.body_velocity_pub.publish(msg)
    
    def set_attitude_thrust(self, roll: float, pitch: float, yaw: float, thrust: float):
        """
        Control the attitude and thrust of the vehicle
        """
        msg = AttitudeThrustControl()
        msg.attitude = np.array([roll, pitch, yaw])
        msg.thrust = thrust
        self.wrapper.attitude_thrust_pub.publish(msg)
        
    def set_attitude_rate_thrust(self, roll_rate: float, pitch_rate: float, yaw_rate: float, thrust: float):
        """
        Control the angle rate and thrust of the vehicle
        """
        msg = AttitudeRateThrustControl()
        msg.attitude_rate = np.array([roll_rate, pitch_rate, yaw_rate])
        msg.thrust = thrust
        self.wrapper.attitude_rate_thrust_pub.publish(msg)
        
    def set_actuator_control(self, actuators_inputs: np.ndarray):
        """
        Set the control for the actuators of the vehicle
        """
        msg = ActuatorControl()
        msg.group0 = actuators_inputs
        self.wrapper.actuator_control_pub.publish(msg)
    
    def set_position(self, x, y, z, yaw):
        """
        Set the desired position of the vehicle
        """
        msg = PositionControl()
        msg.position = np.array([x, y, z])
        msg.yaw = yaw
        self.wrapper.position_pub.publish(msg)
        
    def set_inertial_acceleration(self, x_accel, y_accel, z_accel):
        """
        Set the acceleration of the drone in the inertial frame
        """
        msg = InertialAccelerationControl()
        msg.inertial_acceleration = np.array([x_accel, y_accel, z_accel])
        self.wrapper.inertial_acceleration_pub.publish(msg)
    
    def arm(self):
        """
        Arm the vehicle
        """
        arm_msg = VehicleArm.Goal()
        arm_msg.arm = True
        self.wrapper.arm_client.wait_for_server()
        return self.wrapper.arm_client.send_goal_async(arm_msg)
        
    def disarm(self):
        """
        Disarm the vehicle
        """
        arm_msg = VehicleArm.Goal()
        arm_msg.arm = False
        self.wrapper.arm_client.wait_for_server()
        return self.wrapper.arm_client.send_goal_async(arm_msg)
            
    def takeoff(self, height=1.0):
        """
        Perform a takeoff
        """
        takeoff_msg = VehicleTakeOff.Goal()
        takeoff_msg.height = height
        self.wrapper.takeoff_client.wait_for_server()
        return self.wrapper.takeoff_client.send_goal_async(takeoff_msg)
    
    def land(self):
        """
        Perform a land
        """
        land_msg = VehicleLand.Goal()
        land_msg.land = True
        self.wrapper.land_client.wait_for_server()
        return self.wrapper.land_client.send_goal_async(land_msg)
    
    class RosWrapper(Node):
        """
        Wrapper class used to start the ros node 
        that subscribes and publishes
        """       

        def __init__(self, pos, orientation, quaternion, body_vel, inertial_vel, ang_vel, namespace='drone', id='1'):
            """
            Constructor for the Drone class
            """
            super().__init__('drone_api')
            
            # Save the number and namespace of the drone being controlled
            self.namespace = namespace
            self.id = id
            
            # Initialize the state of the vehicle
            self.pos = pos
            self.orientation = orientation
            self.quaternion = quaternion
            self.body_vel = body_vel
            self.inertial_vel = inertial_vel
            self.ang_vel = ang_vel
            
            # Intialize all ROS Publisher and Subscribers to 
            self.initialize_subscribers()
            self.initialize_publishers()
            self.initialize_actions()

        def initialize_subscribers(self):
            """
            Method that initializes all the relevant subscribers to get the current state of the vehicle
            """
            
            # Get the position and orientation of the vehicle
            self.state_sub = self.create_subscription(State, self.namespace + str(self.id) + '/nav/state', self.state_callback, 1)
            
            # Get the status of the vehicle (battery level, etc.)
            self.status_sub = self.create_subscription(Status, self.namespace + str(self.id) + '/status', self.status_callback, 1)

        def initialize_publishers(self):
            """
            Method that initialies all the relevant publisher to send commands to the vehicle
            """
            self.body_velocity_pub = self.create_publisher(BodyVelocityControl, self.namespace + str(self.id) + '/control/body_velocity', 1)
            self.attitude_thrust_pub = self.create_publisher(AttitudeThrustControl, self.namespace + str(self.id) + '/control/attitude_thrust', 1)
            self.attitude_rate_thrust_pub = self.create_publisher(AttitudeRateThrustControl, self.namespace + str(self.id) + '/control/attitude_rate_thrust', 1)
            self.actuator_control_pub = self.create_publisher(ActuatorControl, self.namespace + str(self.id) + '/control/actuator_control', 1)
            self.position_pub = self.create_publisher(PositionControl, self.namespace + str(self.id) + '/control/position', 1)
            self.inertial_acceleration_pub = self.create_publisher(InertialAccelerationControl, self.namespace + str(self.id) + '/control/inertial_acceleration', 1)
        
        def initialize_actions(self):
            """
            Method that initializes the arm, disarm, takeoff and land actions
            """
            self.arm_client = ActionClient(self, VehicleArm, self.namespace + str(self.id) + '/arm')
            self.takeoff_client = ActionClient(self, VehicleTakeOff, self.namespace + str(self.id) + '/takeoff')
            self.land_client = ActionClient(self, VehicleLand, self.namespace + str(self.id) + '/land')
        
        def state_callback(self, msg: State):
            """
            Callback that receives a state message with the current state of the vehicle
            """
            
            # Update the position of the vehicle expressed in NED
            self.pos[0] = msg.pose.pose.position.x
            self.pos[1] = msg.pose.pose.position.y
            self.pos[2] = msg.pose.pose.position.z
            
            # Update the orientation quaternion expressed in NED
            self.quaternion[0] = msg.pose.pose.orientation.x
            self.quaternion[1] = msg.pose.pose.orientation.y
            self.quaternion[2] = msg.pose.pose.orientation.z
            self.quaternion[3] = msg.pose.pose.orientation.w
            
            # Update the roll, pitch and yaw angles expressed in NED
            self.orientation[0] = msg.rpy.vector.x
            self.orientation[1] = msg.rpy.vector.y
            self.orientation[2] = msg.rpy.vector.z

            # Update the linear body velocity (u,v, w) in F.R.D
            self.body_vel[0] = msg.body_vel.twist.linear.x
            self.body_vel[1] = msg.body_vel.twist.linear.y
            self.body_vel[2] = msg.body_vel.twist.linear.z
            
            # Update the inertial velocity expressed in NED 
            self.inertial_vel[0] = msg.inertial_vel.vector.x
            self.inertial_vel[1] = msg.inertial_vel.vector.y
            self.inertial_vel[2] = msg.inertial_vel.vector.z
            
            # Update the angular body velocity
            self.ang_vel[0] = msg.body_vel.twist.angular.x
            self.ang_vel[1] = msg.body_vel.twist.angular.y
            self.ang_vel[2] = msg.body_vel.twist.angular.z
            
        def status_callback(self, msg: Status):
            """
            Callback that receives a Status message with the current batter percentage, RC status, etc.
            """
            self.battery = msg.battery
            self.armed = msg.armed