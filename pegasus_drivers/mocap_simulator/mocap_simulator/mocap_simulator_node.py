import rclpy
from rclpy.node import Node
import numpy as np

from geometry_msgs.msg import PoseStamped

class MocapSimulator(Node):

    def __init__(self):
        super().__init__('mocap_simulator')
        self.publisher_ = self.create_publisher(PoseStamped, 'snap/pose', 1)
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = PoseStamped()

        # Mock the position in ENU sent by the mocap system
        msg.pose.position.x = 5.0 + np.random.normal(loc=0.0, scale=0.02)
        msg.pose.position.y = 10.0 + np.random.normal(loc=0.0, scale=0.02)
        msg.pose.position.z = 3.0 + np.random.normal(loc=0.0, scale=0.02)

        # Mock the orientation quaternion given by the mocap system
        msg.pose.orientation.x = 0.0 + np.random.normal(loc=0.0, scale=0.002)
        msg.pose.orientation.y = 0.0 + np.random.normal(loc=0.0, scale=0.002)
        msg.pose.orientation.z = 0.0 + np.random.normal(loc=0.0, scale=0.002)
        msg.pose.orientation.w = 1.0 + np.random.normal(loc=0.0, scale=0.002)

        # Mock the time in the message
        msg.header.stamp = self.get_clock().now().to_msg()

        # Publish the message
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    mocap_simulator = MocapSimulator()

    rclpy.spin(mocap_simulator)
    mocap_simulator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()