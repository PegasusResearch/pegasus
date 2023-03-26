#!/usr/bin/env python
# Copyright (c) 2023, PegasusResearch
# All rights reserved.

# ROS2 imports
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

# Camera driver imports
from .ueye.camera import Camera
from .ueye.utils import FrameThread

class UeyeDriverNode(Node):

    def __init__(self):
        
        super().__init__('ueye_driver_node')

        # Create the publisher for the image data
        self.publisher_ = self.create_publisher(Image, 'image', 1)

        # Create the camera object
        self.camera = Camera()
        self.camera.init()

        self.get_logger().info('Before thread')

        # A thread that waits for new images and processes all connected views
        self.camera_thread = FrameThread(self.camera, views=[self.handle_image])
        self.camera_thread.start()

        self.get_logger().info('After thread')

    def __del__(self):
        
        # Join the camera thread
        self.camera_thread.stop()
        self.camera_thread.join()
        
        # Destroy the camera objet and free the allocated memory
        self.camera.stop_video()
        self.camera.exit()

    def handle_image(self, image_data):

        # Create the ROS2 image message
        msg = Image()

        # Set the image data
        msg.data = image_data.as_1d_image().tobytes()

        self.get_logger().info('Image shape: %s' % str(image_data.as_1d_image().shape))

        # Set the image encoding
        msg.encoding = 'mono8'

        # Set the image height
        msg.height = image_data.mem_info.height

        # Set the image width
        msg.width = image_data.mem_info.width

        # Set the image step
        msg.step = image_data.mem_info.width

        # Publish the image message
        self.publisher_.publish(msg)

        # Unlock the image buffer
        image_data.unlock()


def main(args=None):

    # Initialize the rclpy library
    rclpy.init(args=args)
    mocap_simulator = UeyeDriverNode()

    # Start the ROS 2 node
    rclpy.spin(mocap_simulator)
    mocap_simulator.destroy_node()
    
    # Shutdown the ROS 2 node
    rclpy.shutdown()

if __name__ == '__main__':
    main()