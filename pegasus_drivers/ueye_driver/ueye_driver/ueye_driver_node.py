#!/usr/bin/env python
# Copyright (c) 2023, PegasusResearch
# All rights reserved.

# ROS2 imports
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo

# Camera driver imports
from .ueye.camera import Camera
from .ueye.utils import FrameThread

# OpenCV bridge (python -m pip install cv-bridge)
from cv_bridge import CvBridge

class UeyeDriverNode(Node):

    def __init__(self):
        
        super().__init__('ueye_driver_node')

        # Create the publisher for the image data
        self.publisher_ = self.create_publisher(Image, 'image', 1)

        # Create a bridge to convert the image data to OpenCV format
        self.cv_bridge = CvBridge()

        # Create the camera object
        self.camera = Camera()
        self.camera.init()
        self.camera.alloc()
        self.camera.capture_video()

        # A thread that waits for new images and processes all connected views
        self.camera_thread = FrameThread(self.camera, views=[self.handle_image])
        self.camera_thread.start()

    def stop(self):
        
        # Join the camera thread
        self.camera_thread.stop()
        self.camera_thread.join()
        
        # Destroy the camera objet and free the allocated memory
        self.camera.stop_video()
        self.camera.exit()

    def handle_image(self, image_data, image_encoding='bgr8'):

        # Create the ROS2 image message
        msg = self.cv_bridge.cv2_to_imgmsg(image_data.as_1d_image(), image_encoding)
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "camera"

        # Publish the image message
        self.publisher_.publish(msg)


def main(args=None):

    # Initialize the rclpy library
    rclpy.init(args=args)
    ueye_driver_node = UeyeDriverNode()

    # Start the ROS 2 node and handle keyboard interrupts
    try:
        rclpy.spin(ueye_driver_node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        pass
    finally:
        # Destroy the node explicitly
        ueye_driver_node.stop()
        rclpy.try_shutdown()
        ueye_driver_node.destroy_node()

if __name__ == '__main__':
    main()