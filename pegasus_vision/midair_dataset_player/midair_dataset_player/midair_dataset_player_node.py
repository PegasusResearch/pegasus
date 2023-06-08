#!/usr/bin/env python3

# ROS imports
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from pegasus_msgs.msg import State as StateMsg
from sensor_msgs.msg import Image as ImageMsg
from sensor_msgs.msg import CameraInfo as CameraInfoMsg
from sensor_msgs.msg import Imu as ImuMsg


# Imports for parsing the dataset
import os
import math
import cv2
import h5py
import numpy as np
from PIL import Image


class MidAirDatasetPlayer(Node):

    def __init__(self):
        super().__init__('midair_dataset_player')

        # Set the sampling rate for the groundtruth position
        self.sampling_rate = 100.0 # Hz
        self.sampling_rate_image = 25.0 # Hz
        self.sampling_ratio = int(self.sampling_rate / self.sampling_rate_image)
        
        # Setup the dataset parser
        self.declare_parameter('dataset_path', '')
        self.hdf5_path = self.get_parameter('dataset_path').get_parameter_value().string_value
        print(self.hdf5_path)
        self.database = h5py.File(self.hdf5_path, "r")
        self.db_path = os.path.dirname(self.hdf5_path)

        # Get the list of datasets
        self.datasets_names = list(self.database.keys())
        self.database_length = len(self.database)
        self.datasets_length = []

        for dataset in self.database:
            print("Dataset : %s" % dataset)
            position_sampling_rate  = self.database[dataset]["groundtruth"]["position"].attrs["sampling_frequency"]
            
            positions_count = self.database[dataset]["groundtruth"]["position"].shape
            rgb_camera_count = self.database[dataset]["camera_data"]["color_left"].shape
            depth_camera_count = self.database[dataset]["camera_data"]["depth"].shape

            assert rgb_camera_count[0] == depth_camera_count[0], "The number of samples for each sensor must be the same"
            assert abs((positions_count[0]/rgb_camera_count[0]) - self.sampling_ratio) < 0.01, "The number of positions per image must be 4"
            assert position_sampling_rate == self.sampling_rate, "The position sampling rate must be 100Hz"

            self.datasets_length.append(positions_count[0])

        print(self.database_length)
        print(self.datasets_length)
            
        print("Position sampling rate [Hz] :  %.2f" % position_sampling_rate)
        print("Image sampling rate [Hz]: %.2f" % (position_sampling_rate / 4.0))

        self.current_dataset = 0

        # ROS publishers setup
        self.gt_publisher = self.create_publisher(StateMsg, 'nav/state', qos_profile=qos_profile_sensor_data)
        self.imu_publisher = self.create_publisher(ImuMsg, 'imu/data', qos_profile=qos_profile_sensor_data)
        self.depth_publisher = self.create_publisher(ImageMsg, 'camera/depth/image_raw', qos_profile=qos_profile_sensor_data)
        self.rgb_publisher = self.create_publisher(ImageMsg, 'camera/rgb/image_raw', qos_profile=qos_profile_sensor_data)
        self.depth_camera_info_publisher = self.create_publisher(CameraInfoMsg, 'camera/depth/camera_info', qos_profile=qos_profile_sensor_data)
        self.rgb_camera_info_publisher = self.create_publisher(CameraInfoMsg, 'camera/rgb/camera_info', qos_profile=qos_profile_sensor_data)

        # ROS messages setup
        self.gt_msg = StateMsg()
        self.imu_msg = ImuMsg()
        self.depth_msg = ImageMsg()
        self.rgb_msg = ImageMsg()
        self.depth_camera_info_msg = CameraInfoMsg()
        self.rgb_camera_info_msg = CameraInfoMsg()

        # Get the start time of the simulation
        self.start_time = float(self.get_clock().now().nanoseconds / 1e9)

        timer_period = 1.0 / self.sampling_rate  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def open_float16(self, image_path):
        pic = Image.open(image_path)
        img = np.asarray(pic, np.uint16)
        img.dtype = np.float16
        return img
    
    def get_groundtruth(self, dataset, index):

        groundtruth_position = dataset["groundtruth"]["position"][index]
        groundtruth_velocity = dataset["groundtruth"]["velocity"][index]
        groundtruth_acceleration = dataset["groundtruth"]["acceleration"][index]
        groundtruth_attitude = dataset["groundtruth"]["attitude"][index]
        groundtruth_angular_velocity = dataset["groundtruth"]["angular_velocity"][index]

        return groundtruth_position, groundtruth_velocity, groundtruth_acceleration, groundtruth_attitude, groundtruth_angular_velocity
    
    def get_imu(self, dataset, index):

        imu_accelerometer = dataset["imu"]["accelerometer"][index]
        imu_gyroscope = dataset["imu"]["gyroscope"][index]

        return imu_accelerometer, imu_gyroscope

    def get_depth_images(self, dataset, index):

        # Get the path to the left RGB image and the depth image
        rgb_path = dataset["camera_data"]["color_left"][index].decode("utf-8")
        depth_path = dataset["camera_data"]["depth"][index].decode("utf-8")

        # Open the RGB and depth images with opencv
        color_left = cv2.imread(os.path.join(self.db_path, rgb_path))
        depth = self.open_float16(os.path.join(self.db_path, depth_path))   # depth in meters

        return color_left, depth
    
    def fill_gt_message(self, groundtruth_position, groundtruth_velocity, groundtruth_acceleration, groundtruth_attitude, groundtruth_angular_velocity):

        current_time = self.get_clock().now().to_msg()

        # Create the groundtruth position message
        self.gt_msg.pose.header.stamp = current_time
        self.gt_msg.pose.pose.position.x = groundtruth_position[0]
        self.gt_msg.pose.pose.position.y = groundtruth_position[1]
        self.gt_msg.pose.pose.position.z = groundtruth_position[2]

        # TODO - check this quaternion order...
        self.gt_msg.pose.pose.orientation.w = groundtruth_attitude[0]
        self.gt_msg.pose.pose.orientation.x = groundtruth_attitude[1]
        self.gt_msg.pose.pose.orientation.y = groundtruth_attitude[2]
        self.gt_msg.pose.pose.orientation.z = groundtruth_attitude[3]

        self.gt_msg.inertial_vel.header.stamp = current_time
        self.gt_msg.inertial_vel.vector.x = groundtruth_velocity[0]
        self.gt_msg.inertial_vel.vector.y = groundtruth_velocity[1]
        self.gt_msg.inertial_vel.vector.z = groundtruth_velocity[2]

        self.gt_msg.body_vel.header.stamp = current_time
        self.gt_msg.body_vel.twist.angular.x = groundtruth_angular_velocity[0]
        self.gt_msg.body_vel.twist.angular.y = groundtruth_angular_velocity[1]
        self.gt_msg.body_vel.twist.angular.z = groundtruth_angular_velocity[2]

    def fill_imu_message(self, imu_accelerometer, imu_gyroscope):

        current_time = self.get_clock().now().to_msg()

        # Create the IMU message
        self.imu_msg.header.stamp = current_time
        self.imu_msg.linear_acceleration.x = imu_accelerometer[0]
        self.imu_msg.linear_acceleration.y = imu_accelerometer[1]
        self.imu_msg.linear_acceleration.z = imu_accelerometer[2]
        self.imu_msg.angular_velocity.x = imu_gyroscope[0]
        self.imu_msg.angular_velocity.y = imu_gyroscope[1]
        self.imu_msg.angular_velocity.z = imu_gyroscope[2]

    def fill_rgb_message(self, color_left):

        byte_depth = 1
        num_channels = 3

        # Fill the image message
        self.rgb_msg.header.stamp = self.get_clock().now().to_msg()
        self.rgb_msg.header.frame_id = "rgb_camera"
        self.rgb_msg.height = color_left.shape[0]
        self.rgb_msg.width = color_left.shape[1]
        self.rgb_msg.step = self.rgb_msg.width * byte_depth * num_channels
        self.rgb_msg._data = color_left[:, :, 0:3].tobytes()

        self.rgb_msg.encoding = "bgr8"
        self.rgb_msg.is_bigendian = 0

        focal_length_x = self.rgb_msg.width / 2.0
        focal_length_y = self.rgb_msg.height / 2.0
        optical_center_x = focal_length_x
        optical_center_y = focal_length_y

        # Fill the camera info message
        self.rgb_camera_info_msg.header = self.rgb_msg.header
        self.rgb_camera_info_msg.height = self.rgb_msg.height
        self.rgb_camera_info_msg.width = self.rgb_msg.width
        self.rgb_camera_info_msg.k = [focal_length_x,            0.0, optical_center_x, 
                                                 0.0, focal_length_y, optical_center_y, 
                                                 0.0,            0.0,              1.0]
        self.rgb_camera_info_msg.distortion_model = "plumb_bob"
        self.rgb_camera_info_msg.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        self.rgb_camera_info_msg.p = [focal_length_x,            0.0, optical_center_x, 0.0, 
                                                 0.0, focal_length_y, optical_center_y, 0.0,
                                                 0.0,            0.0,              1.0, 0.0]
        self.rgb_camera_info_msg.binning_x = 0
        self.rgb_camera_info_msg.binning_y = 0
        
        
        
    def fill_depth_message(self, depth):

        byte_depth = 2
        num_channels = 1
        self.depth_msg.header.stamp = self.get_clock().now().to_msg()
        self.depth_msg.header.frame_id = "depth_camera"
        self.depth_msg.height = depth.shape[0]
        self.depth_msg.width = depth.shape[1]
        self.depth_msg.step = self.depth_msg.width * byte_depth * num_channels
        self.depth_msg._data = depth[:, :].tobytes()
        self.depth_msg.encoding = "mono16"
        self.depth_msg.is_bigendian = 0

        focal_length_x = self.depth_msg.width / 2.0
        focal_length_y = self.depth_msg.height / 2.0
        optical_center_x = focal_length_x
        optical_center_y = focal_length_y

        # Fill the camera info message
        self.depth_camera_info_msg.header = self.depth_msg.header
        self.depth_camera_info_msg.height = self.depth_msg.height
        self.depth_camera_info_msg.width = self.depth_msg.width
        self.depth_camera_info_msg.k = [focal_length_x,            0.0, optical_center_x,
                                                   0.0, focal_length_y, optical_center_y,  
                                                   0.0,            0.0,              1.0]
        self.depth_camera_info_msg.distortion_model = "plumb_bob"
        self.depth_camera_info_msg.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        self.depth_camera_info_msg.p = [focal_length_x,            0.0, optical_center_x, 0.0,
                                                   0.0, focal_length_y, optical_center_y, 0.0,
                                                   0.0,            0.0,              1.0, 0.0]
        self.depth_camera_info_msg.binning_x = 0
        self.depth_camera_info_msg.binning_y = 0


    def timer_callback(self):

        # Get the current time
        current_time = float(self.get_clock().now().nanoseconds / 1e9)
        dt = current_time - self.start_time

        gt_index = math.floor(self.sampling_rate * dt)
        image_index = math.floor(self.sampling_rate_image * dt)

        if gt_index >= self.datasets_length[self.current_dataset]:

            gt_index = 0
            image_index = 0
            self.start_time = current_time
            self.current_dataset += 1
        
            if self.current_dataset >= len(self.datasets_names):
                self.destroy_node()
                rclpy.shutdown()

        # Get the current dataset
        dataset = self.database[self.datasets_names[self.current_dataset]]

        # Fill the groundtruth and imu messages
        self.fill_gt_message(*self.get_groundtruth(dataset, gt_index))
        self.fill_imu_message(*self.get_imu(dataset, gt_index))
        
        # Publish the groundtruth message
        self.gt_publisher.publish(self.gt_msg)
        self.imu_publisher.publish(self.imu_msg)
            
        # Get the current rgb and depth images
        color_left, depth = self.get_depth_images(dataset, image_index)

        # Fill the depth and rgb images and publish them
        self.fill_rgb_message(color_left)
        self.fill_depth_message(depth)
        self.rgb_publisher.publish(self.rgb_msg)
        self.depth_publisher.publish(self.depth_msg)
        self.rgb_camera_info_publisher.publish(self.rgb_camera_info_msg)
        self.depth_camera_info_publisher.publish(self.depth_camera_info_msg)



def main(args=None):
    rclpy.init(args=args)
    midair_dataset_player = MidAirDatasetPlayer()

    rclpy.spin(midair_dataset_player)
    midair_dataset_player.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()