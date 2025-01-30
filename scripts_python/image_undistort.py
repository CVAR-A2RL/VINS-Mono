import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image
import cv2
import numpy as np
from cv_bridge import CvBridge
import os
import yaml
import sys

class ImageUndistorter(Node):
    def __init__(self, camera_config):
        super().__init__('image_undistorter')
        sub_topic = camera_config['input_topic']
        pub_topic = camera_config['output_topic']

        self.compressed = camera_config['compressed']
        
        if self.compressed == 1:
            # Subscriber to the CompressedImage topic
            self.subscription = self.create_subscription(
                CompressedImage,
                sub_topic,
                self.image_callback,
                100
            )
        else:
            # Subscriber to the Image topic
            self.subscription = self.create_subscription(
                Image,
                sub_topic,
                self.image_callback,
                100
            )

        
        # Publisher to the CompressedImage topic
        self.publisher_ = self.create_publisher(
            CompressedImage,
            pub_topic,
            100
        )
        
        # Initialize CvBridge
        self.bridge = CvBridge()

        fx = camera_config['camera_matrix']['fx']
        fy = camera_config['camera_matrix']['fy']
        cx = camera_config['camera_matrix']['cx']
        cy = camera_config['camera_matrix']['cy']

        # Camera intrinsic and distortion parameters
        self.intrinsic_matrix = np.array([
            [fx,  0, cx],
            [ 0, fy, cy],
            [ 0,  0,  1]
        ])

        self.distortion_coefficients = np.array(camera_config['distortion_coefficients'])

        print('Intrinsic matrix:\n', self.intrinsic_matrix)
        print('Distortion coefficients:\n', self.distortion_coefficients)

        self.get_optimal_matrix = True
        self.new_camera_matrix = None

        self.get_logger().info('Image undistorter node has started.')

    def image_callback(self, msg):
        # try:
        if self.compressed == 1:
            uncompressed_image = self.bridge.compressed_imgmsg_to_cv2(msg)
        else:
            uncompressed_image = self.bridge.imgmsg_to_cv2(msg)
        if self.get_optimal_matrix:
            self.new_camera_matrix, roi = cv2.getOptimalNewCameraMatrix(
                self.intrinsic_matrix,
                self.distortion_coefficients,
                (uncompressed_image.shape[1], uncompressed_image.shape[0]),
                1,
                (uncompressed_image.shape[1], uncompressed_image.shape[0])
            )
            self.get_optimal_matrix = False
        undistorted_image = cv2.undistort(
            uncompressed_image,
            self.intrinsic_matrix,
            self.distortion_coefficients,
            self.new_camera_matrix
        )
        print('Undistorted image shape:', undistorted_image.shape)
        print('New camera matrix:\n', self.new_camera_matrix)

        # Publish in topic
        new_msg = CompressedImage()
        new_msg.header = msg.header
        new_msg.format = 'jpeg'
        # if self.compressed == 1:
        new_msg.data = self.bridge.cv2_to_compressed_imgmsg(undistorted_image).data
        # else:
        #     new_msg.data = self.bridge.cv2_to_imgmsg(undistorted_image).data
        self.publisher_.publish(new_msg)
        self.get_logger().info('Image processed and published.')
        # except Exception as e:
        #     self.get_logger().error(f'Failed to process image: {e}')


def main(args=None):
    camera_config_file_path = sys.argv[1]
    with open(camera_config_file_path, 'r') as file:
        camera_config = yaml.safe_load(file)

    rclpy.init(args=args)
    node = ImageUndistorter(camera_config)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

