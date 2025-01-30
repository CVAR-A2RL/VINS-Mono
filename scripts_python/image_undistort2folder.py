import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import cv2
import numpy as np
from cv_bridge import CvBridge
import os

class ImageUndistorter(Node):
    def __init__(self):
        super().__init__('image_undistorter')

        # Subscriber to the CompressedImage topic
        self.subscription = self.create_subscription(
            CompressedImage,
            '/drone0/sensor_measurements/camera/image/compressed',
            self.image_callback,
            10
        )
        
        # Initialize CvBridge
        self.bridge = CvBridge()

        # Camera intrinsic and distortion parameters
        self.intrinsic_matrix = np.array([
            [565.81025122, 0, 622.49327277],
            [0, 583.87503009, 510.09288697],
            [0, 0, 1]
        ])

        self.distortion_coefficients = np.array([
            -0.2464095, 0.06922123, -0.01077599, 0.00028745
        ])

        # Directory to save images
        self.output_dir = 'saved_images'
        os.makedirs(self.output_dir, exist_ok=True)

        self.image_count = 0

        self.get_logger().info('Image undistorter node has started.')

    def image_callback(self, msg):
        try:
           
            undistorted_image = self.bridge.compressed_imgmsg_to_cv2(msg)

            # Save the undistorted image
            output_path = os.path.join(self.output_dir, f'undistorted_image_{self.image_count:04d}.jpg')
            cv2.imwrite(output_path, undistorted_image)
            self.image_count += 1

            self.get_logger().info(f'Image saved to {output_path}')
        except Exception as e:
            self.get_logger().error(f'Failed to process image: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = ImageUndistorter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

