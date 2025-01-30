import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import cv2
from cv_bridge import CvBridge
import os

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')
        self.image_folder = 'images'
        self.image_topic = "/drone0/sensor_measurements/camera/image/compressed"
        if not os.path.exists(self.image_folder):
            os.makedirs(self.image_folder)

        self.subscription = self.create_subscription(
            CompressedImage,
            self.image_topic,
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.bridge = CvBridge()

        self.idx = 0

    def listener_callback(self, msg):
        # self.get_logger().info('Receiving image')
        cv_image = self.bridge.compressed_imgmsg_to_cv2(msg)
        # cv2.imshow('image', cv_image)
        # k = cv2.waitKey(1)
        # if k == ord('s'):
        if self.idx % 120 == 0:
            cv2.imwrite(f'images/image_{self.idx}.png', cv_image)
            print('Image' + str(self.idx) + 'saved')
        # elif k == ord('q'):
        #     cv2.destroyAllWindows()
        #     self.destroy_node()
        #     rclpy.shutdown()
        self.idx += 1

def main(args=None):
    rclpy.init(args=args)
    image_subscriber = ImageSubscriber()
    rclpy.spin(image_subscriber)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

