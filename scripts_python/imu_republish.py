import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
import sys

# Node that republishes imu like:
# X axis ->  Z axis
# Z axis ->  X axis
# Y axis -> -Y axis
class ImuRepublish(Node):
    def __init__(self, imu_topic):
        super().__init__('imu_republish')
        sensor_qos = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.imu_sub = self.create_subscription(
            Imu,
            imu_topic,
            self.imu_callback,
            sensor_qos)

        self.imu_pub = self.create_publisher(Imu, imu_topic + '/corrected', 10)

    def imu_callback(self, msg):
        self.get_logger().info('Recieved imu data')
        new_msg = Imu()
        new_msg.header = msg.header
        new_msg.orientation = msg.orientation
        new_msg.orientation_covariance = msg.orientation_covariance
        new_msg.angular_velocity.x = msg.angular_velocity.z
        new_msg.angular_velocity.y = -msg.angular_velocity.y
        new_msg.angular_velocity.z = msg.angular_velocity.x
        new_msg.angular_velocity_covariance = msg.angular_velocity_covariance
        new_msg.linear_acceleration.x = msg.linear_acceleration.z
        new_msg.linear_acceleration.y = -msg.linear_acceleration.y
        new_msg.linear_acceleration.z = msg.linear_acceleration.x
        new_msg.linear_acceleration_covariance = msg.linear_acceleration_covariance
        self.imu_pub.publish(new_msg)

def main(args=None):
    rclpy.init(args=args)

    if len(sys.argv) < 2:
        print("Usage: imu_visualize.py <imu_topic>")
        return

    imu_topic = sys.argv[1]

    imu_republish = ImuRepublish(imu_topic)
    rclpy.spin(imu_republish)

    imu_republish.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
