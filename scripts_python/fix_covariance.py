import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

class ImuCovarianceModifier(Node):
    def __init__(self):
        super().__init__("imu_covariance_modifier")

        # QoS profile for best-effort reliability
        qos_profile = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT, depth=10)

        # Subscribe to the input topic
        self.sub = self.create_subscription(Imu, "/drone0/sensor_measurements/imu", self.imu_callback, qos_profile)

        # Publisher for the modified IMU data
        self.pub = self.create_publisher(Imu, "/drone0/sensor_measurements/imu/corrected", qos_profile)

    def imu_callback(self, msg):
        # Modify the covariance values to 0.0
        msg.angular_velocity_covariance = [0.0] * 9
        msg.linear_acceleration_covariance = [0.0] * 9

        # Publish the modified message
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ImuCovarianceModifier()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

