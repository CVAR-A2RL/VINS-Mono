import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker
import sys
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
import numpy as np
from scipy.spatial.transform import Rotation as R

class ImuVisualize(Node):
    def __init__(self, imu_topic):
        super().__init__('imu_visualize')
        sensor_qos = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.imu_sub = self.create_subscription(
            Imu,
            imu_topic,
            self.imu_callback,
            sensor_qos)  

        # self.linear_pub = self.create_publisher(PoseStamped, 'imu_linear', 10)
        self.linear_pub = self.create_publisher(Marker, 'imu_linear', 1000)
        # self.angular_pub = self.create_publisher(PoseStamped, 'imu_angular', 10)
        self.angular_pub = self.create_publisher(Marker, 'imu_angular', 1000)

    def vector_to_quaternion(self, p1, p2):
        # Compute the vector from p1 to p2
        vector = np.array(p2) - np.array(p1)
    
        # Compute the magnitude of the vector
        magnitude = np.linalg.norm(vector)
    
        # Normalize the vector to obtain a direction
        if magnitude != 0:
            direction = vector / magnitude
        else:
            return vector, magnitude, [0.0, 0.0, 0.0, 1.0]
    
        # Define the reference vector (e.g., [1, 0, 0] in this case)
        reference_vector = np.array([1, 0, 0])
    
        # Compute the quaternion to rotate from the reference vector to the computed direction
        # Using scipy's Rotation.align_vectors
        rotation, _ = R.align_vectors([direction], [reference_vector])
        quaternion = rotation.as_quat()  # Returns quaternion in (x, y, z, w) format
    
        return vector, magnitude, quaternion

    def imu_callback(self, msg):
        self.get_logger().info('Recieved imu data')
        G = 9.81007
        # pose_linear = PoseStamped()
        # pose_linear.header = msg.header
        # pose_linear.header.frame_id = 'earth'
        # pose_linear.pose.position.x = msg.linear_acceleration.x
        # pose_linear.pose.position.y = msg.linear_acceleration.y
        # pose_linear.pose.position.z = msg.linear_acceleration.z
        # self.linear_pub.publish(pose_linear)
        _, module, quat = self.vector_to_quaternion([0, 0, 0], [msg.linear_acceleration.x / G, msg.linear_acceleration.y / G, msg.linear_acceleration.z / G])
        marker_linear = Marker()
        marker_linear.header = msg.header
        marker_linear.header.frame_id = 'body'
        marker_linear.type = Marker.ARROW
        marker_linear.action = 0
        marker_linear.pose.position.x = 0.0
        marker_linear.pose.position.y = 0.0
        marker_linear.pose.position.z = 0.0
        marker_linear.pose.orientation.x = quat[0]
        marker_linear.pose.orientation.y = quat[1]
        marker_linear.pose.orientation.z = quat[2]
        marker_linear.pose.orientation.w = quat[3]
        marker_linear.scale.x = module
        marker_linear.scale.y = 0.05
        marker_linear.scale.z = 0.05
        marker_linear.color.r = 1.0
        marker_linear.color.g = 0.0
        marker_linear.color.b = 0.0
        marker_linear.color.a = 1.0
        self.linear_pub.publish(marker_linear)

        # pose_angular = PoseStamped()
        # pose_angular.header = msg.header
        # pose_angular.header.frame_id = 'earth'
        # pose_angular.pose.position.x = msg.angular_velocity.x
        # pose_angular.pose.position.y = msg.angular_velocity.y
        # pose_angular.pose.position.z = msg.angular_velocity.z
        # self.angular_pub.publish(pose_angular)
        _, module, quat = self.vector_to_quaternion([0, 0, 0], [msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z])
        marker_angular = Marker()
        marker_angular.header = msg.header
        marker_angular.header.frame_id = 'body'
        marker_angular.type = Marker.ARROW
        marker_angular.action = 0
        marker_angular.pose.position.x = 0.0
        marker_angular.pose.position.y = 0.0
        marker_angular.pose.position.z = 0.0
        marker_angular.pose.orientation.x = quat[0]
        marker_angular.pose.orientation.y = quat[1]
        marker_angular.pose.orientation.z = quat[2]
        marker_angular.pose.orientation.w = quat[3]
        marker_angular.scale.x = module
        marker_angular.scale.y = 0.05
        marker_angular.scale.z = 0.05
        marker_angular.color.r = 0.0
        marker_angular.color.g = 1.0
        marker_angular.color.b = 0.0
        marker_angular.color.a = 1.0
        self.angular_pub.publish(marker_angular)

def main(args=None):
    rclpy.init(args=args)

    if len(sys.argv) < 2:
        print("Usage: imu_visualize.py <imu_topic>")
        return

    imu_topic = sys.argv[1]

    imu_visualize = ImuVisualize(imu_topic)

    rclpy.spin(imu_visualize)

    imu_visualize.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
