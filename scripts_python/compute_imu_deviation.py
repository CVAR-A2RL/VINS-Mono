import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import numpy as np

class IMUAllanDeviationNode(Node):
    def __init__(self):
        super().__init__('imu_allan_deviation_node')
        self.declare_parameter('imu_topic', '/drone0/sensor_measurements/imu')
        self.declare_parameter('sampling_frequency', 150.0)  # Hz
        self.declare_parameter('measurement_duration', 120.0)  # seconds

        self.imu_topic = self.get_parameter('imu_topic').get_parameter_value().string_value
        self.sampling_frequency = self.get_parameter('sampling_frequency').get_parameter_value().double_value
        self.measurement_duration = self.get_parameter('measurement_duration').get_parameter_value().double_value

        self.acc_data = []
        self.gyr_data = []
        self.time_stamps = []

        self.subscription = self.create_subscription(
            Imu,
            self.imu_topic,
            self.imu_callback,
            10
        )

        self.get_logger().info(f'Subscribed to IMU topic: {self.imu_topic}')
        self.get_logger().info(f'Sampling Frequency: {self.sampling_frequency} Hz')
        self.get_logger().info(f'Measurement Duration: {self.measurement_duration} seconds')

    def imu_callback(self, msg):
        self.get_logger().info('IMU data received.')

        acc = [msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z]
        gyr = [msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z]

        self.acc_data.append(acc)
        self.gyr_data.append(gyr)
        self.time_stamps.append(self.get_clock().now().nanoseconds * 1e-9)

    def compute_allan_deviation(self, data, fs):
        data = np.array(data)
        N = len(data)
        M_max = 2**np.floor(np.log2(N // 2))
        M = np.logspace(0, np.log10(M_max), num=50).astype(int)
        tau = M / fs

        allan_var = np.zeros(len(M))
        for i, m in enumerate(M):
            two_m = 2 * m
            allan_var[i] = np.sum((data[two_m:] - 2 * data[m:-m] + data[:-two_m])**2)

        allan_var /= (2 * tau**2) * (N - 2 * M)
        allan_dev = np.sqrt(allan_var)

        return tau, allan_dev

    def compute_random_walk(self, tau, allan_dev):
        # Random walk noise is extracted from the slope of -0.5 in log-log plot
        random_walk_index = np.argmin(np.abs(np.log10(tau) + 0.5))
        random_walk_noise = allan_dev[random_walk_index] / np.sqrt(tau[random_walk_index])
        return random_walk_noise

    def analyze_data(self):
        if len(self.gyr_data) < 2 or len(self.acc_data) < 2:
            self.get_logger().warn('Not enough data collected to compute Allan deviation.')
            return

        fs = self.sampling_frequency

        # Gyroscope
        gyr_data = np.array(self.gyr_data)
        gyr_x = np.cumsum(gyr_data[:, 0]) / fs
        gyr_y = np.cumsum(gyr_data[:, 1]) / fs
        gyr_z = np.cumsum(gyr_data[:, 2]) / fs

        tau_x, allan_dev_x = self.compute_allan_deviation(gyr_x, fs)
        tau_y, allan_dev_y = self.compute_allan_deviation(gyr_y, fs)
        tau_z, allan_dev_z = self.compute_allan_deviation(gyr_z, fs)

        random_walk_x = self.compute_random_walk(tau_x, allan_dev_x)
        random_walk_y = self.compute_random_walk(tau_y, allan_dev_y)
        random_walk_z = self.compute_random_walk(tau_z, allan_dev_z)

        self.get_logger().info(f"Gyroscope Bias Random Walk Noise X: {random_walk_x}")
        self.get_logger().info(f"Gyroscope Bias Random Walk Noise Y: {random_walk_y}")
        self.get_logger().info(f"Gyroscope Bias Random Walk Noise Z: {random_walk_z}")

        # Accelerometer
        acc_data = np.array(self.acc_data)
        acc_x = np.cumsum(acc_data[:, 0]) / fs
        acc_y = np.cumsum(acc_data[:, 1]) / fs
        acc_z = np.cumsum(acc_data[:, 2]) / fs

        tau_x, allan_dev_x = self.compute_allan_deviation(acc_x, fs)
        tau_y, allan_dev_y = self.compute_allan_deviation(acc_y, fs)
        tau_z, allan_dev_z = self.compute_allan_deviation(acc_z, fs)

        random_walk_x = self.compute_random_walk(tau_x, allan_dev_x)
        random_walk_y = self.compute_random_walk(tau_y, allan_dev_y)
        random_walk_z = self.compute_random_walk(tau_z, allan_dev_z)

        self.get_logger().info(f"Accelerometer Bias Random Walk Noise X: {random_walk_x}")
        self.get_logger().info(f"Accelerometer Bias Random Walk Noise Y: {random_walk_y}")
        self.get_logger().info(f"Accelerometer Bias Random Walk Noise Z: {random_walk_z}")

        rclpy.shutdown()

    def run(self):
        self.get_logger().info('Collecting IMU data...')
        self.create_timer(self.measurement_duration, self.analyze_data)

def main(args=None):
    rclpy.init(args=args)
    node = IMUAllanDeviationNode()
    try:
        node.run()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

