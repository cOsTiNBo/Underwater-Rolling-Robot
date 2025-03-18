import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import board
import adafruit_bno055


class IMUPublisher(Node):
    def __init__(self):
        super().__init__('imu_publisher')

        # Initialize IMU sensor
        i2c = board.I2C()
        self.sensor = adafruit_bno055.BNO055_I2C(i2c)

        # Publishers for IMU data (all axes)
        self.orientation_publisher = self.create_publisher(Float32MultiArray, 'imu/orientation', 10)
        self.gyroscope_publisher = self.create_publisher(Float32MultiArray, 'imu/gyroscope', 10)
        self.linear_acceleration_publisher = self.create_publisher(Float32MultiArray, 'imu/linear_acceleration', 10)

        # Timer to publish IMU data
        self.create_timer(0.1, self.publish_imu_data)

    def publish_imu_data(self):
        try:
            # Read IMU data
            orientation = self.sensor.euler  # (X, Y, Z, W)
            gyroscope = self.sensor.gyro  # (X, Y, Z)
            linear_acceleration = self.sensor.linear_acceleration  # (X, Y, Z)

            # Publish Orientation (Quaternion: X, Y, Z, W)
            if orientation is not None:
                msg = Float32MultiArray(data=[orientation[0], orientation[1], orientation[2]])
                self.orientation_publisher.publish(msg)

            # Publish Angular Velocity (X, Y, Z)
            if gyroscope is not None:
                msg = Float32MultiArray(data=[gyroscope[0], gyroscope[1], gyroscope[2]])
                self.gyroscope_publisher.publish(msg)

            # Publish Linear Acceleration (X, Y, Z)
            if linear_acceleration is not None:
                msg = Float32MultiArray(data=[linear_acceleration[0], linear_acceleration[1], linear_acceleration[2]])
                self.linear_acceleration_publisher.publish(msg)

            self.get_logger().info("IMU Data Published")

        except Exception as e:
            self.get_logger().error(f"Failed to read IMU: {str(e)}")


def main(args=None):
    rclpy.init(args=args)
    imu_publisher = IMUPublisher()
    try:
        rclpy.spin(imu_publisher)
    except KeyboardInterrupt:
        imu_publisher.get_logger().info('Shutting down IMU publisher...')
    finally:
        imu_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

