import rclpy
import os
import csv
import busio
from datetime import datetime
from smbus2 import SMBus # library used for the power monitor sensor
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import board
import adafruit_bno055
from sensor_msgs.msg import Imu
import adafruit_bus_device.i2c_device as i2c_dev

CSV_FILE_PATH = "IMU_telemetry_log.csv"

# Initialising variables for the power monitor conversion
LTC2945_ADDR = 103
ADIN_OFF = 0x28
ADIN_NBYTES = 2
POWER_OFF = 0x5
POWER_NBYTES = 3
DELTA_SENSE_OFF = 0x14
DELTA_SENSE_NBYTES = 2
VIN_OFF = 0x1E
VIN_NBYTES = 2

# Class that handles the LTC2945 power monitor
class PowerMonitor:
    def __init__(self, bus=1, address=0x68, log_file="power_log.csv"):  # Choosing the communication address for the sensor, 0x68 in this case, can be changed to 0x67 if needed
        self._bus = SMBus(bus)
        self.addr = address
        self.DELTA_RES = 25E-3 / 2  # Dependent on the shunt resistor (2mohm) according to the schematic
        self.VIN_RES = 0.025 # According to the datasheet
        self.POWER_RES = 6E-7
        self.log_file = log_file
        self._init_csv()

    # Create a csv file for the LTC2945
    def _init_csv(self):
        if not os.path.exists(self.log_file):
            with open(self.log_file, mode='w', newline='') as file:
                writer = csv.writer(file)
                writer.writerow(["Timestamp", "Voltage (V)", "Current (A)", "Power (W)"])

    def get_data(self, off, nbytes):
        return self._bus.read_i2c_block_data(self.addr, off, nbytes)

    # Reading the coresponding address and converting the data from 16 to 12 bit values (voltage, current and power)
    def get_vin(self):
        data = self.get_data(0x1E, 2)
        vin = (data[0] << 4) | (data[1] >> 4)
        return vin * self.VIN_RES

    def get_delta_sense(self):
        data = self.get_data(0x14, 2)
        val = (data[0] << 4) | (data[1] >> 4)
        return val * self.DELTA_RES

    def get_power(self):
        data = self.get_data(0x05, 3)
        val = data[0] + (data[1] << 8) + (data[2] << 16)
        return val * self.POWER_RES

    # Log the voltage, the current and the power to the specific csv file
    def log(self):
        voltage = self.get_vin()
        current = self.get_delta_sense()
        power = self.get_power()
        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        with open(self.log_file, mode='a', newline='') as file:
            writer = csv.writer(file)
            writer.writerow([timestamp, voltage, current, power])

    def close(self):
        pass  # Nothing to close anymore

# IMU class, main communication ROS node
class IMUPublisher(Node):
    def __init__(self):
        super().__init__('imu_publisher')

        # Initialise IMU sensor
        i2c = board.I2C()
        self.sensor = adafruit_bno055.BNO055_I2C(i2c)

        # Initialise the power monior
        self.power = PowerMonitor()

        # Publishers for IMU data (all axes)
        self.orientation_publisher = self.create_publisher(Float32MultiArray, 'imu/orientation', 10)
        self.gyroscope_publisher = self.create_publisher(Float32MultiArray, 'imu/gyroscope', 10)
        self.linear_acceleration_publisher = self.create_publisher(Float32MultiArray, 'imu/linear_acceleration', 10)
        self.calibration_publisher = self.create_publisher(Float32MultiArray, 'imu/calibration_status', 10)
        self.imu_data_publisher = self.create_publisher(Imu, 'imu/data', 10)

        # Timer to publish IMU data
        self.create_timer(0.1, self.publish_imu_data)

        self.init_csv()

    # Create a different csv file for the IMU values
    def init_csv(self):
        if not os.path.exists(CSV_FILE_PATH):
            with open(CSV_FILE_PATH, mode='w', newline='') as file:
                writer = csv.writer(file)
                writer.writerow(["Timestamp", "Orientation X (°)", "Orientation Y (°)", "Orientation Z (°)",
                                 "Angular Velo X (rad/s)", "Angular Velo Y (rad/s)", "Angular Velo Z (rad/s)",
                                 "Linear Acceleration X (m/s^2)", "Linear Acceleration Y (m/s^2)",
                                 "Linear Acceleration Z (m/s^2)", "Calibration sys", "Calibration gyro",
                                 "Calibration accel", "Calibration mag"])

    # Log the values into the csv file
    def log_to_csv(self, orientation, gyroscope, linear_acceleration, calibration_status):

        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")

        with open(CSV_FILE_PATH, mode='a', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(
                [timestamp, orientation[0], orientation[1], orientation[2], gyroscope[0], gyroscope[1], gyroscope[2],
                 linear_acceleration[0], linear_acceleration[1], linear_acceleration[2],
                 calibration_status[0], calibration_status[1], calibration_status[2], calibration_status[3]])

    def publish_imu_data(self):
        try:
            # Read IMU data
            orientation = self.sensor.euler  # (X, Y, Z)
            gyroscope = self.sensor.gyro  # (X, Y, Z)
            linear_acceleration = self.sensor.linear_acceleration  # (X, Y, Z)
            calibration_status = self.sensor.calibration_status

            # Publish Orientation (Quaternion: X, Y, Z, W; Euler: X, Y, Z)
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

            # Publish Calibration
            if calibration_status is not None:
                msg = Float32MultiArray(
                    data=[calibration_status[0], calibration_status[1], calibration_status[2], calibration_status[3]])
                self.calibration_publisher.publish(msg)

            self.get_logger().info("IMU Data Published")
            imu_msg = Imu()

            # Fill orientation (no W value from euler, W is need only for quaternion)
            imu_msg.orientation.x = orientation[0] if orientation else 0.0
            imu_msg.orientation.y = orientation[1] if orientation else 0.0
            imu_msg.orientation.z = orientation[2] if orientation else 0.0
            imu_msg.orientation.w = 0.0  # Placeholder

            # Angular velocity
            imu_msg.angular_velocity.x = gyroscope[0] if gyroscope else 0.0
            imu_msg.angular_velocity.y = gyroscope[1] if gyroscope else 0.0
            imu_msg.angular_velocity.z = gyroscope[2] if gyroscope else 0.0

            # Linear acceleration
            imu_msg.linear_acceleration.x = linear_acceleration[0] if linear_acceleration else 0.0
            imu_msg.linear_acceleration.y = linear_acceleration[1] if linear_acceleration else 0.0
            imu_msg.linear_acceleration.z = linear_acceleration[2] if linear_acceleration else 0.0

            # Other values can be read from he IMU such as magnetic field and temperature

            self.imu_data_publisher.publish(imu_msg)
            self.log_to_csv(orientation, gyroscope, linear_acceleration, calibration_status)

            # Publish the power monitor's values
            self.power.log()

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

