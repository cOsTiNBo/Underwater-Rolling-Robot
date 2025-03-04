import rclpy
import csv
import os
from datetime import datetime
from rclpy.node import Node
from std_msgs.msg import Int32, Float32
from dynamixel_sdk import *

# Motor Configuration
DEVICENAME = '/dev/ttyUSB0'
BAUDRATE = 57600
PROTOCOL_VERSION = 2.0
DXL_ID_1 = 1
DXL_ID_2 = 2

# Control Table Addresses
ADDR_TORQUE_ENABLE = 64
ADDR_OPERATING_MODE = 11
ADDR_GOAL_VELOCITY = 104
ADDR_GOAL_POSITION = 116
ADDR_PRESENT_TEMPERATURE = 146
ADDR_PRESENT_CURRENT = 126
ADDR_PRESENT_VOLTAGE = 144

# Control Values
TORQUE_ENABLE = 1
TORQUE_DISABLE = 0
VELOCITY_CONTROL_MODE = 1
POSITION_CONTROL_MODE = 3

# CSV Logging
CSV_FILE_PATH = "dynamixel_telemetry_log.csv"


class DynamixelMotorController(Node):
    def __init__(self):
        super().__init__('dynamixel_motor_controller')

        # Initialize Port and Packet Handlers
        self.port_handler = PortHandler(DEVICENAME)
        self.packet_handler = PacketHandler(PROTOCOL_VERSION)

        if not self.port_handler.openPort() or not self.port_handler.setBaudRate(BAUDRATE):
            self.get_logger().error('Failed to open port or set baud rate.')
            rclpy.shutdown()
            return

        self.current_mode = None

        # Publishers for telemetry data
        self.temperature_publisher = self.create_publisher(Float32, 'motor_temperature', 10)
        self.current_publisher = self.create_publisher(Float32, 'motor_current', 10)
        self.voltage_publisher = self.create_publisher(Float32, 'motor_voltage', 10)

        # Subscribe to Flask control messages
        self.create_subscription(Int32, 'motor_mode', self.set_motor_mode, 10)
        self.create_subscription(Int32, 'motor1_velocity', self.set_motor1_velocity, 10)
        self.create_subscription(Int32, 'motor2_velocity', self.set_motor2_velocity, 10)
        self.create_subscription(Int32, 'motor1_position', self.set_motor1_position, 10)
        self.create_subscription(Int32, 'motor2_position', self.set_motor2_position, 10)

        self.get_logger().info('Dynamixel Motor Controller Ready.')

        # CSV Logger
        self.init_csv()
        self.create_timer(1.0, self.publish_telemetry)

    def init_csv(self):
        if not os.path.exists(CSV_FILE_PATH):
            with open(CSV_FILE_PATH, mode='w', newline='') as file:
                writer = csv.writer(file)
                writer.writerow(["Timestamp", "Temperature (°C)", "Current (A)", "Voltage (V)"])

    def log_to_csv(self, temperature, current, voltage):
        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        with open(CSV_FILE_PATH, mode='a', newline='') as file:
            writer = csv.writer(file)
            writer.writerow([timestamp, temperature, current, voltage])

    def set_motor_mode(self, msg):
        """Switch motor control mode (Velocity or Position)."""
        mode = msg.data
        self.disable_torque(DXL_ID_1)
        self.disable_torque(DXL_ID_2)

        if mode == 1:
            self.set_operating_mode(DXL_ID_1, VELOCITY_CONTROL_MODE)
            self.set_operating_mode(DXL_ID_2, VELOCITY_CONTROL_MODE)
            self.get_logger().info('Switched to Velocity Mode')
        elif mode == 2:
            self.set_operating_mode(DXL_ID_1, POSITION_CONTROL_MODE)
            self.set_operating_mode(DXL_ID_2, POSITION_CONTROL_MODE)
            self.get_logger().info('Switched to Position Mode')
        else:
            self.get_logger().warn('Invalid mode selected.')
            return

        self.enable_torque(DXL_ID_1)
        self.enable_torque(DXL_ID_2)
        self.current_mode = mode

    def set_motor1_velocity(self, msg):
        """Set velocity for Motor 1."""
        if self.current_mode == VELOCITY_CONTROL_MODE:
            velocity = msg.data
            self.packet_handler.write4ByteTxRx(self.port_handler, DXL_ID_1, ADDR_GOAL_VELOCITY, velocity)
            self.get_logger().info(f'Motor 1 - Velocity: {velocity}')

    def set_motor2_velocity(self, msg):
        """Set velocity for Motor 2."""
        if self.current_mode == VELOCITY_CONTROL_MODE:
            velocity = msg.data
            self.packet_handler.write4ByteTxRx(self.port_handler, DXL_ID_2, ADDR_GOAL_VELOCITY, velocity)
            self.get_logger().info(f'Motor 2 - Velocity: {velocity}')

    def set_motor1_position(self, msg):
        """Set position for Motor 1."""
        if self.current_mode == POSITION_CONTROL_MODE:
            position = msg.data
            self.packet_handler.write4ByteTxRx(self.port_handler, DXL_ID_1, ADDR_GOAL_POSITION, position)
            self.get_logger().info(f'Motor 1 - Position: {position}')

    def set_motor2_position(self, msg):
        """Set position for Motor 2."""
        if self.current_mode == POSITION_CONTROL_MODE:
            position = msg.data
            self.packet_handler.write4ByteTxRx(self.port_handler, DXL_ID_2, ADDR_GOAL_POSITION, position)
            self.get_logger().info(f'Motor 2 - Position: {position}')

    def publish_telemetry(self):
        """Read and publish motor telemetry."""
        try:
            temperature, _, dxl_error = self.packet_handler.read1ByteTxRx(self.port_handler, DXL_ID_1, ADDR_PRESENT_TEMPERATURE)
            if dxl_error == 0:
                self.temperature_publisher.publish(Float32(data=temperature))
            else:
                self.temperature_publisher.publish(Float32(data=0.0))

            current, _, dxl_error = self.packet_handler.read2ByteTxRx(self.port_handler, DXL_ID_1, ADDR_PRESENT_CURRENT)
            if dxl_error == 0:
                current = round(current / 1000.0, 3)
                self.current_publisher.publish(Float32(data=current))
            else:
                self.current_publisher.publish(Float32(data=0.0))

            voltage, _, dxl_error = self.packet_handler.read2ByteTxRx(self.port_handler, DXL_ID_1, ADDR_PRESENT_VOLTAGE)
            if dxl_error == 0:
                voltage = round(voltage / 10.0, 1)
                self.voltage_publisher.publish(Float32(data=voltage))
            else:
                self.voltage_publisher.publish(Float32(data=0.0))

            self.log_to_csv(temperature, current, voltage)

        except Exception as e:
            self.get_logger().error(f"Failed to read telemetry: {str(e)}")

    def set_operating_mode(self, dxl_id, mode):
        """Change motor operating mode."""
        self.packet_handler.write1ByteTxRx(self.port_handler, dxl_id, ADDR_OPERATING_MODE, mode)

    def enable_torque(self, dxl_id):
        """Enable torque for motor."""
        self.packet_handler.write1ByteTxRx(self.port_handler, dxl_id, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)

    def disable_torque(self, dxl_id):
        """Disable torque for motor."""
        self.packet_handler.write1ByteTxRx(self.port_handler, dxl_id, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)

    def destroy_node(self):

        self.disable_torque(DXL_ID_1)
        self.disable_torque(DXL_ID_2)
        self.port_handler.closePort()
        self.get_logger().info('Motors stopped, torque disabled, and port closed.')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    controller = DynamixelMotorController()
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
