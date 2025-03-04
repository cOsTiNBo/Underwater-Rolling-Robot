import rclpy
import csv
import os
from rclpy.node import Node
from std_msgs.msg import Int32, Float32
from dynamixel_sdk import *  # Dynamixel SDK library
from sensor_msgs.msg import Joy

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
ADDR_PRESENT_TEMPERATURE = 146
ADDR_PRESENT_CURRENT = 126
ADDR_PRESENT_VOLTAGE = 144
ADDR_PRESENT_POSITION = 132
ADDR_PRESENT_VELOCITY = 128
ADDR_MOVING = 122

# Control Values
TORQUE_ENABLE = 1
TORQUE_DISABLE = 0
VELOCITY_CONTROL_MODE = 1

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

        # Enable Torque and set mode to velocity
        self.set_operating_mode(DXL_ID_1, VELOCITY_CONTROL_MODE)
        self.set_operating_mode(DXL_ID_2, VELOCITY_CONTROL_MODE)
        self._enable_torque(DXL_ID_1)
        self._enable_torque(DXL_ID_2)

        # Publishers for telemetry data
        self.temperature_publisher = self.create_publisher(Float32, 'motor_temperature', 10)
        self.current_publisher = self.create_publisher(Float32, 'motor_current', 10)
        self.voltage_publisher = self.create_publisher(Float32, 'motor_voltage', 10)
        self.position_publisher = self.create_publisher(Float32, 'motor_position_degrees', 10)
        self.velocity_publisher = self.create_publisher(Float32, 'motor_velocity_speed', 10)

        # Timer to periodically read telemetry
        self.create_timer(1.0, self.publish_telemetry)

        # Subscribe to controller input
        self.create_subscription(Joy, '/joy', self.joy_callback, 10)

        self.get_logger().info('Motor Controller ready (PS4 Controller).')

        # Initialize CSV file
        self.init_csv()

    def set_operating_mode(self, motor_id, mode):
        self.packet_handler.write1ByteTxRx(self.port_handler, motor_id, ADDR_OPERATING_MODE, mode)

    def _enable_torque(self, motor_id):
        self.packet_handler.write1ByteTxRx(self.port_handler, motor_id, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
        self.get_logger().info(f"Torque enabled for motor {motor_id}")

    def _disable_torque(self, motor_id):
        self.packet_handler.write1ByteTxRx(self.port_handler, motor_id, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
        self.get_logger().info(f"Torque disabled for motor {motor_id}")

    def init_csv(self):
        if not os.path.exists(CSV_FILE_PATH):
            with open(CSV_FILE_PATH, mode='w', newline='') as file:
                writer = csv.writer(file)
                writer.writerow(
                    ["Timestamp", "Temperature (°C)", "Current (A)", "Voltage (V)", "Position (°)", "Velocity (Units)",
                     "Status"])

    def log_to_csv(self, temperature, current, voltage, position, velocity, moving):
        timestamp = self.get_clock().now().to_msg().sec
        status = "Moving" if moving == 1 else "Idle"

        with open(CSV_FILE_PATH, mode='a', newline='') as file:
            writer = csv.writer(file)
            writer.writerow([timestamp, temperature, current, voltage, position, velocity, status])

    def joy_callback(self, msg):
        # Read joystick vertical axis for velocity (left Y-axis)
        joystick_value = msg.axes[1]  # [-1.0, 1.0]
        motor_velocity = int(joystick_value * 200)  # Scale to motor range

        # Read button inputs
        motor1_selected = msg.buttons[4]  # L1 - Motor 1
        motor2_selected = msg.buttons[5]  # R1 - Motor 2

        if motor1_selected and not motor2_selected:
            self.set_motor_velocity(DXL_ID_1, motor_velocity)
        elif motor2_selected and not motor1_selected:
            self.set_motor_velocity(DXL_ID_2, motor_velocity)
        else:
            self.set_motor_velocity(DXL_ID_1, -motor_velocity)
            self.set_motor_velocity(DXL_ID_2, motor_velocity)

    def set_motor_velocity(self, motor_id, velocity):
        # Convert negative values from two's complement
        if velocity < 0:
            velocity = velocity + 0x100000000  # Convert from signed to unsigned
        self.packet_handler.write4ByteTxRx(self.port_handler, motor_id, ADDR_GOAL_VELOCITY, velocity)

    def publish_telemetry(self):
        try:
            temperature, result, dxl_error = self.packet_handler.read1ByteTxRx(self.port_handler, DXL_ID_1,
                                                                               ADDR_PRESENT_TEMPERATURE)
            if dxl_error == 0:
                self.temperature_publisher.publish(Float32(data=float(temperature)))
            else:
                self.temperature_publisher.publish(Float32(data=0.0))

            current, result, dxl_error = self.packet_handler.read2ByteTxRx(self.port_handler, DXL_ID_1,
                                                                           ADDR_PRESENT_CURRENT)
            if dxl_error == 0:
                self.current_publisher.publish(Float32(data=float(current) / 1000.0))
            else:
                self.current_publisher.publish(Float32(data=0.0))

            voltage, result, dxl_error = self.packet_handler.read2ByteTxRx(self.port_handler, DXL_ID_1,
                                                                           ADDR_PRESENT_VOLTAGE)
            if dxl_error == 0:
                self.voltage_publisher.publish(Float32(data=float(voltage) / 10.0))
            else:
                self.voltage_publisher.publish(Float32(data=0.0))

            position, result, dxl_error = self.packet_handler.read2ByteTxRx(self.port_handler, DXL_ID_1,
                                                                            ADDR_PRESENT_POSITION)
            if dxl_error == 0:
                self.voltage_publisher.publish(Float32(data=float(position) / 10.0))
            else:
                self.voltage_publisher.publish(Float32(data=0.0))

            velocity, result, dxl_error = self.packet_handler.read4ByteTxRx(self.port_handler, DXL_ID_1,
                                                                            ADDR_PRESENT_VELOCITY)
            # Convert to integer
            if dxl_error == 0:
                if velocity > 0x7FFFFFFF:
                    velocity -= 0x100000000  # Convert from unsigned to signed
                    self.voltage_publisher.publish(Float32(data=float(velocity)))
            else:
                self.voltage_publisher.publish(Float32(data=0.0))

            moving, result, dxl_error = self.packet_handler.read4ByteTxRx(self.port_handler, DXL_ID_1, ADDR_MOVING)
            if dxl_error == 0:
                self.voltage_publisher.publish(Float32(data=float(moving) / 10.0))
            else:
                self.voltage_publisher.publish(Float32(data=0.0))

            position_degrees = position * 0.087891
            velocity_rev = velocity * 0.229
            voltageV = voltage / 10

            # if velocity > 65000:
            #    velocity = 0.0
            # if current > 65000:
            #    current = 0.0

            self.voltage_publisher.publish(Float32(data=float(voltageV) / 10.0))

            self.log_to_csv(temperature, current, voltageV, position_degrees, velocity_rev, moving)

        except Exception as e:
            self.get_logger().error(f"Failed to read telemetry: {str(e)}")


def main(args=None):
    rclpy.init(args=args)
    controller = DynamixelMotorController()
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        controller.get_logger().info('Shutting down motor controller')
    finally:
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()


