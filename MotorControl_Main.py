import rclpy
import csv
import os
import math
from datetime import datetime
from rclpy.node import Node
from std_msgs.msg import Int32, Float32
from dynamixel_sdk import *
from sensor_msgs.msg import Joy

# Motor Configuration
DEVICENAME = '/dev/ttyUSB0'
BAUDRATE = 57600 # Can be incresed for faster responses
PROTOCOL_VERSION = 2.0
DXL_ID_1 = 1 # Different IDs for the motors in order to control individually
DXL_ID_2 = 2

# Control Table Addresses
ADDR_TORQUE_ENABLE = 64
ADDR_OPERATING_MODE = 11
ADDR_GOAL_VELOCITY = 104
ADDR_GOAL_POSITION = 116
ADDR_PRESENT_TEMPERATURE = 146
ADDR_PRESENT_CURRENT = 126
ADDR_PRESENT_VOLTAGE = 144
ADDR_PRESENT_POSITION = 132
ADDR_PRESENT_VELOCITY = 128

# Control Values
TORQUE_ENABLE = 1
TORQUE_DISABLE = 0
VELOCITY_CONTROL_MODE = 1
POSITION_CONTROL_MODE = 3

CSV_FILE_PATH = "dynamixel_telemetry_log.csv"


class DynamixelDualMotorController(Node):
    def __init__(self):
        super().__init__('dynamixel_dual_motor_controller')

        # Initialise Port and Packet Handlers
        self.port_handler = PortHandler(DEVICENAME)
        self.packet_handler = PacketHandler(PROTOCOL_VERSION)

        if not self.port_handler.openPort() or not self.port_handler.setBaudRate(BAUDRATE):
            self.get_logger().error('Failed to open port or set baud rate.')
            rclpy.shutdown()
            return
        # Initialiase variables
        self.current_mode = None
        self.sinusoidal_mode = False
        self.min_velocity = 0
        self.max_velocity = 0
        self.frequency = 0.0
        self.phase = 0.0
        self.goal_velocity = 0

        # Publishers for telemetry data
        self.temperature_publisher = self.create_publisher(Float32, 'motor_temperature', 10)
        self.current_publisher = self.create_publisher(Float32, 'motor_current', 10)
        self.voltage_publisher = self.create_publisher(Float32, 'motor_voltage', 10)
        self.position_publisher = self.create_publisher(Float32, 'motor_position_degrees', 10)
        self.velocity_publisher = self.create_publisher(Float32, 'motor_velocity_speed', 10)
        self.goal_velocity_publisher = self.create_publisher(Int32, 'motor_goal_velocity', 10)

        # Timer to periodically read telemetry
        self.create_timer(1.0, self.publish_telemetry)
        self.create_timer(0.01, self.send_sinusoidal_velocity)

        # Subscribe to topics for both motors (liner and sinusoidal velocity)
        self.create_subscription(Int32, 'motor_mode', self.set_motor_mode, 10)
        self.create_subscription(Int32, 'motor_reboot', self.reboot_motors, 10)
        self.create_subscription(Int32, 'motor1_velocity', self.set_motor1_velocity, 10)
        self.create_subscription(Int32, 'motor2_velocity', self.set_motor2_velocity, 10)
        self.create_subscription(Int32, 'motor1_position', self.set_motor1_position, 10)
        self.create_subscription(Int32, 'motor2_position', self.set_motor2_position, 10)
        self.create_subscription(Int32, 'sinusoidal_mode', self.set_sinusoidal_mode, 10)
        self.create_subscription(Int32, 'sin_min', self.set_min_velocity, 10)
        self.create_subscription(Int32, 'sin_max', self.set_max_velocity, 10)
        self.create_subscription(Int32, 'sinus_freq', self.set_frequency, 10)

        self.get_logger().info('Dual Motor Controller ready (Velocity/Position).')

        # Subscribe to the joystick topic
        self.create_subscription(Joy, '/joy', self.joy_callback, 10)

        # Initialise csv writing
        self.init_csv()

    def init_csv(self):
    # Reading and writing important parameters from the motors
        if not os.path.exists(CSV_FILE_PATH):
            with open(CSV_FILE_PATH, mode='w', newline='') as file:
                writer = csv.writer(file)
                writer.writerow(["Timestamp", "Temperature (°C)", "Current (mA)", "Voltage (V)", "Position (°)",
                                 "Velocity (rev/min)", "Goal Velocity"])
    # Log the parameters into csv
    def log_to_csv(self, temperature, current, voltage, position, velocity, goal_velocity):

        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")

        with open(CSV_FILE_PATH, mode='a', newline='') as file:
            writer = csv.writer(file)
            writer.writerow([timestamp, temperature, current, voltage, position, velocity, goal_velocity])

    def set_motor_mode(self, msg):
        # Choosing an operation mode
        mode = msg.data
        self.disable_torque(DXL_ID_1) # Disabling torque before switching
        self.disable_torque(DXL_ID_2)

        if mode == 1:
            self.set_operating_mode(DXL_ID_1, VELOCITY_CONTROL_MODE)
            self.set_operating_mode(DXL_ID_2, VELOCITY_CONTROL_MODE)
            self.current_mode = 'velocity'
            self.get_logger().info('Switched to Velocity Mode')
        elif mode == 2:
            self.set_operating_mode(DXL_ID_1, POSITION_CONTROL_MODE)
            self.set_operating_mode(DXL_ID_2, POSITION_CONTROL_MODE)
            self.current_mode = 'position'
            self.get_logger().info('Switched to Position Mode')
        else:
            self.get_logger().warn('Invalid mode selected')
            return

        self.enable_torque(DXL_ID_1) # Enabling torque after switching
        self.enable_torque(DXL_ID_2)

    def joy_callback(self, msg):
        # Read joystick vertical axis for velocity (left Y-axis)
        joystick_value = msg.axes[1]
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

    def reboot_motors(self, msg):
        # Reboot the motors
        if msg.data == 1:
            # Disable torque before rebooting
            self.packet_handler.write1ByteTxRx(self.port_handler, DXL_ID_1, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
            self.packet_handler.write1ByteTxRx(self.port_handler, DXL_ID_2, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)

            dxl_comm_result, dxl_error = self.packet_handler.reboot(self.port_handler, DXL_ID_1)
            if dxl_comm_result != COMM_SUCCESS:
                self.get_logger().error(f"Motor 1 Reboot Error: {self.packet_handler.getTxRxResult(dxl_comm_result)}")
            elif dxl_error != 0:
                self.get_logger().error(f"Motor 1 Error: {self.packet_handler.getRxPacketError(dxl_error)}")
            else:
                self.get_logger().info("Motor 1 Rebooted")

            dxl_comm_result, dxl_error = self.packet_handler.reboot(self.port_handler, DXL_ID_2)
            if dxl_comm_result != COMM_SUCCESS:
                self.get_logger().error(f"Motor 2 Reboot Error: {self.packet_handler.getTxRxResult(dxl_comm_result)}")
            elif dxl_error != 0:
                self.get_logger().error(f"Motor 2 Error: {self.packet_handler.getRxPacketError(dxl_error)}")
            else:
                self.get_logger().info("Motor 2 Rebooted")

    def set_motor1_velocity(self, msg):
        # Set linear velocity to the first motor - each motor can be controlled individually in this section, same happens to the second motor
        if self.current_mode == 'velocity' and not self.sinusoidal_mode:
            velocity = msg.data
            self.manual_velocity = velocity
            self.packet_handler.write4ByteTxRx(self.port_handler, DXL_ID_1, ADDR_GOAL_VELOCITY, velocity)
            self.get_logger().info(f'Motor 1 - Velocity: {velocity}')

    def set_motor2_velocity(self, msg):
        # Set linear velocity to the second motor
        if self.current_mode == 'velocity' and not self.sinusoidal_mode:
            velocity = msg.data
            self.packet_handler.write4ByteTxRx(self.port_handler, DXL_ID_2, ADDR_GOAL_VELOCITY, velocity)
            self.get_logger().info(f'Motor 2 - Velocity: {velocity}')

    def send_sinusoidal_velocity(self):
        # Set sinusoidal velocity to boh motors - one motor has the invers speed of the other
        if self.current_mode != 'velocity' or not self.sinusoidal_mode:
            return

        # Setting all the parameters to create the sinus wave
        amplitude = (self.max_velocity - self.min_velocity)
        offset = (self.max_velocity + self.min_velocity)
        velocity = int(offset + amplitude * math.sin(2 * math.pi * self.frequency * self.phase))

        if velocity < 0:
            velocity = velocity + 0x100000000

        self.packet_handler.write4ByteTxRx(self.port_handler, DXL_ID_1, ADDR_GOAL_VELOCITY, velocity)
        self.packet_handler.write4ByteTxRx(self.port_handler, DXL_ID_2, ADDR_GOAL_VELOCITY, -velocity)

        self.phase += 0.01

    def set_motor1_position(self, msg):
        if self.current_mode == 'position':
            position = msg.data
            self.packet_handler.write4ByteTxRx(
                self.port_handler, DXL_ID_1, ADDR_GOAL_POSITION, position)
            self.get_logger().info(f'Motor 1 - Position: {position}')

    def set_motor2_position(self, msg):
        if self.current_mode == 'position':
            position = msg.data
            self.packet_handler.write4ByteTxRx(
                self.port_handler, DXL_ID_2, ADDR_GOAL_POSITION, position)
            self.get_logger().info(f'Motor 2 - Position: {position}')

    def set_sinusoidal_mode(self, msg):
        # Bool value to turn on and off the sinusoidal mode
        self.sinusoidal_mode = bool(msg.data)
        self.get_logger().info(f'Sinusoidal mode set to: {self.sinusoidal_mode}')

    def set_min_velocity(self, msg):
        # Setting the minimum velocity for the sinus wave
        self.min_velocity = msg.data
        self.get_logger().info(f'Sinusoidal amplitude set to: {self.min_velocity}')

    def set_max_velocity(self, msg):
        # Setting the maximum velocity for the sinus wave
        self.max_velocity = msg.data
        self.get_logger().info(f'Sinusoidal offset set to: {self.max_velocity}')

    def set_frequency(self, msg):
        # Setting the frequency for the sinus wave
        self.frequency = msg.data
        self.get_logger().info(f'Sinusoidal frequency set to: {self.frequency}')

    def publish_telemetry(self):
        # Publish telemtery from the first motor
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
            # Convert to signed 32-bit integer
            if dxl_error == 0:
                if velocity > 0x7FFFFFFF:
                    velocity -= 0x100000000
                self.voltage_publisher.publish(Float32(data=float(velocity)))
            else:
                self.voltage_publisher.publish(Float32(data=0.0))

            # Convert the values according to the Dynamixel Wizard to degrees, rev/min and volts
            position_degrees = position * 0.087891
            velocity_rev = velocity * 0.229
            voltageV = voltage / 10

            if current > 65000:
                current = 0.0

            self.voltage_publisher.publish(Float32(data=float(voltageV) / 10.0))

            # Log the values into the csv file
            self.log_to_csv(temperature, current, voltageV, position_degrees, velocity_rev, self.goal_velocity * 0.229)

        except Exception as e:
            self.get_logger().error(f"Failed to read telemetry: {str(e)}")

    def set_operating_mode(self, dxl_id, mode):
        self.packet_handler.write1ByteTxRx(
            self.port_handler, dxl_id, ADDR_OPERATING_MODE, mode)

    def enable_torque(self, dxl_id):
        self.packet_handler.write1ByteTxRx(
            self.port_handler, dxl_id, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)

    def disable_torque(self, dxl_id):
        self.packet_handler.write1ByteTxRx(
            self.port_handler, dxl_id, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)

    def destroy_node(self):
        self.disable_torque(DXL_ID_1)
        self.disable_torque(DXL_ID_2)
        self.port_handler.closePort()
        self.get_logger().info('Motors stopped, torque disabled, and port closed.')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    controller = DynamixelDualMotorController()
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        controller.get_logger().info('Shutting down dual motor controller...')
    finally:
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

