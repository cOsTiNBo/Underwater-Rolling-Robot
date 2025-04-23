from flask import Flask, render_template, request, jsonify
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Float32
from sensor_msgs.msg import Imu

app = Flask(__name__)


class MotorControlNode(Node):
    def __init__(self):
        super().__init__('motor_control_web')

        # Publishers for mode and velocity control
        self.mode_publisher = self.create_publisher(Int32, 'motor_mode', 10)
        self.velocity_publisher_1 = self.create_publisher(Int32, 'motor1_velocity', 10)
        self.velocity_publisher_2 = self.create_publisher(Int32, 'motor2_velocity', 10)
        self.reboot_publisher = self.create_publisher(Int32, 'motor_reboot', 10)
        #self.telemetry = {
        #    "imu": {"orientation": [0, 0, 0, 1], "angular_velocity": [0, 0, 0], "linear_acceleration": [0, 0, 0]}}
        self.create_subscription(Imu, 'imu/data', self.update_imu, 10)
        self.sinusoidal_mode_pub = self.create_publisher(Int32, 'sinusoidal_mode', 10)
        self.sin_min_pub = self.create_publisher(Int32, 'sin_min', 10)
        self.sin_max_pub = self.create_publisher(Int32, 'sin_max', 10)
        self.sin_freq_pub = self.create_publisher(Int32, 'sinus_freq', 10)

    def update_imu(self, msg):
        """Update telemetry with IMU sensor data and log updates."""
        self.telemetry["imu"] = {
            "orientation": [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w],
            "angular_velocity": [msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z],
            "linear_acceleration": [msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z]
        }
        self.get_logger().info(f"Updated IMU Data: {self.telemetry['imu']}")

    def enable_velocity_mode(self):
        """Enable velocity mode"""
        mode_msg = Int32()
        mode_msg.data = 1  # Velocity Mode
        self.mode_publisher.publish(mode_msg)
        self.get_logger().info("Velocity Mode Enabled")

    def reboot_motors(self):
        """Reboot both motors."""
        reboot_msg = Int32()
        reboot_msg.data = 1  # Reboot command
        self.reboot_publisher.publish(reboot_msg)
        self.get_logger().info("Motors Rebooted")

    def send_velocity(self, speed):
        """Send velocity to both motors"""
        speed1 = int(speed)
        speed2 = -speed1  # Opposite direction

        msg1 = Int32()
        msg2 = Int32()
        msg1.data = speed1
        msg2.data = speed2

        self.velocity_publisher_1.publish(msg1)
        self.velocity_publisher_2.publish(msg2)

    def toggle_sinusoidal_mode(self, enable):
        self.sinusoidal_mode_pub.publish(Int32(data=1 if enable else 0))
        self.get_logger().info(f"Sinusoidal Mode: {'Enabled' if enable else 'Disabled'}")

    def set_sinusoidal_params(self, sin_min, sin_max, freq):
        self.sin_min_pub.publish(Int32(data=sin_min))
        self.sin_max_pub.publish(Int32(data=sin_max))
        self.sin_freq_pub.publish(Int32(data=freq))
        self.get_logger().info(f"Sinusoidal Params - Min: {sin_min}, Max: {sin_max}, Freq: {freq}")


rclpy.init()
ros_node = MotorControlNode()


@app.route('/')
def home():
    return render_template('index.html')


@app.route('/telemetry', methods=['GET'])
def telemetry():
    return jsonify(ros_node.telemetry)


@app.route('/control', methods=['POST'])
def control():
    if "mode" in request.form:
        mode = request.form.get("mode")
        if mode == "1":
            ros_node.enable_velocity_mode()
            return "Velocity Mode Enabled"

    if "reboot_motors" in request.form:
        ros_node.reboot_motors()
        return "Motors Rebooted"

    if "sinusoidal_mode" in request.form:
        enabled = request.form.get("sinusoidal_mode") == "1"
        ros_node.toggle_sinusoidal_mode(enabled)

        if enabled:
            try:
                sin_min = int(request.form.get("sin_min"))
                sin_max = int(request.form.get("sin_max"))
                freq = int(request.form.get("sinus_freq"))
                ros_node.set_sinusoidal_params(sin_min, sin_max, freq)
            except Exception as e:
                return f"Invalid sinusoidal params: {str(e)}", 400

        return "Sinusoidal Control Updated"

    velocity1 = request.form.get('velocity1')
    velocity2 = request.form.get('velocity2')

    if velocity1 is not None and velocity2 is not None:
        ros_node.send_velocity(velocity1)
        return "Motors Updated"

    return "Invalid Input", 400


if __name__ == '__main__':
    try:
        app.run(host='0.0.0.0', port=5000)
    except KeyboardInterrupt:
        ros_node.destroy_node()
        rclpy.shutdown()

