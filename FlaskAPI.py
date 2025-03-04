from flask import Flask, render_template, request
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32

app = Flask(__name__)


class MotorControlNode(Node):
    def __init__(self):
        super().__init__('motor_control_web')
        self.mode_publisher = self.create_publisher(Int32, 'motor_mode', 10)
        self.velocity_publisher_1 = self.create_publisher(Int32, 'motor1_velocity', 10)
        self.velocity_publisher_2 = self.create_publisher(Int32, 'motor2_velocity', 10)
        self.position_publisher_1 = self.create_publisher(Int32, 'motor1_position', 10)
        self.position_publisher_2 = self.create_publisher(Int32, 'motor2_position', 10)

    def send_command(self, mode=None, velocity=None, position=None, motor=None):
        if mode is not None:
            msg = Int32()
            msg.data = mode
            self.mode_publisher.publish(msg)

        if motor == "1" and velocity is not None:
            msg = Int32()
            msg.data = velocity
            self.velocity_publisher_1.publish(msg)

        if motor == "2" and velocity is not None:
            msg = Int32()
            msg.data = velocity
            self.velocity_publisher_2.publish(msg)

        if motor == "1" and position is not None:
            msg = Int32()
            msg.data = position
            self.position_publisher_1.publish(msg)

        if motor == "2" and position is not None:
            msg = Int32()
            msg.data = position
            self.position_publisher_2.publish(msg)


rclpy.init()
ros_node = MotorControlNode()


@app.route('/')
def home():
    return render_template('index.html')


@app.route('/control', methods=['POST'])
def control():
    mode = request.form.get('mode')
    velocity = request.form.get('velocity')
    position = request.form.get('position')
    motors = request.form.get('motor').split(",") if request.form.get('motor') else []

    print(f"Received: Mode={mode}, Velocity={velocity}, Position={position}, Motors={motors}")

    if not motors:
        return "No motor selected", 400

    mode_msg = Int32()
    mode_msg.data = int(mode)
    ros_node.mode_publisher.publish(mode_msg)

    for motor in motors:
        if velocity and mode == "1":
            velocity_msg = Int32()
            velocity_msg.data = int(velocity)
            if motor == "1":
                ros_node.velocity_publisher_1.publish(velocity_msg)
            elif motor == "2":
                ros_node.velocity_publisher_2.publish(velocity_msg)

        if position and mode == "2":
            position_msg = Int32()
            position_msg.data = int(position)
            if motor == "1":
                ros_node.position_publisher_1.publish(position_msg)
            elif motor == "2":
                ros_node.position_publisher_2.publish(position_msg)

    return "Command Sent"


if __name__ == '__main__':
    try:
        app.run(host='0.0.0.0', port=5000)
    except KeyboardInterrupt:
        ros_node.destroy_node()
        rclpy.shutdown()

