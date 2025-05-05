import sys
import numpy as np
from PyQt5.QtWidgets import QApplication, QMainWindow, QSlider, QDial, QLabel, QPushButton, QCheckBox, QRadioButton, \
    QWidget, QTableWidget, QTableWidgetItem, QVBoxLayout # PyQt5 library for visualisation widgets
from PyQt5.QtCore import Qt, QRect, QTimer
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Float32MultiArray



class MotorDualControlGUI(QMainWindow):
    def __init__(self):
        super().__init__()
        self.node = rclpy.create_node('motor_dual_control_gui')
        self.init_ros()
        self.setupUi()
        self.start_ros_timer()

    def init_ros(self):
        # Publishers for the mode and velocity control
        self.mode_publisher = self.node.create_publisher(Int32, 'motor_mode', 10)
        self.velocity_publisher_1 = self.node.create_publisher(Int32, 'motor1_velocity', 10)
        self.velocity_publisher_2 = self.node.create_publisher(Int32, 'motor2_velocity', 10)
        self.position_publisher_1 = self.node.create_publisher(Int32, 'motor1_position', 10)
        self.position_publisher_2 = self.node.create_publisher(Int32, 'motor2_position', 10)

        # Publishers for the telemetry
        self.node.create_subscription(Float32MultiArray, 'imu/orientation', self.update_orientation, 10)
        self.node.create_subscription(Float32MultiArray, 'imu/gyroscope', self.update_gyroscope, 10)
        self.node.create_subscription(Float32MultiArray, 'imu/linear_acceleration', self.update_linear_acceleration, 10)

    def setupUi(self):
        # Design of the visualisation
        self.setWindowTitle("Dual Motor Control (Velocity/Position - Telemetry)")
        self.setGeometry(100, 100, 1200, 450)
        self.centralwidget = QWidget(self)
        self.setCentralWidget(self.centralwidget)

        # Motor Selection Checkboxes
        self.motor1Checkbox = QCheckBox("Motor 1", self.centralwidget)
        self.motor1Checkbox.setGeometry(QRect(50, 50, 100, 30))
        self.motor1Checkbox.setChecked(True)

        self.motor2Checkbox = QCheckBox("Motor 2", self.centralwidget)
        self.motor2Checkbox.setGeometry(QRect(50, 90, 100, 30))
        self.motor2Checkbox.setChecked(True)

        # Mode Selection (Velocity/Position)
        self.velMode = QRadioButton("Velocity Mode", self.centralwidget)
        self.velMode.setGeometry(QRect(150, 20, 150, 30))
        self.velMode.clicked.connect(self.set_velocity_mode)

        self.posMode = QRadioButton("Position Mode", self.centralwidget)
        self.posMode.setGeometry(QRect(350, 20, 150, 30))
        self.posMode.clicked.connect(self.set_position_mode)

        # Velocity Slider
        self.velocitySlider = QSlider(Qt.Horizontal, self.centralwidget)
        self.velocitySlider.setGeometry(QRect(150, 100, 200, 30))
        self.velocitySlider.setMinimum(-330)
        self.velocitySlider.setMaximum(330)
        self.velocitySlider.setValue(0)
        self.velocitySlider.valueChanged.connect(self.update_velocity_label)
        self.velocityLabel = QLabel("Velocity: 0", self.centralwidget)
        self.velocityLabel.setGeometry(QRect(175, 70, 150, 20))

        # Position Dial (0-360 degrees)
        self.positionDial = QDial(self.centralwidget)
        self.positionDial.setGeometry(QRect(400, 100, 120, 120))
        self.positionDial.setMinimum(0)
        self.positionDial.setMaximum(4095)
        self.positionDial.setValue(0)
        self.positionDial.valueChanged.connect(self.update_position_label)
        self.positionLabel = QLabel("Position: 0°", self.centralwidget)
        self.positionLabel.setGeometry(QRect(425, 70, 150, 20))

        # Telemetry Table
        self.imuTable = QTableWidget(self.centralwidget)
        self.imuTable.setGeometry(QRect(550, 50, 550, 150))
        self.imuTable.setRowCount(3)
        self.imuTable.setColumnCount(4)
        self.imuTable.setHorizontalHeaderLabels(["X", "Y", "Z", "W"])
        self.imuTable.setVerticalHeaderLabels(["Orientation", "Gyroscope", "Linear Acceleration"])

        # Start and Stop Buttons
        self.startButton = QPushButton("Start", self.centralwidget)
        self.startButton.setGeometry(QRect(175, 350, 100, 40))
        self.startButton.clicked.connect(self.send_value)

        self.stopButton = QPushButton("Stop", self.centralwidget)
        self.stopButton.setGeometry(QRect(325, 350, 100, 40))
        self.stopButton.clicked.connect(self.stop_motor)

        self.centralwidget.layout = QVBoxLayout(self.centralwidget)
        self.centralwidget.setLayout(self.centralwidget.layout)

    def set_velocity_mode(self):
        msg = Int32()
        msg.data = 1  # Velocity mode
        self.mode_publisher.publish(msg)
        self.positionDial.setEnabled(False)
        self.velocitySlider.setEnabled(True)

    def set_position_mode(self):
        msg = Int32()
        msg.data = 2  # Position mode
        self.mode_publisher.publish(msg)
        self.velocitySlider.setEnabled(False)
        self.positionDial.setEnabled(True)

    def update_velocity_label(self, value):
        self.velocityLabel.setText(f"Velocity: {value}")

    def update_position_label(self, value):
        scaled_value = round((value / 4095) * 360)
        self.positionLabel.setText(f"Position: {scaled_value}°")

    # Reading data from the IMU
    def update_orientation(self, msg):
        for i in range(4):
            self.imuTable.setItem(0, i, QTableWidgetItem(f"{msg.data[i]:.2f}"))

    def update_gyroscope(self, msg):
        for i in range(3):
            self.imuTable.setItem(1, i, QTableWidgetItem(f"{msg.data[i]:.2f}"))

    def update_linear_acceleration(self, msg):
        for i in range(3):
            self.imuTable.setItem(2, i, QTableWidgetItem(f"{msg.data[i]:.2f}"))

    # Setting the velocity to both motors
    def send_value(self):
        velocity_value = self.velocitySlider.value()
        position_value = self.positionDial.value()

        if self.motor1Checkbox.isChecked():
            if self.velMode.isChecked():
                self.velocity_publisher_1.publish(Int32(data=velocity_value))
            elif self.posMode.isChecked():
                self.position_publisher_1.publish(Int32(data=position_value))

        if self.motor2Checkbox.isChecked():
            if self.velMode.isChecked():
                self.velocity_publisher_2.publish(Int32(data=velocity_value))
            elif self.posMode.isChecked():
                self.position_publisher_2.publish(Int32(data=position_value))
    # Stop the motors
    def stop_motor(self):
        self.velocity_publisher_1.publish(Int32(data=0))
        self.velocity_publisher_2.publish(Int32(data=0))
        self.position_publisher_1.publish(Int32(data=0))
        self.position_publisher_2.publish(Int32(data=0))

    def start_ros_timer(self):
        self.timer = QTimer(self)
        self.timer.timeout.connect(lambda: rclpy.spin_once(self.node, timeout_sec=0.1))
        self.timer.start(100)


def main(args=None):
    rclpy.init(args=args)
    app = QApplication(sys.argv)
    gui = MotorDualControlGUI()
    gui.show()
    app.exec_()
    gui.node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

