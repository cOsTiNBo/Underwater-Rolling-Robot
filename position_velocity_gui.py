import sys
import numpy as np
from PyQt5.QtWidgets import QApplication, QMainWindow, QSlider, QDial, QLabel, QPushButton, QCheckBox, QRadioButton, \
    QWidget, QTableWidget, QTableWidgetItem, QVBoxLayout
from PyQt5.QtCore import Qt, QRect, QTimer
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Float32
import matplotlib.pyplot as plt
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
import matplotlib

matplotlib.use("Qt5Agg")


class MotorDualControlGUI(QMainWindow):
    def __init__(self):
        super().__init__()
        self.node = rclpy.create_node('motor_dual_control_gui')
        self.init_ros()
        self.setupUi()
        self.start_ros_timer()
        self.init_graph()

    def init_ros(self):
        self.mode_publisher = self.node.create_publisher(Int32, 'motor_mode', 10)
        self.velocity_publisher_1 = self.node.create_publisher(Int32, 'motor1_velocity', 10)
        self.velocity_publisher_2 = self.node.create_publisher(Int32, 'motor2_velocity', 10)
        self.position_publisher_1 = self.node.create_publisher(Int32, 'motor1_position', 10)
        self.position_publisher_2 = self.node.create_publisher(Int32, 'motor2_position', 10)

        self.temperature_subscriber = self.node.create_subscription(Float32, 'motor_temperature',
                                                                    self.update_temperature, 10)
        self.current_subscriber = self.node.create_subscription(Float32, 'motor_current', self.update_current, 10)
        self.voltage_subscriber = self.node.create_subscription(Float32, 'motor_voltage', self.update_voltage, 10)

    def setupUi(self):
        self.setWindowTitle("Dual Motor Control (Velocity/Position - Telemetry)")
        self.setGeometry(100, 100, 800, 450)
        self.centralwidget = QWidget(self)
        self.setCentralWidget(self.centralwidget)

        # Motor Selection Checkboxes
        self.motor1Checkbox = QCheckBox("Motor 1", self.centralwidget)
        self.motor1Checkbox.setGeometry(QRect(50, 50, 100, 30))
        self.motor1Checkbox.setChecked(True)

        self.motor2Checkbox = QCheckBox("Motor 2", self.centralwidget)
        self.motor2Checkbox.setGeometry(QRect(50, 90, 100, 30))
        self.motor2Checkbox.setChecked(True)

        # Mode Selection (Velocity/Position - Exclusive)
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

        # Position Dial (Scaled 0-360)
        self.positionDial = QDial(self.centralwidget)
        self.positionDial.setGeometry(QRect(400, 100, 120, 120))
        self.positionDial.setMinimum(0)
        self.positionDial.setMaximum(4095)
        self.positionDial.setValue(0)
        self.positionDial.valueChanged.connect(self.update_position_label)
        self.positionLabel = QLabel("Position: 0°", self.centralwidget)
        self.positionLabel.setGeometry(QRect(425, 70, 150, 20))

        # Telemetry Table
        self.telemetryTable = QTableWidget(self.centralwidget)
        self.telemetryTable.setGeometry(QRect(550, 50, 215, 115))
        self.telemetryTable.setRowCount(3)
        self.telemetryTable.setColumnCount(2)
        self.telemetryTable.setHorizontalHeaderLabels(["Metric", "Value"])
        self.telemetryTable.setItem(0, 0, QTableWidgetItem("Temperature"))
        self.telemetryTable.setItem(1, 0, QTableWidgetItem("Current"))
        self.telemetryTable.setItem(2, 0, QTableWidgetItem("Voltage"))

        # Start and Stop Buttons
        self.startButton = QPushButton("Start", self.centralwidget)
        self.startButton.setGeometry(QRect(175, 350, 100, 40))
        self.startButton.clicked.connect(self.send_value)

        self.stopButton = QPushButton("Stop", self.centralwidget)
        self.stopButton.setGeometry(QRect(325, 350, 100, 40))
        self.stopButton.clicked.connect(self.stop_motor)

        self.centralwidget.layout = QVBoxLayout(self.centralwidget)
        self.centralwidget.setLayout(self.centralwidget.layout)

    def init_graph(self):
        self.figure = plt.figure()  # Create a new figure
        self.ax = self.figure.add_subplot(111)  # Add a subplot
        self.canvas = FigureCanvas(self.figure)  # Create Matplotlib canvas
        # self.centralwidget.layout.addWidget(self.canvas)
        self.graph_window = QMainWindow(self)
        self.graph_window.setWindowTitle("Real-Time Graph")
        self.graph_window.setGeometry(200, 200, 600, 400)
        self.graph_widget = QWidget()
        self.graph_layout = QVBoxLayout(self.graph_widget)
        self.graph_layout.addWidget(self.canvas)
        self.graph_window.setCentralWidget(self.graph_widget)
        self.graph_window.show()

        self.x_data = []
        self.y_data = []

        self.timer = QTimer()
        self.timer.timeout.connect(self.update_graph)
        self.timer.start(1000)

    def update_graph(self):
        self.ax.clear()
        if len(self.x_data) > 0:
            self.ax.plot(self.x_data, self.velocity_data, label='Velocity', color='blue')
            self.ax.set_xlabel("Time")
            self.ax.set_ylabel("Value")
            self.ax.legend()
            self.canvas.draw()

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

    def update_temperature(self, msg):
        self.telemetryTable.setItem(0, 1, QTableWidgetItem(f"{msg.data:.2f}°C"))

    def update_current(self, msg):
        self.telemetryTable.setItem(1, 1, QTableWidgetItem(f"{msg.data:.3f} mA"))

    def update_voltage(self, msg):
        self.telemetryTable.setItem(2, 1, QTableWidgetItem(f"{msg.data:.1f} V"))

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

