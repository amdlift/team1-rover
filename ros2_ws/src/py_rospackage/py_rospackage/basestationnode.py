# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import sys
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32
from PyQt6.QtWidgets import (
    QApplication, QWidget, QLabel, QVBoxLayout, QHBoxLayout,
    QPushButton, QGridLayout
)
from PyQt6.QtCore import QTimer, pyqtSignal
from threading import Thread, Lock
import pyqtgraph as pg
import keyboard
import serial

# Try to import and init DualSense
ds = None
try:
    from pydualsense import pydualsense
    ds = pydualsense()
    ds.init()
except Exception as e:
    print(f"DualSense init failed: {e}")

class RobotState:
    def __init__(self):
        self._lock = Lock()
        self.temperature = 0.0  # °C
        self.pressure = 0.0     # hPa
        self.acceleration = 0.0 # ft/s²
        self.altitude = 0.0     # ft
        self.wrist_position = 0.0
        self.lower_elbow_position = 0.0
        self.upper_elbow_position = 0.0
        self.turret_position = 0.0
        self.claw_position = 0.0

    def update_sensor(self, sensor_type, value):
        with self._lock:
            setattr(self, sensor_type, value)

    def update_motor(self, motor_type, angle_change):
        with self._lock:
            current = getattr(self, motor_type)
            setattr(self, motor_type, current + angle_change)

    def get_sensor(self, sensor_type):
        with self._lock:
            return getattr(self, sensor_type)

    def get_motor(self, motor_type):
        with self._lock:
            return getattr(self, motor_type)

class BaseStationNode(Node):
    def __init__(self, state, update_signal):
        super().__init__('base_station')
        self.state = state
        self.update_signal = update_signal

        # Initialize publisher
        self.publisher_ = self.create_publisher(String, 'command', 10)

        # Initialize serial connection
        self.serial = None
        try:
            self.serial = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)
            self.get_logger().info('Serial connection established')
        except serial.SerialException as e:
            self.get_logger().error(f"Serial connection failed: {e}")

        # Initialize subscriptions
        self.sub_temperature = self.create_subscription(
            Float32, 'temperature', self.temp_callback, 10)
        self.sub_pressure = self.create_subscription(
            Float32, 'pressure', self.press_callback, 10)
        self.sub_altitude = self.create_subscription(
            Float32, 'altitude', self.alt_callback, 10)
        self.sub_acceleration = self.create_subscription(
            Float32, 'acceleration', self.acc_callback, 10)

        self.get_logger().info('Base Station Node initialized')

    def temp_callback(self, msg):
        self.state.update_sensor('temperature', msg.data)
        self.update_signal.emit()
        self.get_logger().info(f'Received temperature: {msg.data}')

    def press_callback(self, msg):
        self.state.update_sensor('pressure', msg.data)
        self.update_signal.emit()
        self.get_logger().info(f'Received pressure: {msg.data}')

    def alt_callback(self, msg):
        self.state.update_sensor('altitude', msg.data)
        self.update_signal.emit()
        self.get_logger().info(f'Received altitude: {msg.data}')

    def acc_callback(self, msg):
        self.state.update_sensor('acceleration', msg.data)
        self.update_signal.emit()
        self.get_logger().info(f'Received acceleration: {msg.data}')

    def update_motor(self, wrist=0.0, claw=0.0, upper_elbow=0.0, lower_elbow=0.0, turret=0.0):
        # Update state
        if claw != 0.0:
            self.state.update_motor('claw_position', claw)
        if wrist != 0.0:
            self.state.update_motor('wrist_position', wrist)
        if upper_elbow != 0.0:
            self.state.update_motor('upper_elbow_position', upper_elbow)
        if lower_elbow != 0.0:
            self.state.update_motor('lower_elbow_position', lower_elbow)
        if turret != 0.0:
            self.state.update_motor('turret_position', turret)

        # Send serial command
        if self.serial and self.serial.is_open:
            command = f"C{claw}:W{wrist}:U{upper_elbow}:L{lower_elbow}:T{turret}\n"
            try:
                self.serial.write(command.encode())
                self.get_logger().info(f'Sent serial command: {command.strip()}')
            except serial.SerialException as e:
                self.get_logger().error(f"Serial write failed: {e}")

        # Publish ROS message
        msg = String()
        msg.data = f"C{claw}:W{wrist}:U{upper_elbow}:L{lower_elbow}:T{turret}"
        self.publisher_.publish(msg)
        self.get_logger().info(f'Published command: {msg.data}')

    def destroy_node(self):
        if self.serial and self.serial.is_open:
            self.serial.close()
            self.get_logger().info('Serial connection closed')
        super().destroy_node()

class Ros2Thread(Thread):
    def __init__(self, state, update_signal):
        super().__init__()
        self.state = state
        self.update_signal = update_signal

    def run(self):
        node = BaseStationNode(self.state, self.update_signal)
        rclpy.spin(node)
        node.destroy_node()

class ControlWindow(QWidget):
    def __init__(self, node):
        super().__init__()
        self.node = node
        self.setWindowTitle("Robotic Arm Control (No Controller - Buttons/Keyboard Only)" if not ds else "Robotic Arm Control")
        self.setGeometry(100, 100, 400, 300)

        # Buttons
        btn_style = "min-width: 120px; min-height: 30px;"
        self.open_claw = QPushButton("Open Claw")
        self.close_claw = QPushButton("Close Claw")
        self.wrist_up = QPushButton("Wrist Right")
        self.wrist_down = QPushButton("Wrist Left")
        self.elbow1_up = QPushButton("Lower Elbow Up")
        self.elbow1_down = QPushButton("Lower Elbow Down")
        self.elbow2_up = QPushButton("Upper Elbow Up")
        self.elbow2_down = QPushButton("Upper Elbow Down")
        self.turret_left = QPushButton("Turret Left")
        self.turret_right = QPushButton("Turret Right")

        for b in [self.open_claw, self.close_claw, self.wrist_up, self.wrist_down,
                  self.elbow1_up, self.elbow1_down, self.elbow2_up, self.elbow2_down,
                  self.turret_left, self.turret_right]:
            b.setStyleSheet(btn_style)

        # Button connections
        self.open_claw.clicked.connect(lambda: self.node.update_motor(claw=10.0))
        self.close_claw.clicked.connect(lambda: self.node.update_motor(claw=-10.0))
        self.wrist_up.clicked.connect(lambda: self.node.update_motor(wrist=10.0))
        self.wrist_down.clicked.connect(lambda: self.node.update_motor(wrist=-10.0))
        self.elbow1_up.clicked.connect(lambda: self.node.update_motor(lower_elbow=10.0))
        self.elbow1_down.clicked.connect(lambda: self.node.update_motor(lower_elbow=-10.0))
        self.elbow2_up.clicked.connect(lambda: self.node.update_motor(upper_elbow=10.0))
        self.elbow2_down.clicked.connect(lambda: self.node.update_motor(upper_elbow=-10.0))
        self.turret_left.clicked.connect(lambda: self.node.update_motor(turret=-10.0))
        self.turret_right.clicked.connect(lambda: self.node.update_motor(turret=10.0))

        # Layout
        main = QVBoxLayout()
        main.setContentsMargins(10, 10, 10, 10)
        main.setSpacing(10)

        claw_box = QHBoxLayout()
        claw_box.addWidget(self.open_claw)
        claw_box.addWidget(self.close_claw)
        main.addWidget(QLabel("<b>Claw Controls</b>"))
        main.addLayout(claw_box)

        wrist_box = QHBoxLayout()
        wrist_box.addWidget(self.wrist_up)
        wrist_box.addWidget(self.wrist_down)
        main.addWidget(QLabel("<b>Wrist Controls</b>"))
        main.addLayout(wrist_box)

        elbow_box = QHBoxLayout()
        elbow1_box = QVBoxLayout()
        elbow1_box.addWidget(self.elbow1_up)
        elbow1_box.addWidget(self.elbow1_down)
        elbow2_box = QVBoxLayout()
        elbow2_box.addWidget(self.elbow2_up)
        elbow2_box.addWidget(self.elbow2_down)
        elbow_box.addLayout(elbow1_box)
        elbow_box.addLayout(elbow2_box)
        main.addWidget(QLabel("<b>Elbow Controls</b>"))
        main.addLayout(elbow_box)

        turret_box = QHBoxLayout()
        turret_box.addWidget(self.turret_left)
        turret_box.addWidget(self.turret_right)
        main.addWidget(QLabel("<b>Turret Controls</b>"))
        main.addLayout(turret_box)

        self.setLayout(main)

        # Input polling (keyboard and controller)
        def poll_inputs():
            # Keyboard inputs
            if keyboard.is_pressed('a'):
                self.node.update_motor(wrist=10.0)
            elif keyboard.is_pressed('d'):
                self.node.update_motor(wrist=-10.0)
            elif keyboard.is_pressed('w'):
                self.node.update_motor(upper_elbow=10.0)
            elif keyboard.is_pressed('s'):
                self.node.update_motor(upper_elbow=-10.0)
            elif keyboard.is_pressed('up'):
                self.node.update_motor(lower_elbow=10.0)
            elif keyboard.is_pressed('down'):
                self.node.update_motor(lower_elbow=-10.0)
            elif keyboard.is_pressed('left'):
                self.node.update_motor(turret=-10.0)
            elif keyboard.is_pressed('right'):
                self.node.update_motor(turret=10.0)
            elif keyboard.is_pressed('q'):
                self.node.update_motor(claw=10.0)
            elif keyboard.is_pressed('e'):
                self.node.update_motor(claw=-10.0)

            # Controller inputs
            if ds:
                state = ds.sendReport()
                if not state:
                    return
                if state.DPadUp:
                    self.node.update_motor(wrist=10.0)
                if state.DPadDown:
                    self.node.update_motor(wrist=-10.0)
                if state.DPadLeft:
                    self.node.update_motor(turret=-10.0)
                if state.DPadRight:
                    self.node.update_motor(turret=10.0)
                if state.L2 > 128:
                    self.node.update_motor(claw=10.0)
                if state.R2 > 128:
                    self.node.update_motor(claw=-10.0)
                if state.LeftStickY > 0.5:
                    self.node.update_motor(lower_elbow=10.0)
                if state.LeftStickY < -0.5:
                    self.node.update_motor(lower_elbow=-10.0)
                if state.RightStickY > 0.5:
                    self.node.update_motor(upper_elbow=10.0)
                if state.RightStickY < -0.5:
                    self.node.update_motor(upper_elbow=-10.0)
                if any([state.DPadUp, state.DPadDown, state.DPadLeft, state.DPadRight,
                        state.L2 > 128, state.R2 > 128, abs(state.LeftStickY) > 0.5, abs(state.RightStickY) > 0.5]):
                    self.trigger_feedback()

        input_timer = QTimer(self)
        input_timer.timeout.connect(poll_inputs)
        input_timer.start(50)  # Poll at 20 Hz

    def trigger_feedback(self):
        if ds:
            try:
                ds.trigger.set_mode('Vibration')
                ds.trigger.set_effect(0.5, 0.1)  # Light rumble
            except Exception as e:
                print(f"Controller feedback failed: {e}")

class PlotWindow(QWidget):
    update_signal = pyqtSignal()

    def __init__(self, state):
        super().__init__()
        self.state = state
        self.setWindowTitle("Graphical Information")
        self.setGeometry(520, 100, 800, 600)

        # Widgets
        self.temp_label = QLabel("Temperature: –")
        self.temp_graph = pg.PlotWidget(title="Temperature")
        self.temp_graph.setLabel('left', 'Temperature (°C)')
        self.temp_graph.setLabel('bottom', 'Time (s)')
        self.temp_curve = self.temp_graph.plot(pen='b', symbol='o', symbolPen='b', symbolBrush='b')

        self.pres_label = QLabel("Pressure: –")
        self.pres_graph = pg.PlotWidget(title="Pressure")
        self.pres_graph.setLabel('left', 'Pressure (hPa)')
        self.pres_graph.setLabel('bottom', 'Time (s)')
        self.pres_curve = self.pres_graph.plot(pen='g', symbol='o', symbolPen='g', symbolBrush='g')

        self.acc_label = QLabel("Acceleration: –")
        self.acc_graph = pg.PlotWidget(title="Acceleration")
        self.acc_graph.setLabel('left', 'Acceleration (ft/s²)')
        self.acc_graph.setLabel('bottom', 'Time (s)')
        self.acc_curve = self.acc_graph.plot(pen='r', symbol='o', symbolPen='r', symbolBrush='r')

        self.alt_label = QLabel("Altitude: –")
        self.alt_graph = pg.PlotWidget(title="Altitude")
        self.alt_graph.setLabel('left', 'Altitude (ft)')
        self.alt_graph.setLabel('bottom', 'Time (s)')
        self.alt_curve = self.alt_graph.plot(pen='y', symbol='o', symbolPen='y', symbolBrush='y')

        # Data buffers
        self.max_points = 60
        self.t_data = []
        self.temp_data, self.pres_data, self.acc_data, self.alt_data = [], [], [], []
        self.start_time = time.time()

        # Grid layout
        layout = QGridLayout()
        layout.addWidget(self.temp_label, 0, 0)
        layout.addWidget(self.temp_graph, 1, 0)
        layout.addWidget(self.pres_label, 0, 1)
        layout.addWidget(self.pres_graph, 1, 1)
        layout.addWidget(self.acc_label, 2, 0)
        layout.addWidget(self.acc_graph, 3, 0)
        layout.addWidget(self.alt_label, 2, 1)
        layout.addWidget(self.alt_graph, 3, 1)
        self.setLayout(layout)

        # Connect update signal
        self.update_signal.connect(self.update)

        # Timer for periodic updates
        timer = QTimer(self)
        timer.timeout.connect(self.update)
        timer.start(1000)  # Update every second

    def update(self):
        now = time.time() - self.start_time

        # Update labels
        self.temp_label.setText(f"Temperature: {self.state.get_sensor('temperature'):.2f} °C")
        self.pres_label.setText(f"Pressure: {self.state.get_sensor('pressure'):.2f} hPa")
        self.acc_label.setText(f"Acceleration: {self.state.get_sensor('acceleration'):.2f} ft/s²")
        self.alt_label.setText(f"Altitude: {self.state.get_sensor('altitude'):.2f} ft")

        # Append data
        self.t_data.append(now)
        self.temp_data.append(self.state.get_sensor('temperature'))
        self.pres_data.append(self.state.get_sensor('pressure'))
        self.acc_data.append(self.state.get_sensor('acceleration'))
        self.alt_data.append(self.state.get_sensor('altitude'))

        # Trim data if necessary
        if len(self.t_data) > self.max_points:
            self.t_data = self.t_data[-self_max_points:]
            self.temp_data = self.temp_data[-self.max_points:]
            self.pres_data = self.pres_data[-self.max_points:]
            self.acc_data = self.acc_data[-self.max_points:]
            self.alt_data = self.alt_data[-self.max_points:]

        # Update curves
        self.temp_curve.setData(self.t_data, self.temp_data)
        self.pres_curve.setData(self.t_data, self.pres_data)
        self.acc_curve.setData(self.t_data, self.acc_data)
        self.alt_curve.setData(self.t_data, self.alt_data)

def main(args=None):
    rclpy.init(args=args)
    app = QApplication(sys.argv)
    state = RobotState()
    plot_win = PlotWindow(state)
    node = BaseStationNode(state, plot_win.update_signal)
    ctrl_win = ControlWindow(node)
    ctrl_win.show()
    plot_win.show()

    # Start ROS 2 thread
    ros_thread = Ros2Thread(state, plot_win.update_signal)
    ros_thread.start()

    # Cleanup
    def cleanup():
        if ds:
            ds.close()
        node.destroy_node()
        rclpy.shutdown()

    app.aboutToQuit.connect(cleanup)
    sys.exit(app.exec())

if __name__ == "__main__":
    main()