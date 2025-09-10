import sys
import math
import time
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                            QHBoxLayout, QLabel, QSlider, QPushButton, QComboBox)
from PyQt5.QtCore import Qt, QTimer
import ctypes
from pathlib import Path

class SensorSimulatorGUI(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("IMU Sensor Simulator")
        
        # Load the host peripherals library
        lib_path = Path(__file__).parent.parent / "build" / "libhostperipherals.dll"
        self.lib = ctypes.CDLL(str(lib_path))
        
        # Set up function prototypes
        self.lib.host_set_sensor_orientation.argtypes = [
            ctypes.c_uint8, ctypes.c_float, ctypes.c_float, ctypes.c_float
        ]
        self.lib.host_set_sensor_acceleration.argtypes = [
            ctypes.c_uint8, ctypes.c_float, ctypes.c_float, ctypes.c_float
        ]
        self.lib.host_set_sensor_angular_velocity.argtypes = [
            ctypes.c_uint8, ctypes.c_float, ctypes.c_float, ctypes.c_float
        ]
        self.lib.host_set_sensor_magnetic_field.argtypes = [
            ctypes.c_uint8, ctypes.c_float, ctypes.c_float, ctypes.c_float
        ]
        
        # Initialize UI
        self.setup_ui()
        
        # Setup timer for animation
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_animation)
        self.timer.start(50)  # 20 Hz update rate
        
        self.animation_angle = 0.0
        self.animation_enabled = False

    def setup_ui(self):
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        layout = QVBoxLayout(central_widget)
        
        # Sensor selection
        sensor_layout = QHBoxLayout()
        sensor_label = QLabel("Sensor:")
        self.sensor_combo = QComboBox()
        self.sensor_combo.addItems(["KX122", "LSM6DS3", "LSM303C"])
        sensor_layout.addWidget(sensor_label)
        sensor_layout.addWidget(self.sensor_combo)
        layout.addLayout(sensor_layout)
        
        # Orientation controls
        for axis, name in [("roll", "Roll"), ("pitch", "Pitch"), ("yaw", "Yaw")]:
            axis_layout = QHBoxLayout()
            label = QLabel(f"{name} (deg):")
            slider = QSlider(Qt.Horizontal)
            slider.setRange(-180, 180)
            slider.setValue(0)
            value_label = QLabel("0")
            
            slider.valueChanged.connect(
                lambda v, l=value_label, a=axis: self.update_orientation(v, l, a)
            )
            
            axis_layout.addWidget(label)
            axis_layout.addWidget(slider)
            axis_layout.addWidget(value_label)
            layout.addLayout(axis_layout)
            
            setattr(self, f"{axis}_slider", slider)
            setattr(self, f"{axis}_label", value_label)
        
        # Animation controls
        anim_layout = QHBoxLayout()
        self.anim_button = QPushButton("Start Animation")
        self.anim_button.setCheckable(True)
        self.anim_button.toggled.connect(self.toggle_animation)
        anim_layout.addWidget(self.anim_button)
        layout.addLayout(anim_layout)
        
        self.setGeometry(100, 100, 600, 400)

    def update_orientation(self, value, label, axis):
        label.setText(str(value))
        sensor_id = self.sensor_combo.currentIndex()
        
        roll = self.roll_slider.value()
        pitch = self.pitch_slider.value()
        yaw = self.yaw_slider.value()
        
        self.lib.host_set_sensor_orientation(sensor_id, 
            ctypes.c_float(roll),
            ctypes.c_float(pitch),
            ctypes.c_float(yaw))

    def toggle_animation(self, enabled):
        self.animation_enabled = enabled
        self.anim_button.setText("Stop Animation" if enabled else "Start Animation")

    def update_animation(self):
        if not self.animation_enabled:
            return
            
        self.animation_angle += 2.0
        if self.animation_angle >= 360.0:
            self.animation_angle = 0.0
        
        # Update sliders with smooth circular motion
        roll = 45.0 * math.sin(math.radians(self.animation_angle))
        pitch = 45.0 * math.cos(math.radians(self.animation_angle))
        yaw = self.animation_angle
        
        self.roll_slider.setValue(int(roll))
        self.pitch_slider.setValue(int(pitch))
        self.yaw_slider.setValue(int(yaw % 360 - 180))

def main():
    app = QApplication(sys.argv)
    window = SensorSimulatorGUI()
    window.show()
    sys.exit(app.exec_())
