#!/usr/bin/env python3
"""
Enhanced drone base station with automatic PID tuning and real telemetry
Matches existing base station structure and adds advanced features
"""

import sys
import struct
import time
import threading
import json
import numpy as np
from collections import deque
from PyQt5.QtWidgets import *
from PyQt5.QtGui import QFont, QColor, QPainter, QPen
from PyQt5.QtCore import Qt, QTimer, pyqtSignal, QThread
import pyqtgraph as pg
import serial

class AutoPIDTuner:
    """Automatic PID tuning using Ziegler-Nichols and optimization methods"""
    
    def __init__(self):
        self.tuning_active = False
        self.step_response_data = deque(maxlen=1000)
        self.current_axis = 'roll'  # roll, pitch, yaw
        self.tuning_phase = 'idle'  # idle, step_test, oscillation_test, optimization
        self.step_start_time = 0
        self.step_magnitude = 0.1  # 0.1 radian step
        self.kp_critical = 0
        self.period_critical = 0
        
    def start_tuning(self, axis):
        """Start automatic PID tuning for specified axis"""
        self.current_axis = axis
        self.tuning_active = True
        self.tuning_phase = 'step_test'
        self.step_response_data.clear()
        self.step_start_time = time.time()
        print(f"Starting PID tuning for {axis} axis")
        
    def add_response_data(self, setpoint, actual, timestamp):
        """Add response data point for analysis"""
        if not self.tuning_active:
            return
            
        self.step_response_data.append({
            'time': timestamp,
            'setpoint': setpoint,
            'actual': actual,
            'error': setpoint - actual
        })
        
    def analyze_step_response(self):
        """Analyze step response to determine initial PID gains"""
        if len(self.step_response_data) < 100:
            return None
            
        data = list(self.step_response_data)
        
        # Calculate step response characteristics
        final_value = np.mean([d['actual'] for d in data[-20:]])  # Last 20 samples
        initial_value = data[0]['actual']
        step_size = final_value - initial_value
        
        if abs(step_size) < 0.01:  # Too small response
            return None
            
        # Find 63% rise time (tau)
        target_63 = initial_value + 0.63 * step_size
        tau_time = None
        for d in data:
            if d['actual'] >= target_63:
                tau_time = d['time'] - data[0]['time']
                break
                
        if tau_time is None or tau_time <= 0:
            return None
            
        # Calculate Ziegler-Nichols gains for PI controller (safer for drones)
        kp = 0.9 / tau_time
        ki = kp / (3.0 * tau_time)
        kd = 0  # Start with no derivative term
        
        return {'kp': kp, 'ki': ki, 'kd': kd, 'tau': tau_time}
    
    def start_oscillation_test(self, initial_kp):
        """Start critical gain oscillation test"""
        self.tuning_phase = 'oscillation_test'
        self.kp_critical = initial_kp * 2.0  # Start with higher gain
        print(f"Starting oscillation test with Kp = {self.kp_critical:.3f}")
        
    def detect_oscillation(self):
        """Detect sustained oscillation and measure period"""
        if len(self.step_response_data) < 200:
            return False
            
        # Analyze last 200 samples for oscillation
        recent_data = list(self.step_response_data)[-200:]
        errors = [d['error'] for d in recent_data]
        
        # Simple oscillation detection using zero crossings
        zero_crossings = []
        for i in range(1, len(errors)):
            if (errors[i] > 0) != (errors[i-1] > 0):
                zero_crossings.append(i)
                
        if len(zero_crossings) >= 4:  # At least 2 complete cycles
            # Calculate period
            periods = []
            for i in range(2, len(zero_crossings)):
                period = (zero_crossings[i] - zero_crossings[i-2]) * 0.001  # Convert to seconds
                periods.append(period)
                
            self.period_critical = np.mean(periods)
            return True
            
        return False
        
    def calculate_final_gains(self):
        """Calculate final PID gains using Ziegler-Nichols ultimate method"""
        if self.kp_critical > 0 and self.period_critical > 0:
            # Ziegler-Nichols ultimate method (conservative for drones)
            kp = 0.45 * self.kp_critical  # Reduced from 0.6 for stability
            ki = kp * 2.0 / self.period_critical
            kd = kp * self.period_critical / 8.0
            
            return {'kp': kp, 'ki': ki, 'kd': kd}
        
        return None
        
    def stop_tuning(self):
        """Stop PID tuning"""
        self.tuning_active = False
        self.tuning_phase = 'idle'
        print(f"PID tuning stopped for {self.current_axis}")

class TelemetryProcessor(QThread):
    """Background thread for processing telemetry data"""
    
    telemetry_received = pyqtSignal(dict)
    
    def __init__(self, serial_port):
        super().__init__()
        self.serial_port = serial_port
        self.running = False
        
    def run(self):
        self.running = True
        buffer = b''
        
        while self.running:
            try:
                if self.serial_port and self.serial_port.in_waiting:
                    data = self.serial_port.read(self.serial_port.in_waiting)
                    buffer += data
                    
                    # Look for telemetry packets (simplified protocol)
                    while len(buffer) >= 64:  # Assuming 64-byte telemetry packets
                        if buffer[0:2] == b'TM':  # Telemetry marker
                            packet = buffer[:64]
                            self.process_telemetry_packet(packet)
                            buffer = buffer[64:]
                        else:
                            buffer = buffer[1:]  # Skip invalid byte
                            
                time.sleep(0.001)  # 1ms sleep
                
            except Exception as e:
                print(f"Telemetry processing error: {e}")
                time.sleep(0.1)
                
    def process_telemetry_packet(self, packet):
        """Process received telemetry packet"""
        try:
            # Unpack telemetry data (adjust format based on your telemetry structure)
            data = struct.unpack('<2s14f4H2I', packet)
            
            telemetry = {
                'timestamp': time.time(),
                'roll': data[2],
                'pitch': data[3], 
                'yaw': data[4],
                'roll_rate': data[5],
                'pitch_rate': data[6],
                'yaw_rate': data[7],
                'throttle': data[8],
                'motor1_pwm': data[18],
                'motor2_pwm': data[19],
                'motor3_pwm': data[20],
                'motor4_pwm': data[21],
                'battery_voltage': data[9],
                'system_status': data[22],
                'flight_mode': data[23]
            }
            
            self.telemetry_received.emit(telemetry)
            
        except struct.error as e:
            print(f"Telemetry unpack error: {e}")
            
    def stop(self):
        self.running = False

class DroneCommander:
    """Enhanced drone communication with telemetry support"""
    
    def __init__(self, port='/dev/ttyUSB0', baudrate=115200):
        self.serial_port = None
        self.port = port
        self.baudrate = baudrate
        self.packet_id = 0
        self.connected = False
        self.telemetry_processor = None
        
    def connect(self):
        """Connect to base station"""
        try:
            self.serial_port = serial.Serial(self.port, self.baudrate, timeout=0.1)
            self.connected = True
            
            # Start telemetry processing thread
            self.telemetry_processor = TelemetryProcessor(self.serial_port)
            self.telemetry_processor.start()
            
            print(f"Connected to {self.port}")
            return True
        except Exception as e:
            print(f"Connection failed: {e}")
            self.connected = False
            return False
    
    def disconnect(self):
        """Disconnect from base station"""
        self.connected = False
        
        if self.telemetry_processor:
            self.telemetry_processor.stop()
            self.telemetry_processor.wait()
            
        if self.serial_port:
            self.serial_port.close()
    
    def _calculate_crc16(self, data):
        """Calculate CRC16 for command packet"""
        crc = 0xFFFF
        for byte in data:
            crc ^= byte
            for _ in range(8):
                if crc & 1:
                    crc = (crc >> 1) ^ 0xA001
                else:
                    crc >>= 1
        return crc & 0xFFFF
    
    def _send_command_packet(self, command_type, data):
        """Send formatted command packet to drone"""
        if not self.connected or not self.serial_port:
            return False
        
        # Pack command packet: header, packet_id, cmd_type, data[4], crc16
        packet_data = struct.pack('<HBBffff', 
                                 0x4350,  # Header "CP"
                                 self.packet_id,
                                 command_type,
                                 data[0] if len(data) > 0 else 0.0,
                                 data[1] if len(data) > 1 else 0.0,
                                 data[2] if len(data) > 2 else 0.0,
                                 data[3] if len(data) > 3 else 0.0)
        
        # Calculate CRC
        crc = self._calculate_crc16(packet_data)
        packet = packet_data + struct.pack('<H', crc)
        
        try:
            self.serial_port.write(packet)
            self.packet_id = (self.packet_id + 1) % 256
            return True
        except Exception as e:
            print(f"Send error: {e}")
            return False
    
    def send_pid_gains(self, axis, kp, ki, kd):
        """Send PID gain update command"""
        axis_map = {'roll': 1.0, 'pitch': 2.0, 'yaw': 3.0}
        axis_value = axis_map.get(axis, 1.0)
        return self._send_command_packet(0x07, [axis_value, kp, ki, kd])  # PID update command
    
    def send_heartbeat(self):
        return self._send_command_packet(0x01, [])
    
    def send_arm_disarm(self, arm):
        return self._send_command_packet(0x02, [1.0 if arm else 0.0])
    
    def send_flight_mode(self, mode):
        return self._send_command_packet(0x03, [float(mode)])
    
    def send_manual_control(self, throttle, roll, pitch, yaw):
        return self._send_command_packet(0x04, [throttle, roll, pitch, yaw])
    
    def send_calibrate(self, calib_type):
        return self._send_command_packet(0x05, [float(calib_type)])

class EnhancedDroneUI(QMainWindow):
    """Enhanced drone UI with auto PID tuning and real telemetry"""
    
    def __init__(self):
        super().__init__()
        self.drone = DroneCommander()
        self.pid_tuner = AutoPIDTuner()
        self.armed = False
        self.flight_mode = 0
        
        # Telemetry data storage
        self.telemetry_history = {
            'time': deque(maxlen=1000),
            'roll': deque(maxlen=1000),
            'pitch': deque(maxlen=1000),
            'yaw': deque(maxlen=1000),
            'roll_rate': deque(maxlen=1000),
            'pitch_rate': deque(maxlen=1000),
            'yaw_rate': deque(maxlen=1000),
            'motor1': deque(maxlen=1000),
            'motor2': deque(maxlen=1000),
            'motor3': deque(maxlen=1000),
            'motor4': deque(maxlen=1000),
        }
        
        self.current_gains = {
            'roll': {'kp': 4.0, 'ki': 0.1, 'kd': 0.2},
            'pitch': {'kp': 4.0, 'ki': 0.1, 'kd': 0.2},
            'yaw': {'kp': 2.0, 'ki': 0.05, 'kd': 0.0}
        }
        
        self.setup_ui()
        self.setup_timers()
        
        # Connect telemetry signal
        if hasattr(self.drone, 'telemetry_processor'):
            self.drone.telemetry_processor.telemetry_received.connect(self.process_telemetry)
        
        # Try to connect automatically
        self.connect_to_drone()
    
    def setup_ui(self):
        """Setup enhanced user interface"""
        self.setWindowTitle("Advanced Drone Control with Auto-PID Tuning")
        self.setStyleSheet("background-color: #181c24; color: #f8f8f2;")
        self.setMinimumSize(1200, 800)
        
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        layout = QHBoxLayout(central_widget)
        
        # Left panel - Controls
        left_panel = self.create_control_panel()
        layout.addWidget(left_panel, 1)
        
        # Right panel - Telemetry and PID tuning
        right_panel = self.create_telemetry_panel()
        layout.addWidget(right_panel, 2)
    
    def create_control_panel(self):
        """Create control panel"""
        panel = QWidget()
        panel.setMaximumWidth(400)
        layout = QVBoxLayout(panel)
        
        # Title
        title = QLabel("🚁 Drone Control")
        title.setFont(QFont("Arial", 18, QFont.Bold))
        title.setAlignment(Qt.AlignCenter)
        title.setStyleSheet("color: #50fa7b; margin: 10px;")
        layout.addWidget(title)
        
        # Connection status
        self.connection_status = QLabel("Status: Disconnected")
        self.connection_status.setStyleSheet("color: #ff5555; margin: 5px;")
        layout.addWidget(self.connection_status)
        
        # Arm/Disarm
        arm_group = QGroupBox("Arm/Disarm Control")
        arm_layout = QVBoxLayout(arm_group)
        
        self.arm_button = QPushButton("ARM DRONE")
        self.arm_button.setStyleSheet("""
            QPushButton { 
                background: #ff5555; color: white; border-radius: 8px; 
                padding: 12px; min-height: 40px; font-weight: bold;
            }
            QPushButton:hover { background: #ff7777; }
        """)
        self.arm_button.clicked.connect(self.toggle_arm)
        arm_layout.addWidget(self.arm_button)
        layout.addWidget(arm_group)
        
        # Flight modes
        mode_group = QGroupBox("Flight Mode")
        mode_layout = QVBoxLayout(mode_group)
        
        self.mode_buttons = QButtonGroup()
        modes = ["Disarmed", "Stabilize", "Altitude Hold", "Autonomous"]
        for i, mode in enumerate(modes):
            btn = QRadioButton(mode)
            if i == 0:
                btn.setChecked(True)
            btn.toggled.connect(lambda checked, m=i: self.set_flight_mode(m) if checked else None)
            self.mode_buttons.addButton(btn, i)
            mode_layout.addWidget(btn)
        layout.addWidget(mode_group)
        
        # Manual control
        control_group = QGroupBox("Manual Control")
        control_layout = QGridLayout(control_group)
        
        # Throttle
        control_layout.addWidget(QLabel("Throttle:"), 0, 0)
        self.throttle_slider = QSlider(Qt.Vertical)
        self.throttle_slider.setRange(0, 100)
        self.throttle_slider.setValue(0)
        control_layout.addWidget(self.throttle_slider, 1, 0)
        self.throttle_label = QLabel("0%")
        control_layout.addWidget(self.throttle_label, 2, 0)
        
        # Yaw
        control_layout.addWidget(QLabel("Yaw:"), 0, 1)
        self.yaw_slider = QSlider(Qt.Horizontal)
        self.yaw_slider.setRange(-100, 100)
        self.yaw_slider.setValue(0)
        control_layout.addWidget(self.yaw_slider, 1, 1)
        self.yaw_label = QLabel("0%")
        control_layout.addWidget(self.yaw_label, 2, 1)
        
        layout.addWidget(control_group)
        
        # Emergency stop
        emergency_btn = QPushButton("🚨 EMERGENCY STOP 🚨")
        emergency_btn.setStyleSheet("""
            QPushButton { 
                background: #ff0000; color: white; border-radius: 8px; 
                padding: 15px; min-height: 50px; font-weight: bold;
            }
        """)
        emergency_btn.clicked.connect(self.emergency_stop)
        layout.addWidget(emergency_btn)
        
        return panel
    
    def create_telemetry_panel(self):
        """Create telemetry and PID tuning panel"""
        panel = QWidget()
        layout = QVBoxLayout(panel)
        
        # Create tabs
        tabs = QTabWidget()
        
        # Telemetry tab
        telemetry_tab = self.create_telemetry_tab()
        tabs.addTab(telemetry_tab, "Real-Time Telemetry")
        
        # PID Tuning tab
        pid_tab = self.create_pid_tuning_tab()
        tabs.addTab(pid_tab, "Auto PID Tuning")
        
        layout.addWidget(tabs)
        return panel
    
    def create_telemetry_tab(self):
        """Create real-time telemetry display"""
        widget = QWidget()
        layout = QVBoxLayout(widget)
        
        # Attitude plot
        self.attitude_plot = pg.PlotWidget(title="Attitude (degrees)")
        self.attitude_plot.setLabel('left', 'Angle', 'degrees')
        self.attitude_plot.setLabel('bottom', 'Time', 's')
        self.attitude_plot.addLegend()
        
        self.roll_curve = self.attitude_plot.plot(pen='r', name='Roll')
        self.pitch_curve = self.attitude_plot.plot(pen='g', name='Pitch')  
        self.yaw_curve = self.attitude_plot.plot(pen='b', name='Yaw')
        
        layout.addWidget(self.attitude_plot)
        
        # Motor outputs plot
        self.motor_plot = pg.PlotWidget(title="Motor Outputs (PWM μs)")
        self.motor_plot.setLabel('left', 'PWM', 'μs')
        self.motor_plot.setLabel('bottom', 'Time', 's')
        self.motor_plot.addLegend()
        
        self.motor1_curve = self.motor_plot.plot(pen='r', name='Motor 1')
        self.motor2_curve = self.motor_plot.plot(pen='g', name='Motor 2')
        self.motor3_curve = self.motor_plot.plot(pen='b', name='Motor 3')
        self.motor4_curve = self.motor_plot.plot(pen='y', name='Motor 4')
        
        layout.addWidget(self.motor_plot)
        
        # Status display
        status_layout = QHBoxLayout()
        
        # Current values
        current_group = QGroupBox("Current Values")
        current_layout = QGridLayout(current_group)
        
        self.roll_value = QLabel("Roll: 0.0°")
        self.pitch_value = QLabel("Pitch: 0.0°")
        self.yaw_value = QLabel("Yaw: 0.0°")
        self.battery_value = QLabel("Battery: --V")
        
        current_layout.addWidget(self.roll_value, 0, 0)
        current_layout.addWidget(self.pitch_value, 0, 1)
        current_layout.addWidget(self.yaw_value, 1, 0)
        current_layout.addWidget(self.battery_value, 1, 1)
        
        status_layout.addWidget(current_group)
        layout.addLayout(status_layout)
        
        return widget
    
    def create_pid_tuning_tab(self):
        """Create automatic PID tuning interface"""
        widget = QWidget()
        layout = QVBoxLayout(widget)
        
        # PID Tuning Control
        tuning_group = QGroupBox("Automatic PID Tuning")
        tuning_layout = QVBoxLayout(tuning_group)
        
        # Axis selection
        axis_layout = QHBoxLayout()
        axis_layout.addWidget(QLabel("Tune Axis:"))
        
        self.axis_combo = QComboBox()
        self.axis_combo.addItems(["Roll", "Pitch", "Yaw"])
        axis_layout.addWidget(self.axis_combo)
        
        self.start_tuning_btn = QPushButton("Start Auto-Tuning")
        self.start_tuning_btn.clicked.connect(self.start_auto_tuning)
        axis_layout.addWidget(self.start_tuning_btn)
        
        self.stop_tuning_btn = QPushButton("Stop Tuning")
        self.stop_tuning_btn.clicked.connect(self.stop_auto_tuning)
        self.stop_tuning_btn.setEnabled(False)
        axis_layout.addWidget(self.stop_tuning_btn)
        
        tuning_layout.addLayout(axis_layout)
        
        # Tuning status
        self.tuning_status = QLabel("Status: Ready")
        tuning_layout.addWidget(self.tuning_status)
        
        # Progress bar
        self.tuning_progress = QProgressBar()
        tuning_layout.addWidget(self.tuning_progress)
        
        layout.addWidget(tuning_group)
        
        # Current PID Gains
        gains_group = QGroupBox("Current PID Gains")
        gains_layout = QGridLayout(gains_group)
        
        # Create gain displays and editors
        self.gain_displays = {}
        self.gain_editors = {}
        
        axes = ['roll', 'pitch', 'yaw']
        gains = ['kp', 'ki', 'kd']
        
        gains_layout.addWidget(QLabel("Axis"), 0, 0)
        gains_layout.addWidget(QLabel("Kp"), 0, 1)
        gains_layout.addWidget(QLabel("Ki"), 0, 2)
        gains_layout.addWidget(QLabel("Kd"), 0, 3)
        gains_layout.addWidget(QLabel("Action"), 0, 4)
        
        for i, axis in enumerate(axes):
            gains_layout.addWidget(QLabel(axis.title()), i+1, 0)
            
            self.gain_editors[axis] = {}
            for j, gain in enumerate(gains):
                editor = QDoubleSpinBox()
                editor.setRange(0.0, 100.0)
                editor.setSingleStep(0.1)
                editor.setDecimals(3)
                editor.setValue(self.current_gains[axis][gain])
                editor.valueChanged.connect(lambda val, a=axis, g=gain: self.update_gain(a, g, val))
                
                self.gain_editors[axis][gain] = editor
                gains_layout.addWidget(editor, i+1, j+1)
            
            # Send button
            send_btn = QPushButton("Send")
            send_btn.clicked.connect(lambda checked, a=axis: self.send_gains(a))
            gains_layout.addWidget(send_btn, i+1, 4)
        
        layout.addWidget(gains_group)
        
        # Tuning results
        results_group = QGroupBox("Auto-Tuning Results")
        results_layout = QVBoxLayout(results_group)
        
        self.tuning_results = QTextEdit()
        self.tuning_results.setMaximumHeight(150)
        self.tuning_results.setReadOnly(True)
        results_layout.addWidget(self.tuning_results)
        
        layout.addWidget(results_group)
        
        return widget
    
    def setup_timers(self):
        """Setup periodic timers"""
        # Heartbeat timer
        self.heartbeat_timer = QTimer()
        self.heartbeat_timer.timeout.connect(self.send_heartbeat)
        self.heartbeat_timer.start(500)
        
        # Control update timer
        self.control_timer = QTimer()
        self.control_timer.timeout.connect(self.send_control_update)
        self.control_timer.start(100)
        
        # Plot update timer
        self.plot_timer = QTimer()
        self.plot_timer.timeout.connect(self.update_plots)
        self.plot_timer.start(50)  # 20Hz plot updates
    
    def connect_to_drone(self):
        """Connect to drone"""
        if self.drone.connect():
            self.connection_status.setText("Status: Connected")
            self.connection_status.setStyleSheet("color: #50fa7b;")
            
            # Connect telemetry signal
            if self.drone.telemetry_processor:
                self.drone.telemetry_processor.telemetry_received.connect(self.process_telemetry)
        else:
            self.connection_status.setText("Status: Failed")
            self.connection_status.setStyleSheet("color: #ff5555;")
    
    def process_telemetry(self, data):
        """Process incoming telemetry data"""
        current_time = time.time()
        
        # Store telemetry data
        self.telemetry_history['time'].append(current_time)
        self.telemetry_history['roll'].append(math.degrees(data['roll']))
        self.telemetry_history['pitch'].append(math.degrees(data['pitch']))
        self.telemetry_history['yaw'].append(math.degrees(data['yaw']))
        self.telemetry_history['roll_rate'].append(data['roll_rate'])
        self.telemetry_history['pitch_rate'].append(data['pitch_rate'])
        self.telemetry_history['yaw_rate'].append(data['yaw_rate'])
        self.telemetry_history['motor1'].append(data['motor1_pwm'])
        self.telemetry_history['motor2'].append(data['motor2_pwm'])
        self.telemetry_history['motor3'].append(data['motor3_pwm'])
        self.telemetry_history['motor4'].append(data['motor4_pwm'])
        
        # Update current value displays
        self.roll_value.setText(f"Roll: {math.degrees(data['roll']):.1f}°")
        self.pitch_value.setText(f"Pitch: {math.degrees(data['pitch']):.1f}°")
        self.yaw_value.setText(f"Yaw: {math.degrees(data['yaw']):.1f}°")
        self.battery_value.setText(f"Battery: {data['battery_voltage']:.1f}V")
        
        # Feed data to PID tuner if active
        if self.pid_tuner.tuning_active:
            axis_map = {'roll': data['roll'], 'pitch': data['pitch'], 'yaw': data['yaw']}
            if self.pid_tuner.current_axis in axis_map:
                # For tuning, we need setpoint - this would come from test commands
                setpoint = 0.1 if self.pid_tuner.tuning_phase == 'step_test' else 0.0
                actual = axis_map[self.pid_tuner.current_axis]
                self.pid_tuner.add_response_data(setpoint, actual, current_time)
    
    def update_plots(self):
        """Update telemetry plots"""
        if len(self.telemetry_history['time']) < 2:
            return
        
        # Convert time to relative seconds
        times = list(self.telemetry_history['time'])
        if times:
            rel_times = [t - times[0] for t in times]
            
            # Update attitude plot
            self.roll_curve.setData(rel_times, list(self.telemetry_history['roll']))
            self.pitch_curve.setData(rel_times, list(self.telemetry_history['pitch']))
            self.yaw_curve.setData(rel_times, list(self.telemetry_history['yaw']))
            
            # Update motor plot
            self.motor1_curve.setData(rel_times, list(self.telemetry_history['motor1']))
            self.motor2_curve.setData(rel_times, list(self.telemetry_history['motor2']))
            self.motor3_curve.setData(rel_times, list(self.telemetry_history['motor3']))
            self.motor4_curve.setData(rel_times, list(self.telemetry_history['motor4']))
    
    def start_auto_tuning(self):
        """Start automatic PID tuning"""
        if not self.drone.connected:
            QMessageBox.warning(self, "Not Connected", "Please connect to drone first")
            return
        
        if not self.armed:
            QMessageBox.warning(self, "Not Armed", "Please arm drone before tuning")
            return
        
        axis = self.axis_combo.currentText().lower()
        self.pid_tuner.start_tuning(axis)
        
        self.start_tuning_btn.setEnabled(False)
        self.stop_tuning_btn.setEnabled(True)
        self.tuning_progress.setValue(0)
        self.tuning_status.setText(f"Tuning {axis} axis - Step test phase")
        
        # Send step command to drone
        # This would send a small step input to test response
        
    def stop_auto_tuning(self):
        """Stop automatic PID tuning"""
        # Analyze results if tuning was active
        if self.pid_tuner.tuning_active:
            if self.pid_tuner.tuning_phase == 'step_test':
                results = self.pid_tuner.analyze_step_response()
                if results:
                    axis = self.pid_tuner.current_axis
                    self.tuning_results.append(f"\nStep Response Analysis for {axis}:")
                    self.tuning_results.append(f"Suggested gains - Kp: {results['kp']:.3f}, Ki: {results['ki']:.3f}, Kd: {results['kd']:.3f}")
                    
                    # Auto-update gain values
                    self.gain_editors[axis]['kp'].setValue(results['kp'])
                    self.gain_editors[axis]['ki'].setValue(results['ki'])
                    self.gain_editors[axis]['kd'].setValue(results['kd'])
        
        self.pid_tuner.stop_tuning()
        
        self.start_tuning_btn.setEnabled(True)
        self.stop_tuning_btn.setEnabled(False)
        self.tuning_progress.setValue(0)
        self.tuning_status.setText("Status: Ready")
    
    def update_gain(self, axis, gain_type, value):
        """Update gain value"""
        self.current_gains[axis][gain_type] = value
    
    def send_gains(self, axis):
        """Send PID gains to drone"""
        gains = self.current_gains[axis]
        if self.drone.send_pid_gains(axis, gains['kp'], gains['ki'], gains['kd']):
            self.tuning_results.append(f"Sent {axis} gains: Kp={gains['kp']:.3f}, Ki={gains['ki']:.3f}, Kd={gains['kd']:.3f}")
        else:
            self.tuning_results.append(f"Failed to send {axis} gains")
    
    def toggle_arm(self):
        """Toggle arm state"""
        self.armed = not self.armed
        if self.armed:
            self.arm_button.setText("DISARM")
            self.arm_button.setStyleSheet("QPushButton { background: #50fa7b; color: black; }")
            self.flight_mode = 1
            self.mode_buttons.button(1).setChecked(True)
        else:
            self.arm_button.setText("ARM DRONE")
            self.arm_button.setStyleSheet("QPushButton { background: #ff5555; color: white; }")
            self.flight_mode = 0
            self.mode_buttons.button(0).setChecked(True)
        
        self.drone.send_arm_disarm(self.armed)
        self.drone.send_flight_mode(self.flight_mode)
    
    def set_flight_mode(self, mode):
        """Set flight mode"""
        if not self.armed and mode != 0:
            self.mode_buttons.button(0).setChecked(True)
            return
        
        self.flight_mode = mode
        self.drone.send_flight_mode(mode)
    
    def send_control_update(self):
        """Send control updates"""
        if self.armed and self.drone.connected:
            throttle = self.throttle_slider.value() / 100.0
            yaw = self.yaw_slider.value() / 100.0
            
            self.throttle_label.setText(f"{int(throttle * 100)}%")
            self.yaw_label.setText(f"{int(yaw * 100)}%")
            
            self.drone.send_manual_control(throttle, 0.0, 0.0, yaw)
    
    def send_heartbeat(self):
        """Send heartbeat"""
        if self.drone.connected:
            self.drone.send_heartbeat()
    
    def emergency_stop(self):
        """Emergency stop"""
        self.armed = False
        self.flight_mode = 0
        self.arm_button.setText("ARM DRONE")
        self.arm_button.setStyleSheet("QPushButton { background: #ff5555; color: white; }")
        self.mode_buttons.button(0).setChecked(True)
        
        self.drone.send_arm_disarm(False)
        self.drone.send_flight_mode(0)
        
        QMessageBox.warning(self, "Emergency Stop", "Drone emergency stopped!")

if __name__ == '__main__':
    import math
    
    app = QApplication(sys.argv)
    window = EnhancedDroneUI()
    window.show()
    sys.exit(app.exec_())