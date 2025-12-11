#!/usr/bin/env python3
"""
Simple PyQt5 GUI to run the VM-based simulation and start/stop simple sensor simulators.

Features:
- Select an ARM ELF or test binary
- Configure emulator path (default: qemu-arm)
- Start/stop simple TCP-based sensor simulators (IMU/GPS/RF)
- Launch the VM runner (run_vm_tests.exe or qemu-arm) and capture its output
- Highlight TEST: PASS/FAIL lines

This is a minimal tool for interactive runs and debugging on host.
"""
import sys
import os
import subprocess
import threading
import time
from pathlib import Path

from PyQt5 import QtWidgets, QtGui, QtCore

from sensor_simulator import SensorManager


class ProcessThread(QtCore.QThread):
    output = QtCore.pyqtSignal(str)
    finished_signal = QtCore.pyqtSignal(int)

    def __init__(self, cmd):
        super().__init__()
        self.cmd = cmd
        self._proc = None

    def run(self):
        try:
            self._proc = subprocess.Popen(self.cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        except Exception as e:
            self.output.emit(f"ERROR: failed to start process: {e}\n")
            self.finished_signal.emit(-1)
            return

        for line in self._proc.stdout:
            self.output.emit(line.rstrip('\n'))

        rc = self._proc.wait()
        self.finished_signal.emit(rc)

    def stop(self):
        if self._proc and self._proc.poll() is None:
            try:
                self._proc.terminate()
            except Exception:
                pass


class OrientationPanel(QtWidgets.QGroupBox):
    def __init__(self, parent=None):
        super().__init__("Sensor Orientation", parent)
        layout = QtWidgets.QGridLayout(self)
        
        self.sliders = {}
        row = 0
        for axis in ['roll', 'pitch', 'yaw']:
            label = QtWidgets.QLabel(f"{axis.title()}:")
            slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
            slider.setRange(-180, 180)
            slider.setValue(0)
            value_label = QtWidgets.QLabel("0°")
            
            slider.valueChanged.connect(
                lambda v, l=value_label, a=axis: self.on_slider_changed(v, l, a)
            )
            
            layout.addWidget(label, row, 0)
            layout.addWidget(slider, row, 1)
            layout.addWidget(value_label, row, 2)
            
            self.sliders[axis] = (slider, value_label)
            row += 1
            
        # Animation controls
        self.anim_btn = QtWidgets.QPushButton("Start Animation")
        self.anim_btn.setCheckable(True)
        self.anim_btn.toggled.connect(self.on_animation_toggled)
        layout.addWidget(self.anim_btn, row, 0, 1, 3)
        
        # Animation timer
        self.anim_timer = QtCore.QTimer(self)
        self.anim_timer.timeout.connect(self.update_animation)
        self.anim_angle = 0.0
        
        # Load the host peripherals library
        import ctypes
        from pathlib import Path
        lib_path = Path(__file__).parent.parent.parent / "firmware" / "build" / "libhostperipherals.dll"
        self._lib = ctypes.CDLL(str(lib_path))
        
        self._lib.host_set_sensor_orientation.argtypes = [
            ctypes.c_uint8, ctypes.c_float, ctypes.c_float, ctypes.c_float
        ]
        
    def on_slider_changed(self, value, label, axis):
        label.setText(f"{value}°")
        self.update_orientation()
        
    def update_orientation(self):
        roll = self.sliders['roll'][0].value()
        pitch = self.sliders['pitch'][0].value()
        yaw = self.sliders['yaw'][0].value()
        
        # Update all sensors
        for sensor_id in [0, 1, 2]:  # KX122, LSM6DS3, LSM303C
            self._lib.host_set_sensor_orientation(sensor_id,
                ctypes.c_float(roll),
                ctypes.c_float(pitch),
                ctypes.c_float(yaw))
    
    def on_animation_toggled(self, enabled):
        if enabled:
            self.anim_timer.start(50)  # 20Hz update
            self.anim_btn.setText("Stop Animation")
        else:
            self.anim_timer.stop()
            self.anim_btn.setText("Start Animation")
    
    def update_animation(self):
        import math
        self.anim_angle += 2.0
        if self.anim_angle >= 360.0:
            self.anim_angle = 0.0
        
        # Create smooth circular motion
        roll = 45.0 * math.sin(math.radians(self.anim_angle))
        pitch = 45.0 * math.cos(math.radians(self.anim_angle))
        yaw = self.anim_angle
        
        # Update sliders
        self.sliders['roll'][0].setValue(int(roll))
        self.sliders['pitch'][0].setValue(int(pitch))
        self.sliders['yaw'][0].setValue(int(yaw % 360 - 180))

class MainWindow(QtWidgets.QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle('VM Simulation Runner')

        self.elf_path = QtWidgets.QLineEdit()
        self.emulator_path = QtWidgets.QLineEdit('qemu-arm')
        self.run_btn = QtWidgets.QPushButton('Run')
        self.stop_btn = QtWidgets.QPushButton('Stop')
        self.start_sensors_btn = QtWidgets.QPushButton('Start Sensors')
        self.stop_sensors_btn = QtWidgets.QPushButton('Stop Sensors')
        self.output_view = QtWidgets.QTextEdit()
        self.output_view.setReadOnly(True)
        
        self.orientation_panel = OrientationPanel(self)
        browse_btn = QtWidgets.QPushButton('Browse ELF')

        layout = QtWidgets.QGridLayout(self)
        layout.addWidget(QtWidgets.QLabel('Emulator:'), 0, 0)
        layout.addWidget(self.emulator_path, 0, 1)
        layout.addWidget(QtWidgets.QLabel('ARM ELF:'), 1, 0)
        layout.addWidget(self.elf_path, 1, 1)
        layout.addWidget(browse_btn, 1, 2)
        layout.addWidget(self.start_sensors_btn, 2, 0)
        layout.addWidget(self.stop_sensors_btn, 2, 1)
        layout.addWidget(self.run_btn, 3, 0)
        layout.addWidget(self.stop_btn, 3, 1)
        
        # Add orientation panel in a horizontal layout with the output view
        hlayout = QtWidgets.QHBoxLayout()
        hlayout.addWidget(self.orientation_panel)
        hlayout.addWidget(self.output_view, stretch=1)
        layout.addLayout(hlayout, 4, 0, 1, 3)

        browse_btn.clicked.connect(self.on_browse)
        self.run_btn.clicked.connect(self.on_run)
        self.stop_btn.clicked.connect(self.on_stop)
        self.start_sensors_btn.clicked.connect(self.on_start_sensors)
        self.stop_sensors_btn.clicked.connect(self.on_stop_sensors)

        self.proc_thread = None
        self.sensor_manager = SensorManager()

    def on_browse(self):
        fname, _ = QtWidgets.QFileDialog.getOpenFileName(self, 'Select ARM ELF', '.', 'ELF Files (*)')
        if fname:
            self.elf_path.setText(fname)

    def highlight_and_append(self, text):
        # Very simple highlighting of TEST: PASS/FAIL
        colored = text
        if text.startswith('TEST:') and 'PASS' in text:
            colored = f"<span style='color:green'>{QtCore.Qt.escape(text)}</span>"
        elif text.startswith('TEST:') and 'FAIL' in text:
            colored = f"<span style='color:red'>{QtCore.Qt.escape(text)}</span>"
        else:
            colored = QtCore.QCoreApplication.translate('', text)

        self.output_view.append(text)

    def on_run(self):
        elf = self.elf_path.text().strip()
        emulator = self.emulator_path.text().strip() or 'qemu-arm'
        if not elf or not Path(elf).exists():
            QtWidgets.QMessageBox.warning(self, 'Error', 'Please select a valid ELF to run')
            return

        # Prefer using local run_vm_tests.exe if present, otherwise call emulator directly
        runner = Path('.') / 'run_vm_tests.exe'
        if runner.exists():
            cmd = [str(runner), str(elf)]
        else:
            cmd = [emulator, str(elf)]

        self.output_view.clear()
        self.proc_thread = ProcessThread(cmd)
        self.proc_thread.output.connect(self.on_process_output)
        self.proc_thread.finished_signal.connect(self.on_process_finished)
        self.proc_thread.start()

    def on_stop(self):
        if self.proc_thread:
            self.proc_thread.stop()

    def on_process_output(self, line):
        # basic parse and color
        try:
            if line.startswith('TEST:'):
                if 'PASS' in line:
                    self.output_view.append(f"<span style='color:green'>{line}</span>")
                elif 'FAIL' in line:
                    self.output_view.append(f"<span style='color:red'>{line}</span>")
                else:
                    self.output_view.append(line)
            else:
                self.output_view.append(line)
        except Exception:
            self.output_view.append(line)

    def on_process_finished(self, rc):
        self.output_view.append(f"Process finished with code {rc}")

    def on_start_sensors(self):
        self.sensor_manager.start_all()
        self.output_view.append('Sensor simulators started')

    def on_stop_sensors(self):
        self.sensor_manager.stop_all()
        self.output_view.append('Sensor simulators stopped')


def main():
    app = QtWidgets.QApplication(sys.argv)
    w = MainWindow()
    w.resize(800, 600)
    w.show()
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()
