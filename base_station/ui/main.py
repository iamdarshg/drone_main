# main.py - Base Station UI (PyQt5 starter)
import sys
from PyQt5.QtWidgets import QApplication, QMainWindow, QLabel, QVBoxLayout, QWidget, QPushButton

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Drone Base Station UI")
        layout = QVBoxLayout()
        self.status = QLabel("Status: Connected")
        self.telemetry = QLabel("Telemetry: --")
        self.send_btn = QPushButton("Send Command")
        layout.addWidget(self.status)
        layout.addWidget(self.telemetry)
        layout.addWidget(self.send_btn)
        container = QWidget()
        container.setLayout(layout)
        self.setCentralWidget(container)

app = QApplication(sys.argv)
window = MainWindow()
window.show()
sys.exit(app.exec_())
