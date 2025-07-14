# main.py - Base Station UI (PyQt5 starter)
import sys
import threading
import time
from PyQt5.QtWidgets import QApplication, QMainWindow, QLabel, QVBoxLayout, QWidget, QPushButton, QHBoxLayout, QProgressBar, QGraphicsView, QGraphicsScene, QGraphicsEllipseItem, QGraphicsTextItem, QGroupBox
from PyQt5.QtGui import QFont, QColor, QPalette
from PyQt5.QtCore import Qt, QTimer, pyqtSignal

class MapWidget(QGraphicsView):
    def __init__(self, base_lat, base_lon, parent=None):
        super().__init__(parent)
        self.base_lat = base_lat
        self.base_lon = base_lon
        self.scene = QGraphicsScene(self)
        self.setScene(self.scene)
        self.setFixedHeight(300)
        self.setFixedWidth(400)
        # Draw base station
        self.base_item = QGraphicsEllipseItem(195, 145, 10, 10)
        self.base_item.setBrush(QColor('#f1fa8c'))
        self.scene.addItem(self.base_item)
        self.base_label = QGraphicsTextItem("Base")
        self.base_label.setDefaultTextColor(QColor('#f1fa8c'))
        self.base_label.setPos(200, 130)
        self.scene.addItem(self.base_label)
        # Drone marker
        self.drone_item = QGraphicsEllipseItem(195, 195, 10, 10)
        self.drone_item.setBrush(QColor('#50fa7b'))
        self.scene.addItem(self.drone_item)
        self.drone_label = QGraphicsTextItem("Drone")
        self.drone_label.setDefaultTextColor(QColor('#50fa7b'))
        self.drone_label.setPos(200, 210)
        self.scene.addItem(self.drone_label)
        # Waypoints
        self.waypoint_items = []

    def update_drone_position(self, drone_lat, drone_lon):
        # Simple flat projection: 1 deg lat/lon = 100 px (for demo)
        dx = (drone_lon - self.base_lon) * 100
        dy = -(drone_lat - self.base_lat) * 100
        self.drone_item.setRect(195 + dx, 145 + dy, 10, 10)
        self.drone_label.setPos(200 + dx, 210 + dy)

    def set_waypoints(self, waypoints):
        # waypoints: list of (lat, lon)
        # Remove old waypoints
        for item in self.waypoint_items:
            self.scene.removeItem(item)
        self.waypoint_items = []
        for i, (lat, lon) in enumerate(waypoints):
            dx = (lon - self.base_lon) * 100
            dy = -(lat - self.base_lat) * 100
            wp = QGraphicsEllipseItem(195 + dx, 145 + dy, 8, 8)
            wp.setBrush(QColor('#ff79c6'))
            self.scene.addItem(wp)
            label = QGraphicsTextItem(f"WP{i+1}")
            label.setDefaultTextColor(QColor('#ff79c6'))
            label.setPos(195 + dx, 135 + dy)
            self.scene.addItem(label)
            self.waypoint_items.extend([wp, label])

class MainWindow(QMainWindow):
    position_update_signal = pyqtSignal(float, float)
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Drone Base Station UI")
        self.setStyleSheet("background-color: #181c24; color: #f8f8f2;")
        layout = QVBoxLayout()
        title = QLabel("🛸 Drone Command Center 🛸")
        title.setFont(QFont("Orbitron", 20, QFont.Bold))
        title.setAlignment(Qt.AlignCenter)
        layout.addWidget(title)
        self.status = QLabel("Status: Connected")
        self.status.setFont(QFont("Consolas", 12, QFont.Bold))
        self.telemetry = QLabel("Telemetry: --")
        self.telemetry.setFont(QFont("Consolas", 11))
        # Map display
        self.map_group = QGroupBox("Position Map")
        map_layout = QVBoxLayout()
        self.map_widget = MapWidget(base_lat=37.7749, base_lon=-122.4194)  # Example: San Francisco
        map_layout.addWidget(self.map_widget)
        self.map_group.setLayout(map_layout)
        layout.addWidget(self.map_group)
        # RF Monitoring UI
        rf_layout = QHBoxLayout()
        self.rf_signal = QProgressBar()
        self.rf_signal.setRange(0, 100)
        self.rf_signal.setValue(0)
        self.rf_signal.setFormat("RF Signal: %p%")
        self.rf_signal.setStyleSheet("QProgressBar {background: #222; border: 2px solid #444;} QProgressBar::chunk {background: #50fa7b;}")
        self.rf_tx = QLabel("TX: 0")
        self.rf_rx = QLabel("RX: 0")
        self.rf_errors = QLabel("Errors: 0")
        for lbl in [self.rf_tx, self.rf_rx, self.rf_errors]:
            lbl.setFont(QFont("Consolas", 10, QFont.Bold))
            lbl.setStyleSheet("color: #8be9fd;")
        rf_layout.addWidget(self.rf_signal)
        rf_layout.addWidget(self.rf_tx)
        rf_layout.addWidget(self.rf_rx)
        rf_layout.addWidget(self.rf_errors)
        self.send_btn = QPushButton("🚀 Launch Command")
        self.send_btn.setFont(QFont("Orbitron", 12, QFont.Bold))
        self.send_btn.setStyleSheet("background: #282a36; color: #f1fa8c; border-radius: 8px; padding: 8px 16px;")
        layout.addWidget(self.status)
        layout.addWidget(self.telemetry)
        layout.addLayout(rf_layout)
        layout.addWidget(self.send_btn)
        container = QWidget()
        container.setLayout(layout)
        self.setCentralWidget(container)
        # Simulate RF and position updates (replace with real data source)
        self.update_rf_monitor(75, 10, 8, 1)
        self.update_drone_position(37.7752, -122.4189)  # Example: drone near base
        self.set_waypoints([
            (37.7755, -122.4190),
            (37.7758, -122.4187),
            (37.7760, -122.4185)
        ])
        # Timer for background updates
        self.timer = QTimer(self)
        self.timer.timeout.connect(self._background_update_position)
        self.timer.start(1000)  # Update every 1 second
        self.position_update_signal.connect(self.update_drone_position)
        self._start_telemetry_thread()

    def update_rf_monitor(self, signal, tx, rx, errors):
        self.rf_signal.setValue(signal)
        self.rf_tx.setText(f"TX: {tx}")
        self.rf_rx.setText(f"RX: {rx}")
        self.rf_errors.setText(f"Errors: {errors}")

    def update_drone_position(self, drone_lat, drone_lon):
        self.map_widget.update_drone_position(drone_lat, drone_lon)

    def set_waypoints(self, waypoints):
        self.map_widget.set_waypoints(waypoints)

    def _start_telemetry_thread(self):
        def telemetry_loop():
            # Replace this with your real data source (e.g., serial, socket, etc.)
            while True:
                # Example: read from a file, socket, or serial port
                # For demo, simulate receiving new position every second
                # Replace the next two lines with real data parsing
                lat, lon = self._drone_lat, self._drone_lon
                # --- Real data integration example ---
                # lat, lon = get_position_from_serial()  # or socket, etc.
                self.position_update_signal.emit(lat, lon)
                time.sleep(1)
        t = threading.Thread(target=telemetry_loop, daemon=True)
        t.start()

    def _background_update_position(self):
        # Remove the demo movement logic; real data will update position
        pass

app = QApplication(sys.argv)
window = MainWindow()
window.show()
sys.exit(app.exec_())
