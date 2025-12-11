import threading
import socket
import time


class _ServerThread(threading.Thread):
    def __init__(self, port, generator, interval=0.1):
        super().__init__(daemon=True)
        self.port = port
        self.generator = generator
        self.interval = interval
        self._stop = threading.Event()
        self._sock = None

    def run(self):
        try:
            self._sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self._sock.bind(('127.0.0.1', self.port))
            self._sock.listen(1)
            while not self._stop.is_set():
                self._sock.settimeout(1.0)
                try:
                    conn, _ = self._sock.accept()
                except socket.timeout:
                    continue
                with conn:
                    while not self._stop.is_set():
                        data = self.generator()
                        try:
                            conn.sendall(data)
                        except Exception:
                            break
                        time.sleep(self.interval)
        except Exception:
            pass
        finally:
            if self._sock:
                try:
                    self._sock.close()
                except Exception:
                    pass

    def stop(self):
        self._stop.set()
        # create connection to unblock
        try:
            s = socket.create_connection(('127.0.0.1', self.port), timeout=0.5)
            s.close()
        except Exception:
            pass


class SensorManager:
    def __init__(self):
        # default ports
        self.imu = _ServerThread(40001, self._gen_imu_packet, interval=0.02)
        self.gps = _ServerThread(40002, self._gen_gps_packet, interval=1.0)
        self.rf = _ServerThread(40003, self._gen_rf_packet, interval=0.2)

    def _gen_imu_packet(self):
        # Get data from host peripherals simulation
        import struct, time, ctypes
        from pathlib import Path

        # Load the host peripherals library if not already loaded
        if not hasattr(self, '_lib'):
            lib_path = Path(__file__).parent.parent.parent / "firmware" / "build" / "libhostperipherals.dll"
            self._lib = ctypes.CDLL(str(lib_path))
            
            # Set up function prototypes
            self._lib.host_get_sensor_orientation.argtypes = [
                ctypes.c_uint8, 
                ctypes.POINTER(ctypes.c_float), 
                ctypes.POINTER(ctypes.c_float), 
                ctypes.POINTER(ctypes.c_float)
            ]

        # Get orientation data from LSM6DS3 (sensor_id = 1)
        roll = ctypes.c_float()
        pitch = ctypes.c_float()
        yaw = ctypes.c_float()
        self._lib.host_get_sensor_orientation(1, ctypes.byref(roll), ctypes.byref(pitch), ctypes.byref(yaw))
        
        # Convert orientation to accelerometer and gyro values
        import math
        g = 9.81  # gravity in m/s^2
        scale_accel = 2000  # Scale to match expected range
        scale_gyro = 500   # Scale to match expected range
        
        # Calculate gravity components based on orientation
        sin_roll = math.sin(math.radians(roll.value))
        cos_roll = math.cos(math.radians(roll.value))
        sin_pitch = math.sin(math.radians(pitch.value))
        cos_pitch = math.cos(math.radians(pitch.value))
        
        # Convert to sensor format
        ts = int(time.time() * 1000) & 0xFFFFFFFF
        ax = int(g * sin_pitch * scale_accel / 9.81)
        ay = int(-g * sin_roll * cos_pitch * scale_accel / 9.81)
        az = int(-g * cos_roll * cos_pitch * scale_accel / 9.81)
        
        # Gyro values from angular rates
        gx = int(roll.value * scale_gyro / 360.0)  # Convert deg to scale
        gy = int(pitch.value * scale_gyro / 360.0)
        gz = int(yaw.value * scale_gyro / 360.0)
        
        return struct.pack('<Ihhhhhh', ts, ax, ay, az, gx, gy, gz)

    def _gen_gps_packet(self):
        # ASCII NMEA-like minimal sentence
        import time, random
        lat = 37.7749 + (random.random() - 0.5) * 0.001
        lon = -122.4194 + (random.random() - 0.5) * 0.001
        ts = time.strftime('%H%M%S', time.gmtime())
        s = f"$GPGGA,{ts},{lat:.6f},N,{lon:.6f},W,1,08,0.9,10.0,M,0.0,M,,\r\n"
        return s.encode('ascii')

    def _gen_rf_packet(self):
        # simple telemetry-like line
        import time, random
        rssi = -30 + int((random.random() - 0.5) * 40)
        seq = int(time.time()) & 0xFFFF
        return f"RF,{seq},{rssi}\n".encode('ascii')

    def start_all(self):
        # Initialize host peripherals if not already done
        if not hasattr(self, '_lib'):
            from pathlib import Path
            import ctypes
            lib_path = Path(__file__).parent.parent.parent / "firmware" / "build" / "libhostperipherals.dll"
            self._lib = ctypes.CDLL(str(lib_path))
            
            # Initialize sensors with default orientation
            for sensor_id in [0, 1, 2]:  # KX122, LSM6DS3, LSM303C
                self._lib.host_set_sensor_orientation(sensor_id,
                    ctypes.c_float(0.0),
                    ctypes.c_float(0.0),
                    ctypes.c_float(0.0))
        
        self.imu.start()
        self.gps.start()
        self.rf.start()

    def stop_all(self):
        self.imu.stop()
        self.gps.stop()
        self.rf.stop()
