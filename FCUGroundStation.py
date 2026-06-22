"""
FCU IMU Telemetry Ground Station
------------------------------------------------------------
Merged single-file application containing UDP Data Client 
and Modern PyQtGraph HUD.

Author: Patrick Andrasena T., Nathanael Indra R. P.
Date: 2026-06-14
"""

import socket
import struct
import csv
import os
from datetime import datetime
import time
import threading
import queue
import numpy as np

import pyqtgraph as pg
from PyQt5 import QtWidgets, QtCore
import sys

# ============================================================
# UDP & PACKET CONFIGURATION
# ============================================================
UDP_LOCAL_PORT = 8889
UDP_BUFFER_SIZE = 1500

# Fix: 105 Bytes packet format
PACKET_FORMAT = '<IB25f'
PACKET_SIZE = 105

# Logged CSV columns (timestamped)
FIELD_NAMES = [
    'timestamp_us', 'event_flags', 'B_madgwick',
    'ctrl_acc_x', 'ctrl_acc_y', 'ctrl_acc_z',
    'ctrl_gyro_x', 'ctrl_gyro_y', 'ctrl_gyro_z',
    'ctrl_mag_x', 'ctrl_mag_y', 'ctrl_mag_z',
    'mon_acc_x', 'mon_acc_y', 'mon_acc_z',
    'mon_gyro_x', 'mon_gyro_y', 'mon_gyro_z',
    'mon_mag_x', 'mon_mag_y', 'mon_mag_z',
    'ctrl_roll', 'ctrl_pitch', 'ctrl_yaw',
    'mon_roll', 'mon_pitch', 'mon_yaw',
]

EVENT_FLAGS = {
    'command_change': 0b00000001,
    'throttle_change': 0b00000010,
    'roll_command': 0b00000100,
    'pitch_command': 0b00001000,
    'yaw_command': 0b00010000,
    'attitude_change': 0b00100000,
    'control_active': 0b01000000,
    'disturbance': 0b10000000
}

MAX_DATA_POINTS = 1000  # Number of points to keep in rolling buffer

# ============================================================
# UDP DATA CLIENT LOGIC
# ============================================================
class UDPDataClient:
    def __init__(self):
        self.sock = None
        self.data_queue = queue.Queue()
        self.running = False
        self.packet_count = 0
        self.start_time = None
        self.first_timestamp_us = None
        
        self.event_log = []
        self.last_event_flags = 0
        self.event_log_file = None
        
        self.session_timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.output_dir = os.path.join("data_logs", self.session_timestamp)
        os.makedirs(self.output_dir, exist_ok=True)
        
        self.csv_file = os.path.join(self.output_dir, "imu_data.csv")
        self.csv_writer = None
        self.csv_file_handle = None
        
        self.event_log_file = os.path.join(self.output_dir, "event_log.txt")
        self.event_log_file_handle = None
        
        from collections import deque
        self.time_buffer = deque(maxlen=MAX_DATA_POINTS)
        self.data_buffers = {field: deque(maxlen=MAX_DATA_POINTS) for field in FIELD_NAMES[2:]}
        
        self._init_csv()
        self._init_event_log()
        
        print(f"Data logging initialized:\n  Output: {self.output_dir}")
    
    def _init_csv(self):
        self.csv_file_handle = open(self.csv_file, 'w', newline='')
        self.csv_writer = csv.DictWriter(self.csv_file_handle, fieldnames=FIELD_NAMES)
        self.csv_writer.writeheader()
        self.csv_file_handle.flush()
    
    def _init_event_log(self):
        self.event_log_file_handle = open(self.event_log_file, 'w')
        self.event_log_file_handle.write("# FCU IMU Event Log\n")
        self.event_log_file_handle.write(f"# Session: {self.session_timestamp}\n")
        self.event_log_file_handle.write(f"# Started: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
        self.event_log_file_handle.write("# Format: [Timestamp] [Event_Type] [Description]\n#\n")
        self.event_log_file_handle.flush()
    
    def _log_event(self, timestamp_us, event_flags, packet_data):
        if event_flags == self.last_event_flags:
            return
        timestamp_s = timestamp_us / 1_000_000
        for flag_name, flag_bit in EVENT_FLAGS.items():
            if flag_name == 'reserved': continue
            flag_set = bool(event_flags & flag_bit)
            prev_flag_set = bool(self.last_event_flags & flag_bit)
            if flag_set != prev_flag_set and flag_set:
                description = self._get_event_description(flag_name, packet_data)
                log_entry = f"[{timestamp_s:.6f}] [{flag_name.upper()}] {description}\n"
                if self.event_log_file_handle and not self.event_log_file_handle.closed:
                    self.event_log_file_handle.write(log_entry)
                    self.event_log_file_handle.flush()
                print(f"EVENT: {flag_name.upper()} - {description}")
        self.last_event_flags = event_flags
    
    def _get_event_description(self, flag_name, packet_data):
        if flag_name == 'command_change': return f"Command input detected"
        elif flag_name == 'throttle_change': return f"Throttle change detected"
        elif flag_name == 'roll_command': return f"Roll command active"
        elif flag_name == 'pitch_command': return f"Pitch command active"
        elif flag_name == 'yaw_command': return f"Yaw command active"
        elif flag_name == 'attitude_change': return f"Attitude change beyond threshold"
        elif flag_name == 'disturbance': return f"DISTURBANCE DETECTED - POSSIBLE WIND/TURBULENCE"
        elif flag_name == 'control_active': return "Control system activated (Armed)"
        else: return f"Event: {flag_name}"
    
    def start(self):
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.sock.bind(('', UDP_LOCAL_PORT))
            self.sock.settimeout(1.0)
            
            self.running = True
            self.start_time = time.time()
            print(f"UDP listener started on port {UDP_LOCAL_PORT}\nWaiting for data packets...")
            
            processing_thread = threading.Thread(target=self._process_data, daemon=True)
            processing_thread.start()
            
            while self.running:
                try:
                    data, addr = self.sock.recvfrom(UDP_BUFFER_SIZE)
                    if len(data) == PACKET_SIZE:
                        self.data_queue.put((data, addr))
                except socket.timeout:
                    continue
                except Exception as e:
                    if self.running: print(f"Error receiving data: {e}")
        except Exception as e:
            print(f"Error starting UDP client: {e}")
        finally:
            self.stop()
    
    def _process_data(self):
        while self.running:
            try:
                data, addr = self.data_queue.get(timeout=0.1)
                try:
                    unpacked = struct.unpack(PACKET_FORMAT, data)
                    packet_dict = dict(zip(FIELD_NAMES, unpacked))
                    
                    if self.first_timestamp_us is None:
                        self.first_timestamp_us = packet_dict['timestamp_us']
                    
                    relative_time_s = (packet_dict['timestamp_us'] - self.first_timestamp_us) / 1e6
                    self._log_event(packet_dict['timestamp_us'], packet_dict['event_flags'], packet_dict)
                    
                    if self.csv_file_handle and not self.csv_file_handle.closed:
                        self.csv_writer.writerow(packet_dict)
                        self.csv_file_handle.flush()
                    
                    self.time_buffer.append(relative_time_s)
                    for field in FIELD_NAMES[2:]:
                        self.data_buffers[field].append(packet_dict[field])
                    
                    self.packet_count += 1
                except struct.error as e:
                    pass
            except queue.Empty:
                continue
            except Exception as e:
                pass
    
    def stop(self):
        self.running = False
        if self.sock: self.sock.close()
        if self.csv_file_handle and not self.csv_file_handle.closed:
            self.csv_file_handle.close()
            
        # Fix: I/O closed file bug
        if hasattr(self, 'event_log_file_handle') and self.event_log_file_handle and not self.event_log_file_handle.closed:
            self.event_log_file_handle.write(f"# Session ended: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
            self.event_log_file_handle.write(f"# Total packets received: {self.packet_count}\n")
            self.event_log_file_handle.close()
        
        print(f"\nSession complete: {self.packet_count} packets received.")

# ============================================================
# MODERN HUD GUI (PyQtGraph)
# ============================================================
class ModernHUD(QtWidgets.QMainWindow):
    def __init__(self, data_client):
        super().__init__()
        self.data_client = data_client
        self.setWindowTitle("FCU Telemetry HUD - Full Data & Legend")
        self.resize(1280, 800)

        pg.setConfigOption('background', '#121212')
        pg.setConfigOption('foreground', '#d3d3d3')

        self.central_widget = QtWidgets.QWidget()
        self.setCentralWidget(self.central_widget)
        self.layout = QtWidgets.QGridLayout(self.central_widget)

        self.plot_attitude = pg.PlotWidget(title="Attitude Comparison (Roll, Pitch, Yaw)")
        self.plot_accel = pg.PlotWidget(title="Accelerometer (g)")
        self.plot_gyro = pg.PlotWidget(title="Gyroscope (deg/s)")
        self.plot_mag = pg.PlotWidget(title="Magnetometer (uT)")

        for plot in [self.plot_attitude, self.plot_accel, self.plot_gyro, self.plot_mag]:
            plot.showGrid(x=True, y=True, alpha=0.3)
            plot.addLegend(offset=(10, 10))

        self.layout.addWidget(self.plot_attitude, 0, 0)
        self.layout.addWidget(self.plot_mag, 0, 1)
        self.layout.addWidget(self.plot_accel, 1, 0)
        self.layout.addWidget(self.plot_gyro, 1, 1)

        pen_ctrl_r = pg.mkPen('r', width=2)
        pen_ctrl_g = pg.mkPen('g', width=2)
        pen_ctrl_b = pg.mkPen('b', width=2)
        
        pen_mon_r = pg.mkPen('r', width=1.5, style=QtCore.Qt.DashLine)
        pen_mon_g = pg.mkPen('g', width=1.5, style=QtCore.Qt.DashLine)
        pen_mon_b = pg.mkPen('b', width=1.5, style=QtCore.Qt.DashLine)

        self.c_roll_ctrl = self.plot_attitude.plot(pen=pen_ctrl_r, name="Ctrl Roll")
        self.c_pitch_ctrl = self.plot_attitude.plot(pen=pen_ctrl_g, name="Ctrl Pitch")
        self.c_yaw_ctrl = self.plot_attitude.plot(pen=pen_ctrl_b, name="Ctrl Yaw")
        self.c_roll_mon = self.plot_attitude.plot(pen=pen_mon_r, name="Mon Roll")
        self.c_pitch_mon = self.plot_attitude.plot(pen=pen_mon_g, name="Mon Pitch")
        self.c_yaw_mon = self.plot_attitude.plot(pen=pen_mon_b, name="Mon Yaw")

        self.c_acc_x_ctrl = self.plot_accel.plot(pen=pen_ctrl_r, name="Ctrl Acc X")
        self.c_acc_y_ctrl = self.plot_accel.plot(pen=pen_ctrl_g, name="Ctrl Acc Y")
        self.c_acc_z_ctrl = self.plot_accel.plot(pen=pen_ctrl_b, name="Ctrl Acc Z")
        self.c_acc_x_mon = self.plot_accel.plot(pen=pen_mon_r, name="Mon Acc X")
        self.c_acc_y_mon = self.plot_accel.plot(pen=pen_mon_g, name="Mon Acc Y")
        self.c_acc_z_mon = self.plot_accel.plot(pen=pen_mon_b, name="Mon Acc Z")

        self.c_gyro_x_ctrl = self.plot_gyro.plot(pen=pen_ctrl_r, name="Ctrl Gyro X")
        self.c_gyro_y_ctrl = self.plot_gyro.plot(pen=pen_ctrl_g, name="Ctrl Gyro Y")
        self.c_gyro_z_ctrl = self.plot_gyro.plot(pen=pen_ctrl_b, name="Ctrl Gyro Z")
        self.c_gyro_x_mon = self.plot_gyro.plot(pen=pen_mon_r, name="Mon Gyro X")
        self.c_gyro_y_mon = self.plot_gyro.plot(pen=pen_mon_g, name="Mon Gyro Y")
        self.c_gyro_z_mon = self.plot_gyro.plot(pen=pen_mon_b, name="Mon Gyro Z")

        self.c_mag_x_ctrl = self.plot_mag.plot(pen=pen_ctrl_r, name="Ctrl Mag X")
        self.c_mag_y_ctrl = self.plot_mag.plot(pen=pen_ctrl_g, name="Ctrl Mag Y")
        self.c_mag_z_ctrl = self.plot_mag.plot(pen=pen_ctrl_b, name="Ctrl Mag Z")
        self.c_mag_x_mon = self.plot_mag.plot(pen=pen_mon_r, name="Mon Mag X")
        self.c_mag_y_mon = self.plot_mag.plot(pen=pen_mon_g, name="Mon Mag Y")
        self.c_mag_z_mon = self.plot_mag.plot(pen=pen_mon_b, name="Mon Mag Z")

        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update_plots)
        self.timer.start(50)

    def update_plots(self):
        if not self.data_client.running or len(self.data_client.time_buffer) == 0:
            return

        time_data = list(self.data_client.time_buffer)
        db = self.data_client.data_buffers 

        try:
            self.c_roll_ctrl.setData(time_data, list(db['ctrl_roll']))
            self.c_pitch_ctrl.setData(time_data, list(db['ctrl_pitch']))
            self.c_yaw_ctrl.setData(time_data, list(db['ctrl_yaw']))
            self.c_roll_mon.setData(time_data, list(db['mon_roll']))
            self.c_pitch_mon.setData(time_data, list(db['mon_pitch']))
            self.c_yaw_mon.setData(time_data, list(db['mon_yaw']))

            self.c_acc_x_ctrl.setData(time_data, list(db['ctrl_acc_x']))
            self.c_acc_y_ctrl.setData(time_data, list(db['ctrl_acc_y']))
            self.c_acc_z_ctrl.setData(time_data, list(db['ctrl_acc_z']))
            self.c_acc_x_mon.setData(time_data, list(db['mon_acc_x']))
            self.c_acc_y_mon.setData(time_data, list(db['mon_acc_y']))
            self.c_acc_z_mon.setData(time_data, list(db['mon_acc_z']))

            self.c_gyro_x_ctrl.setData(time_data, list(db['ctrl_gyro_x']))
            self.c_gyro_y_ctrl.setData(time_data, list(db['ctrl_gyro_y']))
            self.c_gyro_z_ctrl.setData(time_data, list(db['ctrl_gyro_z']))
            self.c_gyro_x_mon.setData(time_data, list(db['mon_gyro_x']))
            self.c_gyro_y_mon.setData(time_data, list(db['mon_gyro_y']))
            self.c_gyro_z_mon.setData(time_data, list(db['mon_gyro_z']))

            self.c_mag_x_ctrl.setData(time_data, list(db['ctrl_mag_x']))
            self.c_mag_y_ctrl.setData(time_data, list(db['ctrl_mag_y']))
            self.c_mag_z_ctrl.setData(time_data, list(db['ctrl_mag_z']))
            self.c_mag_x_mon.setData(time_data, list(db['mon_mag_x']))
            self.c_mag_y_mon.setData(time_data, list(db['mon_mag_y']))
            self.c_mag_z_mon.setData(time_data, list(db['mon_mag_z']))

        except KeyError:
            pass

# ============================================================
# MAIN EXECUTION
# ============================================================
def main():
    print("=" * 60)
    print("FCU IMU Telemetry Ground Station")
    print("=" * 60)
    
    data_client = UDPDataClient()
    
    app = QtWidgets.QApplication(sys.argv)
    hud = ModernHUD(data_client)
    
    # Menjalankan pendengar UDP di background thread
    udp_thread = threading.Thread(target=data_client.start, daemon=True)
    
    try:
        udp_thread.start()
        hud.show()
        sys.exit(app.exec_())
    except KeyboardInterrupt:
        print("\nShutting down...")
    finally:
        data_client.stop()

if __name__ == "__main__":
    main()