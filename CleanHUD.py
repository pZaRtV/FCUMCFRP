"""
Clean GCS HUD - Flight Controller Telemetry
---------------------------------------------------------
Sistem Ground Control Station mandiri dengan Artificial Horizon
dan grafik data sensor real-time. Bebas dari legacy code.
"""

import socket
import struct
import threading
import sys
import math
from collections import deque
import pyqtgraph as pg
from PyQt5 import QtWidgets, QtCore, QtGui

# ============================================================
# KONFIGURASI JARINGAN & PAKET UDP
# ============================================================
UDP_PORT = 8889
PACKET_FORMAT = '<IB25f'
PACKET_SIZE = 105
MAX_BUFFER = 500  # Jumlah titik data pada grafik

# Indeks pemetaan data berdasarkan struktur payload C++
IDX = {
    'time': 0,
    'acc_x': 3, 'acc_y': 4, 'acc_z': 5,
    'gyro_x': 6, 'gyro_y': 7, 'gyro_z': 8,
    'mag_x': 9, 'mag_y': 10, 'mag_z': 11,
    'roll': 21, 'pitch': 22, 'yaw': 23
}

# ============================================================
# KELAS PENERIMA TELEMETRI (BACKGROUND THREAD)
# ============================================================
class TelemetryReceiver:
    def __init__(self):
        self.running = False
        self.sock = None
        
        # Buffer melingkar untuk grafik yang mulus
        self.time_data = deque(maxlen=MAX_BUFFER)
        self.sensor_data = {key: deque(maxlen=MAX_BUFFER) for key in IDX if key != 'time'}
        self.first_time = None

        # Data absolut terbaru untuk Artificial Horizon
        self.current_roll = 0.0
        self.current_pitch = 0.0
        self.current_yaw = 0.0

    def start(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(('', UDP_PORT))
        self.sock.settimeout(1.0)
        self.running = True
        
        print(f"[NETWORK] Mendengarkan telemetri di port {UDP_PORT}...")
        self._listen_loop()

    def _listen_loop(self):
        while self.running:
            try:
                data, _ = self.sock.recvfrom(1024)
                if len(data) == PACKET_SIZE:
                    self._unpack_data(data)
            except socket.timeout:
                continue
            except Exception as e:
                if self.running: print(f"[ERROR] {e}")

    def _unpack_data(self, data):
        try:
            unpacked = struct.unpack(PACKET_FORMAT, data)
            
            # Normalisasi waktu mulai dari 0 detik
            raw_time = unpacked[IDX['time']]
            if self.first_time is None:
                self.first_time = raw_time
            t_sec = (raw_time - self.first_time) / 1000000.0
            
            self.time_data.append(t_sec)
            
            # Simpan data sensor ke buffer
            for key, index in IDX.items():
                if key != 'time':
                    self.sensor_data[key].append(unpacked[index])
            
            # Update data state untuk Horizon
            self.current_roll = unpacked[IDX['roll']]
            self.current_pitch = unpacked[IDX['pitch']]
            self.current_yaw = unpacked[IDX['yaw']]
            
        except struct.error:
            pass

    def stop(self):
        self.running = False
        if self.sock:
            self.sock.close()
        print("[NETWORK] Koneksi ditutup.")

# ============================================================
# WIDGET ARTIFICIAL HORIZON (PFD)
# ============================================================
class ArtificialHorizon(QtWidgets.QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.setMinimumSize(400, 400)
        
        # Palet Warna PFD
        self.sky_color = QtGui.QColor(41, 128, 185)     # Biru Langit
        self.ground_color = QtGui.QColor(139, 115, 85)  # Tanah Coklat
        self.line_color = QtGui.QColor(255, 255, 255)   # Putih
        self.crosshair_color = QtGui.QColor(231, 76, 60)# Merah UI
        self.tape_bg = QtGui.QColor(20, 20, 20, 200)    # Hitam Transparan

    def update_attitude(self, r, p, y):
        self.roll = r
        self.pitch = p
        self.yaw = y
        self.update() # Picu render ulang

    def paintEvent(self, event):
        painter = QtGui.QPainter(self)
        painter.setRenderHint(QtGui.QPainter.Antialiasing)
        
        w, h = self.width(), self.height()
        cx, cy = w / 2, h / 2
        pitch_scale = 4.0 # Sensitivitas pergerakan vertikal horizon
        
        # --- 1. GAMBAR LANGIT & TANAH ---
        painter.save()
        painter.translate(cx, cy)
        painter.rotate(-self.roll)
        painter.translate(0, -self.pitch * pitch_scale)  # Negate: NWU +pitch=nose-up → horizon moves down

        painter.fillRect(int(-w*1.5), int(-h*2), int(w*3), int(h*2), self.sky_color)
        painter.fillRect(int(-w*1.5), 0, int(w*3), int(h*2), self.ground_color)
        
        # Garis Horizon Utama
        painter.setPen(QtGui.QPen(self.line_color, 2))
        painter.drawLine(int(-w), 0, int(w), 0)

        # Pitch Ladder (Derajat Kemiringan)
        font = painter.font()
        font.setPixelSize(12)
        font.setBold(True)
        painter.setFont(font)
        
        for i in range(-90, 91, 10):
            if i == 0: continue
            y_pos = -i * pitch_scale
            line_w = 40 if i % 20 == 0 else 20
            painter.drawLine(int(-line_w), int(y_pos), int(line_w), int(y_pos))
            painter.drawText(int(line_w + 8), int(y_pos + 4), str(abs(i)))
            painter.drawText(int(-line_w - 25), int(y_pos + 4), str(abs(i)))
        painter.restore()

        # --- 2. KOMPAS (HEADING TAPE) ---
        painter.save()
        painter.fillRect(0, 0, int(w), 35, self.tape_bg)
        painter.setPen(QtGui.QPen(self.line_color, 1))
        
        fov_deg = 60
        px_per_deg = w / fov_deg
        
        yaw_display = (-self.yaw) % 360  # Negate: NWU +yaw=CCW, compass tape scrolls CW for heading
        for angle in range(0, 360, 15):
            diff = (angle - yaw_display + 180) % 360 - 180
            if abs(diff) <= fov_deg / 2 + 10:
                x = cx + (diff * px_per_deg)
                painter.drawLine(int(x), 35, int(x), 25)
                
                text = str(angle)
                if angle == 0: text = "N"
                elif angle == 90: text = "E"
                elif angle == 180: text = "S"
                elif angle == 270: text = "W"
                
                painter.drawText(int(x - 8), 20, text)
                
        # Indikator Tengah Kompas
        painter.setPen(QtGui.QPen(self.crosshair_color, 2))
        painter.drawLine(int(cx), 35, int(cx-6), 45)
        painter.drawLine(int(cx), 35, int(cx+6), 45)
        painter.drawLine(int(cx-6), 45, int(cx+6), 45)
        painter.restore()

        # --- 3. CROSSHAIR WAHANA TENGAH ---
        painter.setPen(QtGui.QPen(self.crosshair_color, 4))
        painter.drawLine(int(cx - 60), int(cy), int(cx - 20), int(cy)) # Sayap Kiri
        painter.drawLine(int(cx + 20), int(cy), int(cx + 60), int(cy)) # Sayap Kanan
        painter.drawLine(int(cx), int(cy - 20), int(cx), int(cy - 5))  # Sirip Atas

        # Teks Bantuan Tambahan
        painter.setPen(QtGui.QPen(self.line_color, 1))
        painter.drawText(10, h - 10, f"R: {self.roll:05.1f}° | P: {self.pitch:05.1f}°")

# ============================================================
# KELAS UTAMA GUI (PYQTGRAPH)
# ============================================================
class ModernGCS(QtWidgets.QMainWindow):
    def __init__(self, telemetry):
        super().__init__()
        self.telemetry = telemetry
        self.setWindowTitle("FCU Ground Control Station - Clean Build")
        self.resize(1300, 800)

        pg.setConfigOption('background', '#121212')
        pg.setConfigOption('foreground', '#d3d3d3')

        self.central_widget = QtWidgets.QWidget()
        self.setCentralWidget(self.central_widget)
        self.layout = QtWidgets.QGridLayout(self.central_widget)

        # Inisiasi Widget
        self.pfd = ArtificialHorizon()
        self.plot_acc = pg.PlotWidget(title="Accelerometer (g)")
        self.plot_gyr = pg.PlotWidget(title="Gyroscope (deg/s)")
        self.plot_mag = pg.PlotWidget(title="Magnetometer (uT)")

        for plot in [self.plot_acc, self.plot_gyr, self.plot_mag]:
            plot.showGrid(x=True, y=True, alpha=0.3)
            plot.addLegend(offset=(10, 10))

        # Penataan Layout (Grid 3x2)
        self.layout.addWidget(self.pfd, 0, 0, 3, 1)
        self.layout.addWidget(self.plot_acc, 0, 1)
        self.layout.addWidget(self.plot_gyr, 1, 1)
        self.layout.addWidget(self.plot_mag, 2, 1)
        
        self.layout.setColumnStretch(0, 1)
        self.layout.setColumnStretch(1, 1)

        # Style Garis (Merah=X, Hijau=Y, Biru=Z)
        pX = pg.mkPen('r', width=2)
        pY = pg.mkPen('g', width=2)
        pZ = pg.mkPen('b', width=2)

        # Binding Kurva
        self.c_acc_x = self.plot_acc.plot(pen=pX, name="Acc X")
        self.c_acc_y = self.plot_acc.plot(pen=pY, name="Acc Y")
        self.c_acc_z = self.plot_acc.plot(pen=pZ, name="Acc Z")

        self.c_gyr_x = self.plot_gyr.plot(pen=pX, name="Gyr X")
        self.c_gyr_y = self.plot_gyr.plot(pen=pY, name="Gyr Y")
        self.c_gyr_z = self.plot_gyr.plot(pen=pZ, name="Gyr Z")

        self.c_mag_x = self.plot_mag.plot(pen=pX, name="Mag X")
        self.c_mag_y = self.plot_mag.plot(pen=pY, name="Mag Y")
        self.c_mag_z = self.plot_mag.plot(pen=pZ, name="Mag Z")

        # Timer UI (Refresh Rate 30 FPS)
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update_ui)
        self.timer.start(33)

    def update_ui(self):
        if not self.telemetry.running or len(self.telemetry.time_data) == 0:
            return

        # 1. Update Instrumen PFD
        self.pfd.update_attitude(
            self.telemetry.current_roll,
            self.telemetry.current_pitch,
            self.telemetry.current_yaw
        )

        # 2. Update Grafik Garis
        t = list(self.telemetry.time_data)
        sd = self.telemetry.sensor_data
        
        try:
            self.c_acc_x.setData(t, list(sd['acc_x']))
            self.c_acc_y.setData(t, list(sd['acc_y']))
            self.c_acc_z.setData(t, list(sd['acc_z']))

            self.c_gyr_x.setData(t, list(sd['gyro_x']))
            self.c_gyr_y.setData(t, list(sd['gyro_y']))
            self.c_gyr_z.setData(t, list(sd['gyro_z']))

            self.c_mag_x.setData(t, list(sd['mag_x']))
            self.c_mag_y.setData(t, list(sd['mag_y']))
            self.c_mag_z.setData(t, list(sd['mag_z']))
        except IndexError:
            pass # Cegah crash jika frame terpotong

# ============================================================
# ENTRY POINT
# ============================================================
if __name__ == "__main__":
    print("=" * 50)
    print("MEMULAI GCS HUD MANDIRI")
    print("=" * 50)

    app = QtWidgets.QApplication(sys.argv)
    
    telemetry_engine = TelemetryReceiver()
    rx_thread = threading.Thread(target=telemetry_engine.start, daemon=True)
    
    gui = ModernGCS(telemetry_engine)
    
    try:
        rx_thread.start()
        gui.show()
        sys.exit(app.exec_())
    except KeyboardInterrupt:
        print("\nMenutup GCS...")
    finally:
        telemetry_engine.stop()