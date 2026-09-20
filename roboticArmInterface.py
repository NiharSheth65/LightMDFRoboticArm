import sys
import math
import time
import serial
import serial.tools.list_ports

from PyQt6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QGridLayout, QLabel, QPushButton, QLineEdit, QCheckBox,
    QGroupBox, QFrame, QComboBox
)
from PyQt6.QtGui import QPainter, QColor, QPen, QBrush, QFont
from PyQt6.QtCore import Qt, QPointF, pyqtSignal, QObject, QThread, QTimer

# ----------------- ROBOT CONSTANTS -----------------
L1 = 120.0             # mm
L2 = 100.0             # mm
Z_DRAW_SURFACE = 20.0  # mm
SEGMENT_RESOLUTION = 5.0  # mm spacing for linear interpolation


def calculate_ik(x, y, z):
    """
    X = Forward / Backward
    Y = Side-to-Side
    Z = Up / Down
    """
    base_deg = math.degrees(math.atan2(y, x)) if (x != 0 or y != 0) else 0.0
    xy = math.sqrt(x * x + y * y)
    r_sq = xy * xy + z * z
    r = math.sqrt(r_sq)

    if r > (L1 + L2) or r < abs(L1 - L2) or r == 0:
        return None

    cos_t2 = (L1 * L1 + r_sq - L2 * L2) / (2.0 * L1 * r)
    cos_t3 = (L1 * L1 + L2 * L2 - r_sq) / (2.0 * L1 * L2)

    cos_t2 = max(-1.0, min(1.0, cos_t2))
    cos_t3 = max(-1.0, min(1.0, cos_t3))

    theta1 = math.degrees(math.atan2(z, xy))
    theta2 = math.degrees(math.acos(cos_t2))
    theta3 = math.degrees(math.acos(cos_t3))

    shoulder_deg = theta1 + theta2
    elbow_deg = -(180.0 - theta3)
    wrist_deg = 90.0 + (shoulder_deg + elbow_deg)

    return {
        "base": base_deg,
        "shoulder": shoulder_deg,
        "elbow": elbow_deg,
        "wrist": wrist_deg,
        "xy": xy,
        "z": z,
    }


# ----------------- SERIAL WORKER THREAD -----------------
class SerialWorker(QObject):
    line_received = pyqtSignal(str)
    connection_status = pyqtSignal(bool, str)

    def __init__(self):
        super().__init__()
        self.serial_port = None
        self.is_running = True

    def connect_serial(self, port, baud=115200):
        try:
            if self.serial_port and self.serial_port.is_open:
                self.serial_port.close()
            self.serial_port = serial.Serial(port, baud, timeout=0.1)
            time.sleep(2.0)  # Arduino bootloader reset delay
            self.connection_status.emit(True, f"Connected to {port}")
        except Exception as e:
            self.connection_status.emit(False, str(e))

    def disconnect_serial(self):
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
        self.connection_status.emit(False, "Disconnected")

    def send_command(self, cmd: str):
        if self.serial_port and self.serial_port.is_open:
            clean_cmd = cmd.strip() + "\n"
            self.serial_port.write(clean_cmd.encode("utf-8"))
            self.serial_port.flush()

    def run(self):
        while self.is_running:
            if self.serial_port and self.serial_port.is_open:
                try:
                    line = self.serial_port.readline().decode("utf-8").strip()
                    if line:
                        self.line_received.emit(line)
                except Exception:
                    pass
            time.sleep(0.003)


# ----------------- WORKSPACE CANVAS -----------------
class WorkspaceCanvas(QFrame):
    point_clicked = pyqtSignal(float, float, float)

    BASE_OBSTACLE_RADIUS = 65.0  # mm
    GRID_STEP_MM = 10.0          # 1 cm snapping

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(480, 440)
        self.setStyleSheet("background-color: #1a1b26; border-radius: 8px;")
        self.points = []  # Tuples: (x, y, z, is_linear)
        self.is_homed = False
        self.scale = 1.4

    def set_homed(self, state: bool):
        self.is_homed = state
        self.update()

    def set_points(self, points):
        self.points = points
        self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)

        w = self.width()
        h = self.height()
        cx = w / 2.0
        cy = h - 45.0

        max_y_mm = int(cx / self.scale)
        max_x_mm = int((cy - 30) / self.scale)

        # 1. 10 mm Grid
        for y_mm in range(-max_y_mm, max_y_mm + 1, int(self.GRID_STEP_MM)):
            screen_y_col = cx + y_mm * self.scale
            painter.setPen(QPen(QColor("#3b4261") if y_mm % 50 == 0 else QColor("#24283b"), 1))
            painter.drawLine(int(screen_y_col), int(cy - max_x_mm * self.scale), int(screen_y_col), int(cy))

        for x_mm in range(0, max_x_mm + 1, int(self.GRID_STEP_MM)):
            screen_x_row = cy - x_mm * self.scale
            painter.setPen(QPen(QColor("#3b4261") if x_mm % 50 == 0 else QColor("#24283b"), 1))
            painter.drawLine(int(cx - max_y_mm * self.scale), int(screen_x_row), int(cx + max_y_mm * self.scale), int(screen_x_row))

        # 2. Solid Primary Axes
        painter.setPen(QPen(QColor("#7aa2f7"), 2))
        painter.drawLine(15, int(cy), w - 15, int(cy))
        painter.drawLine(int(cx), int(cy), int(cx), int(cy - max_x_mm * self.scale))

        # 3. Ticks & Labels
        painter.setFont(QFont("Arial", 8))
        painter.setPen(QPen(QColor("#7aa2f7"), 1))
        for y_mm in range(-200, 201, 50):
            if y_mm != 0 and abs(y_mm) <= max_y_mm:
                painter.drawText(int(cx + y_mm * self.scale - 12), int(cy + 16), f"{y_mm}")
        for x_mm in range(50, max_x_mm + 1, 50):
            painter.drawText(int(cx + 6), int(cy - x_mm * self.scale - 2), f"{x_mm}")

        painter.setFont(QFont("Segoe UI", 9, QFont.Weight.Bold))
        painter.setPen(QPen(QColor("#bb9af7")))
        painter.drawText(int(cx - 70), 22, "▲ +X (Forward)")
        painter.drawText(w - 130, int(cy - 6), "+Y (Right) ▶")
        painter.drawText(15, int(cy - 6), "◀ -Y (Left)")

        # 4. Reachable Arc
        r_max = math.sqrt((L1 + L2) ** 2 - Z_DRAW_SURFACE ** 2) * self.scale
        painter.setPen(QPen(QColor("#41a6b5"), 1, Qt.PenStyle.DashLine))
        painter.drawArc(int(cx - r_max), int(cy - r_max), int(r_max * 2), int(r_max * 2), 0, 180 * 16)

        # 5. Base Exclusion Zone
        base_pix_r = self.BASE_OBSTACLE_RADIUS * self.scale
        painter.setBrush(QBrush(QColor(247, 118, 142, 60)))
        painter.setPen(QPen(QColor("#f7768e"), 1.5, Qt.PenStyle.DashDotLine))
        painter.drawPie(int(cx - base_pix_r), int(cy - base_pix_r), int(base_pix_r * 2), int(base_pix_r * 2), 0, 180 * 16)

        painter.setPen(QPen(QColor("#f7768e")))
        painter.setFont(QFont("Segoe UI", 7, QFont.Weight.Bold))
        painter.drawText(int(cx - base_pix_r), int(cy - base_pix_r / 2), int(base_pix_r * 2), 20, Qt.AlignmentFlag.AlignCenter, "BASE (Ø 130mm)")

        # Base Origin
        painter.setBrush(QBrush(QColor("#f7768e")))
        painter.setPen(Qt.PenStyle.NoPen)
        painter.drawEllipse(QPointF(cx, cy), 5, 5)

        # 6. Connecting Paths
        if len(self.points) > 1:
            for i in range(len(self.points) - 1):
                p1 = self.points[i]
                p2 = self.points[i + 1]
                is_linear_segment = p2[3]

                segment_pen = QPen(
                    QColor("#7aa2f7") if is_linear_segment else QColor("#e0af68"),
                    2,
                    Qt.PenStyle.SolidLine if is_linear_segment else Qt.PenStyle.DashLine
                )
                painter.setPen(segment_pen)

                p1_scr_x = cx + p1[1] * self.scale
                p1_scr_y = cy - p1[0] * self.scale
                p2_scr_x = cx + p2[1] * self.scale
                p2_scr_y = cy - p2[0] * self.scale
                painter.drawLine(int(p1_scr_x), int(p1_scr_y), int(p2_scr_x), int(p2_scr_y))

        # 7. Badges
        for idx, (px, py, pz, is_lin) in enumerate(self.points):
            screen_x = cx + py * self.scale
            screen_y = cy - px * self.scale

            badge_color = QColor("#ff9e64") if is_lin else QColor("#2ac3de")
            painter.setBrush(QBrush(badge_color))
            painter.setPen(QPen(QColor("#ffffff"), 1.5))
            painter.drawEllipse(QPointF(screen_x, screen_y), 11, 11)

            painter.setPen(QPen(QColor("#1a1b26")))
            painter.setFont(QFont("Arial", 9, QFont.Weight.Bold))
            painter.drawText(int(screen_x - 7), int(screen_y - 7), 14, 14, Qt.AlignmentFlag.AlignCenter, str(idx + 1))

        # 8. Unhomed Mask
        if not self.is_homed:
            painter.setBrush(QBrush(QColor(15, 17, 26, 215)))
            painter.setPen(Qt.PenStyle.NoPen)
            painter.drawRoundedRect(self.rect(), 8, 8)
            painter.setPen(QPen(QColor("#ff757f")))
            painter.setFont(QFont("Segoe UI", 12, QFont.Weight.Bold))
            painter.drawText(self.rect(), Qt.AlignmentFlag.AlignCenter, "ARM NOT HOMED\nClick 'Start Homing' to Enable Grid")

    def mousePressEvent(self, event):
        if not self.is_homed:
            return
        if event.button() == Qt.MouseButton.LeftButton:
            w = self.width()
            h = self.height()
            cx = w / 2.0
            cy = h - 45.0

            raw_y_mm = (event.position().x() - cx) / self.scale
            raw_x_mm = (cy - event.position().y()) / self.scale

            snapped_x = round(raw_x_mm / self.GRID_STEP_MM) * self.GRID_STEP_MM
            snapped_y = round(raw_y_mm / self.GRID_STEP_MM) * self.GRID_STEP_MM

            if snapped_x < 0:
                return
            if math.sqrt(snapped_x * snapped_x + snapped_y * snapped_y) <= self.BASE_OBSTACLE_RADIUS:
                return
            if calculate_ik(snapped_x, snapped_y, Z_DRAW_SURFACE):
                self.point_clicked.emit(float(int(snapped_x)), float(int(snapped_y)), Z_DRAW_SURFACE)


# ----------------- SIDE-BY-SIDE KINEMATIC PREVIEW -----------------
class KinematicPreview(QFrame):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(420, 190)
        self.setStyleSheet("background-color: #1a1b26; border-radius: 8px;")
        self.current_ik = calculate_ik(0, 0, 220)

    def update_pose(self, ik_result):
        self.current_ik = ik_result
        self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)
        w = self.width()
        h = self.height()
        mid_x = w / 2.0

        # Vertical Divider Line
        painter.setPen(QPen(QColor("#24283b"), 1.5))
        painter.drawLine(int(mid_x), 10, int(mid_x), h - 10)

        # --- LEFT: TOP - XY ---
        painter.setFont(QFont("Segoe UI", 8, QFont.Weight.Bold))
        painter.setPen(QPen(QColor("#bb9af7")))
        painter.drawText(12, 20, "TOP - XY")

        top_cx = mid_x / 2.0
        top_cy = h - 35.0

        painter.setPen(QPen(QColor("#414868"), 1, Qt.PenStyle.DashLine))
        painter.drawArc(int(top_cx - 55), int(top_cy - 55), 110, 110, 0, 180 * 16)

        painter.setPen(QPen(QColor("#3b4261"), 1))
        painter.drawLine(int(top_cx - 65), int(top_cy), int(top_cx + 65), int(top_cy))

        if self.current_ik:
            rad_base = math.radians(self.current_ik["base"])
            end_x = top_cx + 55 * math.sin(rad_base)
            end_y = top_cy - 55 * math.cos(rad_base)

            painter.setPen(QPen(QColor("#7aa2f7"), 3))
            painter.drawLine(QPointF(top_cx, top_cy), QPointF(end_x, end_y))
            painter.setBrush(QBrush(QColor("#f7768e")))
            painter.setPen(Qt.PenStyle.NoPen)
            painter.drawEllipse(QPointF(top_cx, top_cy), 4, 4)

        # --- RIGHT: SIDE - Y ---
        painter.setFont(QFont("Segoe UI", 8, QFont.Weight.Bold))
        painter.setPen(QPen(QColor("#bb9af7")))
        painter.drawText(int(mid_x + 12), 20, "SIDE - Y")

        side_cx = mid_x + 35.0
        side_ground_y = h - 35.0
        scale = 0.50

        painter.setPen(QPen(QColor("#3b4261"), 1))
        painter.drawLine(int(mid_x + 10), int(side_ground_y), w - 10, int(side_ground_y))

        draw_line_y = side_ground_y - (Z_DRAW_SURFACE * scale)
        painter.setPen(QPen(QColor("#e0af68"), 1, Qt.PenStyle.DashLine))
        painter.drawLine(int(mid_x + 10), int(draw_line_y), w - 10, int(draw_line_y))

        if self.current_ik:
            sh_ang = math.radians(self.current_ik["shoulder"])
            el_ang = sh_ang + math.radians(self.current_ik["elbow"])

            j1_x = side_cx
            j1_y = side_ground_y
            j2_x = j1_x + (L1 * scale) * math.cos(sh_ang)
            j2_y = j1_y - (L1 * scale) * math.sin(sh_ang)
            j3_x = j2_x + (L2 * scale) * math.cos(el_ang)
            j3_y = j2_y - (L2 * scale) * math.sin(el_ang)

            painter.setPen(QPen(QColor("#9ece6a"), 4))
            painter.drawLine(QPointF(j1_x, j1_y), QPointF(j2_x, j2_y))
            painter.setPen(QPen(QColor("#2ac3de"), 4))
            painter.drawLine(QPointF(j2_x, j2_y), QPointF(j3_x, j3_y))

            painter.setPen(QPen(QColor("#f7768e"), 2))
            painter.drawLine(QPointF(j3_x, j3_y), QPointF(j3_x, j3_y + 10))

            painter.setBrush(QBrush(QColor("#c0caf5")))
            painter.setPen(Qt.PenStyle.NoPen)
            painter.drawEllipse(QPointF(j1_x, j1_y), 4, 4)
            painter.drawEllipse(QPointF(j2_x, j2_y), 4, 4)
            painter.drawEllipse(QPointF(j3_x, j3_y), 3, 3)


# ----------------- MAIN WINDOW -----------------
class ArmDashboard(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Robotic Arm Trajectory Controller")
        self.resize(960, 720)
        self.setStyleSheet("QMainWindow { background-color: #0f111a; } QLabel { color: #c0caf5; font-family: Segoe UI, Arial; }")

        self.queue = []
        self.stream_buffer = []
        self.is_executing = False
        self.current_hardware_pos = (0.0, 0.0, 220.0)

        self.init_ui()
        self.init_serial_thread()

    def init_ui(self):
        main_widget = QWidget()
        self.setCentralWidget(main_widget)
        main_layout = QHBoxLayout(main_widget)
        main_layout.setSpacing(15)

        # LEFT
        left_layout = QVBoxLayout()
        header_lbl = QLabel("Workspace Canvas (Z = 20 mm Plane, 10 mm Grid)")
        header_lbl.setFont(QFont("Segoe UI", 11, QFont.Weight.Bold))
        left_layout.addWidget(header_lbl)

        self.canvas = WorkspaceCanvas()
        self.canvas.point_clicked.connect(self.add_canvas_point)
        left_layout.addWidget(self.canvas)
        main_layout.addLayout(left_layout, stretch=3)

        # RIGHT
        right_layout = QVBoxLayout()

        # 1. Serial Port Box
        conn_group = QGroupBox("Hardware Serial Connection")
        conn_group.setStyleSheet("QGroupBox { color: #bb9af7; font-weight: bold; border: 1px solid #292e42; border-radius: 6px; margin-top: 10px; padding: 10px; }")
        cg_conn_layout = QHBoxLayout(conn_group)

        self.port_combo = QComboBox()
        self.port_combo.setStyleSheet("background-color: #1f2335; color: white; border: 1px solid #3b4261; border-radius: 4px; padding: 4px;")
        self.refresh_ports()
        cg_conn_layout.addWidget(self.port_combo, stretch=2)

        self.refresh_btn = QPushButton("↻")
        self.refresh_btn.setStyleSheet("background-color: #24283b; color: white; font-weight: bold; border-radius: 4px; padding: 4px 8px;")
        self.refresh_btn.clicked.connect(self.refresh_ports)
        cg_conn_layout.addWidget(self.refresh_btn)

        self.connect_btn = QPushButton("Connect")
        self.connect_btn.setStyleSheet("background-color: #3d59a1; color: white; font-weight: bold; border-radius: 4px; padding: 5px;")
        self.connect_btn.clicked.connect(self.toggle_connection)
        cg_conn_layout.addWidget(self.connect_btn, stretch=2)
        right_layout.addWidget(conn_group)

        # 2. Status Box
        state_group = QGroupBox("System State")
        state_group.setStyleSheet("QGroupBox { color: #bb9af7; font-weight: bold; border: 1px solid #292e42; border-radius: 6px; margin-top: 10px; padding: 10px; }")
        sg_layout = QVBoxLayout(state_group)

        self.status_lbl = QLabel("Status: DISCONNECTED")
        self.status_lbl.setStyleSheet("color: #ff757f; font-weight: bold;")
        sg_layout.addWidget(self.status_lbl)

        self.home_btn = QPushButton("Start Homing Sequence")
        self.home_btn.setStyleSheet("QPushButton { background-color: #3d59a1; color: white; border-radius: 5px; padding: 8px; font-weight: bold; } QPushButton:disabled { background-color: #24283b; color: #565f89; }")
        self.home_btn.setEnabled(False)
        self.home_btn.clicked.connect(self.start_homing)
        sg_layout.addWidget(self.home_btn)
        right_layout.addWidget(state_group)

        # 3. Kinematic Digital Twin
        kin_group = QGroupBox("Kinematic Digital Twin")
        kin_group.setStyleSheet("QGroupBox { color: #bb9af7; font-weight: bold; border: 1px solid #292e42; border-radius: 6px; margin-top: 10px; padding: 10px; }")
        kin_layout = QVBoxLayout(kin_group)
        self.preview = KinematicPreview()
        kin_layout.addWidget(self.preview)
        right_layout.addWidget(kin_group)

        # 4. Trajectory Mode
        traj_group = QGroupBox("Next Move Mode")
        traj_group.setStyleSheet("QGroupBox { color: #bb9af7; font-weight: bold; border: 1px solid #292e42; border-radius: 6px; margin-top: 10px; padding: 10px; }")
        traj_layout = QVBoxLayout(traj_group)

        self.linear_toggle = QCheckBox("Draw as Straight Line (Linear)")
        self.linear_toggle.setChecked(True)
        self.linear_toggle.setStyleSheet("QCheckBox { color: #7aa2f7; font-weight: bold; }")
        self.linear_toggle.toggled.connect(self.update_toggle_label)
        traj_layout.addWidget(self.linear_toggle)

        self.mode_desc_lbl = QLabel("Mode: Next points will draw a STRAIGHT line.")
        self.mode_desc_lbl.setStyleSheet("color: #7982a9; font-size: 8pt;")
        traj_layout.addWidget(self.mode_desc_lbl)
        right_layout.addWidget(traj_group)

        # 5. Manual Coordinates
        manual_group = QGroupBox("Manual Input")
        manual_group.setStyleSheet("QGroupBox { color: #bb9af7; font-weight: bold; border: 1px solid #292e42; border-radius: 6px; margin-top: 10px; padding: 10px; }")
        mg_layout = QGridLayout(manual_group)

        mg_layout.addWidget(QLabel("X:"), 0, 0)
        self.input_x = QLineEdit("0.0")
        mg_layout.addWidget(self.input_x, 0, 1)

        mg_layout.addWidget(QLabel("Y:"), 0, 2)
        self.input_y = QLineEdit("0.0")
        mg_layout.addWidget(self.input_y, 0, 3)

        mg_layout.addWidget(QLabel("Z:"), 0, 4)
        self.input_z = QLineEdit("220.0")
        mg_layout.addWidget(self.input_z, 0, 5)

        for box in (self.input_x, self.input_y, self.input_z):
            box.setStyleSheet("background-color: #1f2335; color: white; border: 1px solid #3b4261; border-radius: 4px; padding: 3px;")

        self.add_btn = QPushButton("Add Coordinate")
        self.add_btn.setStyleSheet("background-color: #24283b; color: #7aa2f7; border: 1px solid #3b4261; border-radius: 4px; padding: 5px;")
        self.add_btn.clicked.connect(self.add_manual_point)
        mg_layout.addWidget(self.add_btn, 1, 0, 1, 6)
        right_layout.addWidget(manual_group)

        # 6. Action Execution
        act_layout = QHBoxLayout()
        self.go_btn = QPushButton("GO (Execute)")
        self.go_btn.setStyleSheet("background-color: #73daca; color: #15161e; font-weight: bold; border-radius: 5px; padding: 10px;")
        self.go_btn.setEnabled(False)
        self.go_btn.clicked.connect(self.execute_queue)

        self.clear_btn = QPushButton("Clear Queue")
        self.clear_btn.setStyleSheet("background-color: #f7768e; color: #15161e; font-weight: bold; border-radius: 5px; padding: 10px;")
        self.clear_btn.clicked.connect(self.clear_queue)

        act_layout.addWidget(self.go_btn)
        act_layout.addWidget(self.clear_btn)
        right_layout.addLayout(act_layout)

        main_layout.addLayout(right_layout, stretch=2)

        # Communication Watchdog
        self.watchdog = QTimer(self)
        self.watchdog.setSingleShot(True)
        self.watchdog.timeout.connect(self.watchdog_timeout)

    # ----------------- SERIAL SETUP -----------------
    def init_serial_thread(self):
        self.thread = QThread()
        self.worker = SerialWorker()
        self.worker.moveToThread(self.thread)

        self.thread.started.connect(self.worker.run)
        self.worker.line_received.connect(self.handle_serial_data)
        self.worker.connection_status.connect(self.handle_connection_status)

        self.thread.start()

    def refresh_ports(self):
        self.port_combo.clear()
        ports = serial.tools.list_ports.comports()
        for p in ports:
            self.port_combo.addItem(f"{p.device}")

    def toggle_connection(self):
        if self.connect_btn.text() == "Connect":
            target_port = self.port_combo.currentText()
            if target_port:
                self.worker.connect_serial(target_port, 115200)
        else:
            self.worker.disconnect_serial()

    def handle_connection_status(self, connected, msg):
        if connected:
            self.status_lbl.setText("Status: CONNECTED (Unhomed)")
            self.status_lbl.setStyleSheet("color: #e0af68; font-weight: bold;")
            self.connect_btn.setText("Disconnect")
            self.connect_btn.setStyleSheet("background-color: #f7768e; color: #15161e; font-weight: bold; border-radius: 4px;")
            self.home_btn.setEnabled(True)
        else:
            self.status_lbl.setText("Status: DISCONNECTED")
            self.status_lbl.setStyleSheet("color: #ff757f; font-weight: bold;")
            self.connect_btn.setText("Connect")
            self.connect_btn.setStyleSheet("background-color: #3d59a1; color: white; font-weight: bold; border-radius: 4px;")
            self.home_btn.setEnabled(False)
            self.go_btn.setEnabled(False)
            self.canvas.set_homed(False)

    # ----------------- HANDSHAKE & EXECUTION -----------------
    def start_homing(self):
        self.status_lbl.setText("Status: HOMING IN PROGRESS...")
        self.status_lbl.setStyleSheet("color: #7aa2f7; font-weight: bold;")
        self.home_btn.setEnabled(False)
        self.worker.send_command("HOME")

    def handle_serial_data(self, line: str):
        if line == "HOMING_COMPLETE":
            self.status_lbl.setText("Status: HOMED AT (0, 0, 220)")
            self.status_lbl.setStyleSheet("color: #9ece6a; font-weight: bold;")
            self.canvas.set_homed(True)
            self.go_btn.setEnabled(True)
            self.current_hardware_pos = (0.0, 0.0, 220.0)
            self.preview.update_pose(calculate_ik(0, 0, 220))
            return

        if line == "POINT_REACHED":
            if self.is_executing:
                self.send_next_subpoint()
            return

        if "ERR_" in line:
            self.status_lbl.setText(f"Hardware Error: {line}")
            self.status_lbl.setStyleSheet("color: #f7768e; font-weight: bold;")
            self.is_executing = False

    def watchdog_timeout(self):
        if self.is_executing:
            print("Watchdog: Acknowledgment delayed, advancing stream...")
            self.send_next_subpoint()

    def update_toggle_label(self, checked):
        if checked:
            self.linear_toggle.setStyleSheet("QCheckBox { color: #7aa2f7; font-weight: bold; }")
            self.mode_desc_lbl.setText("Mode: Next points will draw a STRAIGHT line.")
        else:
            self.linear_toggle.setStyleSheet("QCheckBox { color: #e0af68; font-weight: bold; }")
            self.mode_desc_lbl.setText("Mode: Next points will be RAPID moves.")

    def add_canvas_point(self, x, y, z):
        is_linear = self.linear_toggle.isChecked()
        self.queue.append((x, y, z, is_linear))
        self.canvas.set_points(self.queue)
        ik = calculate_ik(x, y, z)
        if ik:
            self.preview.update_pose(ik)

    def add_manual_point(self):
        try:
            x = float(self.input_x.text())
            y = float(self.input_y.text())
            z = float(self.input_z.text())
            ik = calculate_ik(x, y, z)
            if ik:
                is_linear = self.linear_toggle.isChecked()
                self.queue.append((x, y, z, is_linear))
                self.canvas.set_points(self.queue)
                self.preview.update_pose(ik)
                self.status_lbl.setText("Status: Manual point queued.")
                self.status_lbl.setStyleSheet("color: #9ece6a;")
            else:
                self.status_lbl.setText("Status: Target Out of Reach!")
                self.status_lbl.setStyleSheet("color: #f7768e;")
        except ValueError:
            pass

    def clear_queue(self):
        self.queue.clear()
        self.stream_buffer.clear()
        self.canvas.set_points(self.queue)

    def execute_queue(self):
        if not self.queue or self.is_executing:
            return

        self.stream_buffer = []
        last_pt = self.current_hardware_pos

        for (target_x, target_y, target_z, is_linear) in self.queue:
            if is_linear:
                dist = math.sqrt((target_x - last_pt[0])**2 + (target_y - last_pt[1])**2 + (target_z - last_pt[2])**2)
                steps = max(1, int(dist / SEGMENT_RESOLUTION))

                for s in range(1, steps + 1):
                    interp_x = last_pt[0] + (target_x - last_pt[0]) * (s / steps)
                    interp_y = last_pt[1] + (target_y - last_pt[1]) * (s / steps)
                    interp_z = last_pt[2] + (target_z - last_pt[2]) * (s / steps)
                    self.stream_buffer.append((round(interp_x, 2), round(interp_y, 2), round(interp_z, 2)))
            else:
                self.stream_buffer.append((target_x, target_y, target_z))

            last_pt = (target_x, target_y, target_z)

        self.is_executing = True
        self.status_lbl.setText(f"Status: EXECUTING ({len(self.stream_buffer)} points)")
        self.status_lbl.setStyleSheet("color: #7aa2f7; font-weight: bold;")
        self.send_next_subpoint()

    def send_next_subpoint(self):
        self.watchdog.stop()
        if self.stream_buffer:
            next_pt = self.stream_buffer.pop(0)
            self.current_hardware_pos = next_pt
            cmd = f"GOTO,{next_pt[0]},{next_pt[1]},{next_pt[2]}"
            self.worker.send_command(cmd)
            self.watchdog.start(3000)  # 3.0s recovery timeout
        else:
            self.is_executing = False
            self.status_lbl.setText("Status: QUEUE COMPLETE")
            self.status_lbl.setStyleSheet("color: #9ece6a; font-weight: bold;")
            self.clear_queue()

    def closeEvent(self, event):
        self.worker.is_running = False
        self.worker.disconnect_serial()
        self.thread.quit()
        self.thread.wait()
        event.accept()


if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = ArmDashboard()
    window.show()
    sys.exit(app.exec())