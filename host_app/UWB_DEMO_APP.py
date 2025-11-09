import sys
import json
import time
import numpy as np
from dataclasses import dataclass, field
from PyQt6 import QtWidgets, QtCore, QtGui
import pyqtgraph as pg
from pyqtgraph.GraphicsScene.mouseEvents import MouseDragEvent

from SerialWorker import SerialWorker
from UWBController import UWBController
from TrilaterationSolver3D import TrilaterationSolver3D
from UWBKalmanFilter import UWBKalmanFilter

CONFIG_FILE = "config.json"
PATH_HISTORY_LIMIT = 100


class DraggableScatterPlotItem(pg.ScatterPlotItem):
    """Scatter plot item that allows dragging individual points."""

    pointDragged = QtCore.pyqtSignal(int, QtCore.QPointF)

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._drag_index = None
        self._drag_offset = QtCore.QPointF()

    def mouseDragEvent(self, ev: MouseDragEvent):
        if ev.button() != QtCore.Qt.MouseButton.LeftButton:
            ev.ignore()
            return

        if ev.isStart():
            self._drag_index = None
            pts = self.pointsAt(ev.buttonDownPos())
            if pts is None or len(pts) == 0:
                ev.ignore()
                return

            idx = pts[0].data()
            if idx is None:
                ev.ignore()
                return

            self._drag_index = idx
            self._drag_offset = pts[0].pos() - ev.pos()
            ev.accept()
            return

        if self._drag_index is None:
            ev.ignore()
            return

        ev.accept()
        new_pos = ev.pos() + self._drag_offset
        self.pointDragged.emit(self._drag_index, new_pos)

        if ev.isFinish():
            self._drag_index = None


@dataclass
class Anchor:
    id: int
    pos: list[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    enable: bool = True
    label_item: pg.TextItem | None = None

class AnchorManager:
    """Centralized collection for anchor metadata and coordinates."""

    def __init__(self):
        self._anchors: list[Anchor] = []

    @staticmethod
    def _normalize_pos(pos_values):
        raw_values = list(pos_values)
        normalized = []
        for value in raw_values:
            try:
                normalized.append(float(value))
            except (TypeError, ValueError):
                normalized.append(0.0)
            if len(normalized) == 3:
                break
        while len(normalized) < 3:
            normalized.append(0.0)
        return normalized

    def __len__(self):
        return len(self._anchors)

    def __iter__(self):
        return iter(self._anchors)

    def __getitem__(self, idx: int) -> Anchor:
        return self._anchors[idx]

    def add_anchor(self, anchor_id: int | None = None, pos: list[float] | None = None, enable: bool = True):
        if pos is None:
            pos = [0.0, 0.0, 0.0]
        if anchor_id is None:
            anchor_id = 0xFF00 + len(self._anchors)
        anchor = Anchor(id=anchor_id, pos=self._normalize_pos(pos), enable=enable)
        self._anchors.append(anchor)
        return anchor

    def remove(self, idx: int):
        if 0 <= idx < len(self._anchors):
            del self._anchors[idx]

    def toggle_enable(self, idx: int):
        if 0 <= idx < len(self._anchors):
            self._anchors[idx].enable = not self._anchors[idx].enable

    def active_indices(self):
        return [i for i, anchor in enumerate(self._anchors) if anchor.enable]

    def ids_for_indices(self, indices):
        return [self._anchors[i].id for i in indices if 0 <= i < len(self._anchors)]

    def active_positions(self):
        return [anchor.pos for anchor in self._anchors if anchor.enable]

    def active_ids(self):
        return [anchor.id for anchor in self._anchors if anchor.enable]

    def to_config(self):
        return [{"id": anchor.id, "pos": anchor.pos, "enable": anchor.enable} for anchor in self._anchors]

    def load_config(self, anchors_cfg):
        self._anchors = []
        for entry in anchors_cfg:
            pos = self._normalize_pos(entry.get("pos", [0.0, 0.0, 0.0]))
            self._anchors.append(Anchor(id=entry["id"], pos=pos, enable=entry.get("enable", True)))


@dataclass
class Tag:
    id: int
    z: float = 1.0
    enable: bool = True
    filter: UWBKalmanFilter = field(default_factory=lambda: UWBKalmanFilter(dt=0.2, process_var=1e-3, measurement_var=1e-1))
    last_seen: float = 0.0
    plot_item: pg.ScatterPlotItem | None = None
    path: list[tuple[float, float]] = field(default_factory=list)
    path_item: pg.PlotDataItem | None = None

class TagManager:
    """Centralized collection for tag metadata."""

    def __init__(self):
        self._tags: list[Tag] = []

    @staticmethod
    def _to_float(value, default=0.0):
        try:
            return float(value)
        except (TypeError, ValueError):
            return default

    def __len__(self):
        return len(self._tags)

    def __iter__(self):
        return iter(self._tags)

    def __getitem__(self, idx: int) -> Tag:
        return self._tags[idx]

    def add_tag(self, tag_id: int | None = None, z: float = 1.0, enable: bool = True):
        if tag_id is None:
            tag_id = 0x0000 + len(self._tags)
        tag = Tag(id=tag_id, z=self._to_float(z, 1.0), enable=enable)
        self._tags.append(tag)
        return tag

    def remove(self, idx: int):
        if 0 <= idx < len(self._tags):
            del self._tags[idx]

    def toggle_enable(self, idx: int):
        if 0 <= idx < len(self._tags):
            self._tags[idx].enable = not self._tags[idx].enable

    def set_enable(self, idx: int, enabled: bool):
        if 0 <= idx < len(self._tags):
            self._tags[idx].enable = enabled

    def active_tags(self):
        return [tag for tag in self._tags if tag.enable]

    def ids(self):
        return [tag.id for tag in self._tags]

    def to_config(self):
        return [{"id": tag.id, "z": tag.z, "enable": tag.enable} for tag in self._tags]

    def load_config(self, tags_cfg):
        self._tags = [
            Tag(id=entry["id"], z=self._to_float(entry.get("z", 0.0)), enable=entry.get("enable", True))
            for entry in tags_cfg
        ]


class UWBMainWindow(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("UWB 定位系統 (PyQt6)")
        self.resize(1500, 900)

        # 狀態變數
        self.is_running = False

        # UWB 資料
        self.anchor_manager = AnchorManager()
        self.tag_manager = TagManager()
        self._calibrating = False

        # 模組
        self.worker = None
        self.controller = None

        # 更新計時器
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update_position)

        # UI 初始化
        self._init_ui()
        self.load_config(auto=True)

    # ----------------------------------------------------------------------
    def _init_ui(self):
        main_widget = QtWidgets.QWidget()
        self.setCentralWidget(main_widget)
        layout = QtWidgets.QHBoxLayout(main_widget)

        font = QtGui.QFont()
        font.setPointSize(11)
        self.setFont(font)

        # 左側控制面板
        control_panel = QtWidgets.QFrame()
        control_layout = QtWidgets.QVBoxLayout(control_panel)

        # === 系統控制區 ===
        system_group = QtWidgets.QGroupBox("系統控制")
        sys_layout = QtWidgets.QHBoxLayout(system_group)

        self.btn_start = QtWidgets.QPushButton("▶️ Start")
        self.btn_start.clicked.connect(self.toggle_start)
        self.btn_save = QtWidgets.QPushButton("💾 Save")
        self.btn_save.clicked.connect(self.save_config)
        self.btn_load = QtWidgets.QPushButton("📂 Load")
        self.btn_load.clicked.connect(self.load_config_dialog)
        self.btn_clear_paths = QtWidgets.QPushButton("🧹 Clear Paths")
        self.btn_clear_paths.clicked.connect(self.clear_tag_paths)

        sys_layout.addWidget(self.btn_start)
        sys_layout.addWidget(self.btn_save)
        sys_layout.addWidget(self.btn_load)
        sys_layout.addWidget(self.btn_clear_paths)
        control_layout.addWidget(system_group)

        # === 串口設定 ===
        conn_group = QtWidgets.QGroupBox("連線設定")
        conn_layout = QtWidgets.QHBoxLayout(conn_group)
        conn_layout.addWidget(QtWidgets.QLabel("COM:"))
        self.com_edit = QtWidgets.QLineEdit("COM18")
        self.com_edit.setFixedWidth(100)
        conn_layout.addWidget(self.com_edit)

        conn_layout.addWidget(QtWidgets.QLabel("Baud:"))
        self.baud_edit = QtWidgets.QLineEdit("115200")
        self.baud_edit.setFixedWidth(100)
        conn_layout.addWidget(self.baud_edit)

        self.btn_connect = QtWidgets.QPushButton("🔌 Connect")
        self.btn_connect.clicked.connect(self.connect_serial)
        self.btn_disconnect = QtWidgets.QPushButton("❌ Disconnect")
        self.btn_disconnect.clicked.connect(self.disconnect_serial)
        conn_layout.addWidget(self.btn_connect)
        conn_layout.addWidget(self.btn_disconnect)
        control_layout.addWidget(conn_group)

        # === 手動 Ping / 電壓 ===
        ping_group = QtWidgets.QGroupBox("⚡ Ping / Voltage")
        ping_layout = QtWidgets.QHBoxLayout(ping_group)
        ping_layout.addWidget(QtWidgets.QLabel("Node ID:"))
        self.ping_id_edit = QtWidgets.QLineEdit("0xFF00")
        self.ping_id_edit.setFixedWidth(100)
        ping_layout.addWidget(self.ping_id_edit)
        self.btn_ping = QtWidgets.QPushButton("Ping")
        self.btn_ping.clicked.connect(self.ping_single)
        ping_layout.addWidget(self.btn_ping)
        self.ping_result_label = QtWidgets.QLabel("尚未測試")
        self.ping_result_label.setMinimumWidth(200)
        ping_layout.addWidget(self.ping_result_label, 1)
        control_layout.addWidget(ping_group)

        # === Anchors ===
        anchor_group = QtWidgets.QGroupBox("📍 Anchors")
        anchor_layout = QtWidgets.QVBoxLayout(anchor_group)
        self.anchor_table = self._create_table(["✓", "ID", "X", "Y", "Z"])
        anchor_layout.addWidget(self.anchor_table)

        anchor_btns = QtWidgets.QHBoxLayout()
        btn_add_anchor = QtWidgets.QPushButton("➕ Add")
        btn_del_anchor = QtWidgets.QPushButton("➖ Remove")
        btn_add_anchor.clicked.connect(self.add_anchor)
        btn_del_anchor.clicked.connect(self.remove_anchor)
        anchor_btns.addWidget(btn_add_anchor)
        anchor_btns.addWidget(btn_del_anchor)
        anchor_layout.addLayout(anchor_btns)
        control_layout.addWidget(anchor_group)

        # === Tags ===
        tag_group = QtWidgets.QGroupBox("🎯 Tags")
        tag_layout = QtWidgets.QVBoxLayout(tag_group)
        self.tag_table = self._create_table(["✓", "ID", "Z"])
        tag_layout.addWidget(self.tag_table)

        tag_btns = QtWidgets.QHBoxLayout()
        btn_add_tag = QtWidgets.QPushButton("➕ Add")
        btn_del_tag = QtWidgets.QPushButton("➖ Remove")
        btn_add_tag.clicked.connect(self.add_tag)
        btn_del_tag.clicked.connect(self.remove_tag)
        tag_btns.addWidget(btn_add_tag)
        tag_btns.addWidget(btn_del_tag)
        tag_layout.addLayout(tag_btns)
        control_layout.addWidget(tag_group)

        # === RSSI Monitor ===
        rssi_group = QtWidgets.QGroupBox("📶 RSSI Monitor")
        rssi_layout = QtWidgets.QVBoxLayout(rssi_group)
        self.rssi_table = self._create_table(["Tag", "Anchor", "距離 (m)", "RSSI (dBm)"])
        self.rssi_table.setEditTriggers(QtWidgets.QAbstractItemView.EditTrigger.NoEditTriggers)
        rssi_layout.addWidget(self.rssi_table)
        control_layout.addWidget(rssi_group)

        # === Calibration ===
        calib_group = QtWidgets.QGroupBox("🔧 Anchor Calibration")
        calib_layout = QtWidgets.QVBoxLayout(calib_group)
        calib_desc = QtWidgets.QLabel("設定並啟用 Anchor ID 後可自動兩兩測距，推算 Anchor 的相對座標。")
        calib_desc.setWordWrap(True)
        self.btn_calibrate = QtWidgets.QPushButton("Start Calibration")
        self.btn_calibrate.clicked.connect(self.start_anchor_calibration)
        calib_layout.addWidget(calib_desc)
        calib_layout.addWidget(self.btn_calibrate)
        control_layout.addWidget(calib_group)

        control_layout.addStretch()

        # === 地圖區 ===
        plot_frame = QtWidgets.QFrame()
        plot_layout = QtWidgets.QVBoxLayout(plot_frame)
        self.plot_widget: pg.PlotWidget = pg.PlotWidget(background="w")
        plot_layout.addWidget(self.plot_widget)
        self.plot_widget.setAspectLocked(True)
        self.plot_widget.showGrid(x=True, y=True)
        self.plot_widget.setLabel("bottom", "X (m)")
        self.plot_widget.setLabel("left", "Y (m)")
        self.plot_widget.setTitle("UWB 即時定位 (Top View)", color="black", size="14pt")

        # 繪圖項
        self.anchor_scatter = DraggableScatterPlotItem(symbol="t", size=18, brush="r")
        self.anchor_scatter.pointDragged.connect(self.on_anchor_dragged)
        self.plot_widget.addItem(self.anchor_scatter)

        layout.addWidget(control_panel, 1)
        layout.addWidget(plot_frame, 3)

        # 綁定表格事件
        self.anchor_table.itemChanged.connect(self.on_anchor_item_changed)
        self.tag_table.itemChanged.connect(self.on_tag_item_changed)
        self._update_connection_buttons()

    # ----------------------------------------------------------------------
    def _update_connection_buttons(self):
        connected = self.worker is not None and self.controller is not None
        self.btn_connect.setEnabled(not connected)
        self.btn_disconnect.setEnabled(connected)
        if hasattr(self, "btn_calibrate"):
            self.btn_calibrate.setEnabled(connected and not self._calibrating)

    def _color_for_tag(self, tag_id):
        tag_ids = [tag.id for tag in self.tag_manager]
        idx = tag_ids.index(tag_id) if tag_id in tag_ids else len(tag_ids)
        return pg.intColor(idx, hues=6)
        
    # ----------------------------------------------------------------------
    @staticmethod
    def _format_coord(value):
        try:
            return f"{float(value):.2f}"
        except (TypeError, ValueError):
            return str(value)

    def _create_table(self, headers):
        table = QtWidgets.QTableWidget()
        table.setColumnCount(len(headers))
        table.setHorizontalHeaderLabels(headers)
        table.verticalHeader().setVisible(False)
        table.horizontalHeader().setStretchLastSection(True)
        table.setSelectionBehavior(QtWidgets.QAbstractItemView.SelectionBehavior.SelectRows)
        return table

    def _update_rssi_table(self, rows: list[tuple[str, str, str, str]]):
        """Render the latest RSSI readings."""
        self.rssi_table.setRowCount(len(rows))
        readonly_flags = QtCore.Qt.ItemFlag.ItemIsEnabled | QtCore.Qt.ItemFlag.ItemIsSelectable
        for row_idx, row in enumerate(rows):
            for col_idx, text in enumerate(row):
                item = QtWidgets.QTableWidgetItem(text)
                item.setFlags(readonly_flags)
                self.rssi_table.setItem(row_idx, col_idx, item)

    def _append_tag_path_point(self, tag: Tag, x: float, y: float):
        """Maintain and redraw a tag's trajectory line."""
        tag.path.append((x, y))
        if len(tag.path) > PATH_HISTORY_LIMIT:
            tag.path = tag.path[-PATH_HISTORY_LIMIT:]
        if tag.path_item is None:
            color = self._color_for_tag(tag.id)
            pen = pg.mkPen(color=color, width=2)
            tag.path_item = self.plot_widget.plot([], [], pen=pen)
        xs, ys = zip(*tag.path) if tag.path else ([], [])
        tag.path_item.setData(xs, ys)

    # ----------------------------------------------------------------------
    def connect_serial(self):
        if self.worker:
            QtWidgets.QMessageBox.information(self, "提示", "已經連線中。")
            return
        try:
            port = self.com_edit.text().strip()
            baud = int(self.baud_edit.text().strip())
            self.worker = SerialWorker(port=port, baudrate=baud, debug_print=False)
            self.worker.open()
            self.controller = UWBController(serial_worker=self.worker, debug_print=False)
            self.statusBar().showMessage(f"✅ Connected to {port} @ {baud}")
            self._update_connection_buttons()
        except Exception as e:
            self.worker = None
            self.controller = None
            self._update_connection_buttons()
            QtWidgets.QMessageBox.critical(self, "Error", f"Failed to open port: {e}")

    def disconnect_serial(self):
        if self.worker:
            self.worker.close()
            self.worker = None
            self.controller = None
            self.statusBar().showMessage("❌ Disconnected")
        else:
            self.statusBar().showMessage("⚠️ 尚未連線")
        self._update_connection_buttons()

    def clear_tag_paths(self):
        for tag in self.tag_manager:
            tag.path.clear()
            if tag.path_item:
                tag.path_item.setData([], [])
        self.statusBar().showMessage("🧹 已清除所有 Tag 軌跡")

    def ping_single(self):
        if not self.controller:
            QtWidgets.QMessageBox.warning(self, "提示", "請先連線後再進行 ping。")
            return

        text = self.ping_id_edit.text().strip()
        if not text:
            QtWidgets.QMessageBox.warning(self, "提示", "請輸入目標 Node ID。")
            return

        try:
            node_id = int(text, 16) if text.lower().startswith("0x") else int(text)
        except ValueError:
            QtWidgets.QMessageBox.warning(self, "格式錯誤", "Node ID 請輸入 16 進位 (0xFFFF) 或 10 進位數字。")
            return

        try:
            resp = self.controller.ping(node_id, timeout=0.2)
        except Exception as exc:
            self.ping_result_label.setText("⚠️ 讀取失敗")
            self.statusBar().showMessage(f"⚠️ ping 失敗: {exc}")
            return

        if resp:
            voltage_v = resp.voltage_mv / 1000.0
            self.ping_result_label.setText(f"ID 0x{resp.node_id:04X} → {voltage_v:.2f} V (state {resp.system_state})")
            self.statusBar().showMessage(f"✅ Ping 0x{resp.node_id:04X}: {voltage_v:.2f} V")
        else:
            self.ping_result_label.setText("⚠️ 無回應")
            self.statusBar().showMessage("⚠️ ping timeout")

    # ----------------------------------------------------------------------
    def toggle_start(self):
        if not self.controller:
            QtWidgets.QMessageBox.warning(self, "錯誤", "請先連線！")
            return
        self.is_running = not self.is_running
        self.btn_start.setText("⏸️ Stop" if self.is_running else "▶️ Start")
        if self.is_running:
            # self._reset_tag_markers()
            self.timer.start(200)
        else:
            self.timer.stop()

    # ----------------------------------------------------------------------
    def add_anchor(self):
        self.anchor_manager.add_anchor()
        self.update_tables()

    def remove_anchor(self):
        row = self.anchor_table.currentRow()
        if row >= 0:
            anchor = self.anchor_manager[row]
            if anchor.label_item:
                self.plot_widget.removeItem(anchor.label_item)
                
            self.anchor_manager.remove(row)
            self.update_tables()

    def add_tag(self):
        self.tag_manager.add_tag()
        self.update_tables()

    def remove_tag(self):
        row = self.tag_table.currentRow()
        if row >= 0:
            tag = self.tag_manager[row]
            if tag.plot_item:
                tag.plot_item.setData([], [])
            if tag.path_item:
                self.plot_widget.removeItem(tag.path_item)
                tag.path_item = None
            self.tag_manager.remove(row)
            self.update_tables()

    # ----------------------------------------------------------------------
    def update_tables(self):
        """更新 Anchor/Tag 表格 + 地圖"""
        self.anchor_table.blockSignals(True)
        self.anchor_table.setRowCount(len(self.anchor_manager))
        for i, anchor in enumerate(self.anchor_manager):
            x, y, z = anchor.pos
            chk = QtWidgets.QCheckBox()
            chk.setChecked(anchor.enable)
            chk.stateChanged.connect(lambda _, idx=i: self.toggle_anchor_enable(idx))
            self.anchor_table.setCellWidget(i, 0, chk)
            self.anchor_table.setItem(i, 1, QtWidgets.QTableWidgetItem(f"0x{anchor.id:04X}"))
            self.anchor_table.setItem(i, 2, QtWidgets.QTableWidgetItem(self._format_coord(x)))
            self.anchor_table.setItem(i, 3, QtWidgets.QTableWidgetItem(self._format_coord(y)))
            self.anchor_table.setItem(i, 4, QtWidgets.QTableWidgetItem(self._format_coord(z)))
        self.anchor_table.blockSignals(False)

        self.tag_table.blockSignals(True)
        self.tag_table.setRowCount(len(self.tag_manager))
        for i, tag in enumerate(self.tag_manager):
            chk = QtWidgets.QCheckBox()
            chk.setChecked(tag.enable)
            chk.stateChanged.connect(lambda _, idx=i: self.toggle_tag_enable(idx))
            self.tag_table.setCellWidget(i, 0, chk)
            self.tag_table.setItem(i, 1, QtWidgets.QTableWidgetItem(f"0x{tag.id:04X}"))
            self.tag_table.setItem(i, 2, QtWidgets.QTableWidgetItem(self._format_coord(tag.z)))
        self.tag_table.blockSignals(False)

        self._update_anchor_scatter()
        # clear the tag plot items
        for tag in self.tag_manager:
            if tag.plot_item:
                tag.plot_item.setData([], [])
                tag.last_seen = 0.0
        self._update_rssi_table([])

    def _update_anchor_scatter(self):
        """Refresh anchor scatter plot with index mapping for drag updates."""
        # - anchor 表格修改
        # - 拖曳座標點
        # - Anchor 校正結果
        xs, ys, data = [], [], []
        for idx, anchor in enumerate(self.anchor_manager):
            if not anchor.enable:
                continue
            xs.append(anchor.pos[0])
            ys.append(anchor.pos[1])
            data.append(idx)

        if xs:
            self.anchor_scatter.setData(x=xs, y=ys, data=data)
        else:
            self.anchor_scatter.setData(x=[], y=[], data=[])
        
        """Draw anchor IDs near their markers."""
        # remove existing labels        
        for anchor in self.anchor_manager:
            if anchor.label_item:
                self.plot_widget.removeItem(anchor.label_item)
                anchor.label_item = None

        for anchor in self.anchor_manager:
            if not anchor.enable:
                continue
            label = pg.TextItem(text=f"{anchor.id:04X}", color="darkred", anchor=(0.5, 1.5))
            label.setPos(anchor.pos[0], anchor.pos[1])
            self.plot_widget.addItem(label)
            anchor.label_item = label
        

    def _update_anchor_table_row(self, idx):
        """Update a single anchor row after interactive edits."""
        if not (0 <= idx < self.anchor_table.rowCount()) or not (0 <= idx < len(self.anchor_manager)):
            return

        anchor = self.anchor_manager[idx]
        self.anchor_table.blockSignals(True)
        for axis, col in enumerate((2, 3, 4)):
            item = self.anchor_table.item(idx, col)
            if item is None:
                item = QtWidgets.QTableWidgetItem()
                self.anchor_table.setItem(idx, col, item)
            item.setText(self._format_coord(anchor.pos[axis]))
        self.anchor_table.blockSignals(False)


    # --------------------- 表格編輯事件處理 ---------------------
    def on_anchor_item_changed(self, item: QtWidgets.QTableWidgetItem):
        r, c = item.row(), item.column()
        try:
            if not (0 <= r < len(self.anchor_manager)):
                return
            anchor = self.anchor_manager[r]
            if c == 1:
                anchor.id = int(item.text(), 16)
                self.anchor_table.blockSignals(True)
                item.setText(f"0x{anchor.id:04X}")
                self.anchor_table.blockSignals(False)
            elif c in [2, 3, 4]: # 2:X, 3:Y, 4:Z
                anchor.pos[c - 2] = float(item.text())
                self._update_anchor_table_row(r)
                self._update_anchor_scatter()
        except ValueError:
            pass
    
    def on_tag_item_changed(self, item: QtWidgets.QTableWidgetItem):
        r, c = item.row(), item.column()
        try:
            if not (0 <= r < len(self.tag_manager)):
                return
            tag = self.tag_manager[r]
            if c == 1: # ID
                tag.id = int(item.text(), 16)
            elif c == 2: # Z
                tag.z = float(item.text())
        except ValueError:
            pass

    def toggle_anchor_enable(self, idx):
        self.anchor_manager.toggle_enable(idx)
        self.update_tables()

    def toggle_tag_enable(self, idx):
        self.tag_manager.toggle_enable(idx)
        self.update_tables()

        tag = self.tag_manager[idx]
        item = tag.plot_item
        if item is None:
            color = self._color_for_tag(tag.id)
            dot = self.plot_widget.plot([], [], pen=None, symbol="o", symbolSize=12)
            dot.setSymbolBrush(color)
            tag.plot_item = dot
        else:
            item.setData([], [])
        if not tag.enable:
            if tag.path_item:
                tag.path_item.setData([], [])
        elif tag.path_item and tag.path:
            xs, ys = zip(*tag.path)
            tag.path_item.setData(xs, ys)

    # --------------------- 地圖 ---------------------------
    def on_anchor_dragged(self, idx, pos):
        """拖曳 anchor 更新座標"""
        if not (0 <= idx < len(self.anchor_manager)):
            return

        anchor = self.anchor_manager[idx]
        anchor.pos[0] = pos.x()
        anchor.pos[1] = pos.y()
        self._update_anchor_table_row(idx)
        self._update_anchor_scatter()

    # --------------------- Anchor 校正 ---------------------
    def start_anchor_calibration(self):
        if not self.controller:
            QtWidgets.QMessageBox.warning(self, "提示", "請先連線後再進行校正。")
            return
        active_indices = self.anchor_manager.active_indices()
        if len(active_indices) < 2:
            QtWidgets.QMessageBox.warning(self, "提示", "至少需要兩個啟用中的 Anchor 才能校正。")
            return

        if self._calibrating:
            return

        self._calibrating = True
        self._update_connection_buttons()
        QtWidgets.QApplication.setOverrideCursor(QtCore.Qt.CursorShape.WaitCursor)
        try:
            dist_matrix = self._measure_anchor_distances(active_indices)
            coords = self._compute_anchor_layout(dist_matrix)
            if not coords or len(coords) != len(active_indices):
                raise RuntimeError("無法推算座標，請確認量測結果。")

            for idx, (x, y) in zip(active_indices, coords):
                anchor = self.anchor_manager[idx]
                anchor.pos[0] = float(x)
                anchor.pos[1] = float(y)

            self.update_tables()
            self.statusBar().showMessage("✅ Anchor calibration 完成")
        except Exception as exc:
            QtWidgets.QMessageBox.critical(self, "Calibration Failed", str(exc))
        finally:
            self._calibrating = False
            self._update_connection_buttons()
            QtWidgets.QApplication.restoreOverrideCursor()

    def _measure_anchor_distances(self, anchor_indices):
        anchor_ids = self.anchor_manager.ids_for_indices(anchor_indices)
        n = len(anchor_ids)
        matrix = np.zeros((n, n), dtype=float)
        for i in range(n):
            matrix[i, i] = 0.0
            for j in range(i + 1, n):
                QtWidgets.QApplication.processEvents()
                self.statusBar().showMessage(f"🔧 測距 Anchor {anchor_ids[i]:04X} ↔ {anchor_ids[j]:04X}")
                dist = self._range_between(anchor_ids[i], anchor_ids[j])
                if dist is None:
                    raise RuntimeError(f"Anchor {anchor_ids[i]:04X} ↔ {anchor_ids[j]:04X} 測距失敗")
                matrix[i, j] = matrix[j, i] = dist
        return matrix

    def _range_between(self, aid_a, aid_b, attempts=10):
        if not self.controller:
            return None
        samples = []
        for _ in range(attempts):
            for initiator, responder in ((aid_a, aid_b), (aid_b, aid_a)):
                report = self.controller.trigger(initiator, responder)
                if report and report.distance_m > 0:
                    samples.append(report.distance_m)
                    break
            QtWidgets.QApplication.processEvents()
        if samples:
            return float(np.mean(samples))
        return None

    def _compute_anchor_layout(self, dist_matrix: np.ndarray):
        n = dist_matrix.shape[0]
        if n == 0:
            return []
        if n == 1:
            return [(0.0, 0.0)]

        d2 = dist_matrix ** 2
        j = np.eye(n) - np.ones((n, n)) / n
        b = -0.5 * j @ d2 @ j
        eigvals, eigvecs = np.linalg.eigh(b)
        idx = np.argsort(eigvals)[::-1]
        eigvals = eigvals[idx]
        eigvecs = eigvecs[:, idx]
        positive = eigvals[eigvals > 1e-6]
        if positive.size == 0:
            return None
        dims = min(2, positive.size)
        lambda_sqrt = np.sqrt(np.clip(eigvals[:dims], 0, None))
        coords = eigvecs[:, :dims] * lambda_sqrt
        if dims == 1:
            coords = np.hstack([coords, np.zeros((n, 1))])

        coords -= coords[0]
        if n > 1:
            vec = coords[1]
            angle = np.arctan2(vec[1], vec[0]) if np.linalg.norm(vec) > 1e-8 else 0.0
            rot = np.array([
                [np.cos(-angle), -np.sin(-angle)],
                [np.sin(-angle),  np.cos(-angle)]
            ])
            coords[:, :2] = coords[:, :2] @ rot.T
        if n > 2 and coords[2, 1] < 0:
            coords[:, 1] *= -1

        return [(float(coords[i, 0]), float(coords[i, 1])) for i in range(n)]

    
    # --------------------- 配置檔讀寫 ---------------------
    def save_config(self):
        cfg = {
            "anchors": self.anchor_manager.to_config(),
            "tags": self.tag_manager.to_config(),
        }

        # 自動儲存 config.json
        with open(CONFIG_FILE, "w", encoding="utf-8") as f:
            json.dump(cfg, f, indent=4)

        # 使用者指定
        fname, _ = QtWidgets.QFileDialog.getSaveFileName(self, "另存設定", "", "JSON Files (*.json)")
        if fname:
            with open(fname, "w", encoding="utf-8") as f:
                json.dump(cfg, f, indent=4)

        self.statusBar().showMessage("✅ 設定已儲存")

    def load_config(self, auto=False):
        try:
            with open(CONFIG_FILE, "r", encoding="utf-8") as f:
                cfg = json.load(f)
            self._apply_config(cfg)
            if auto:
                self.statusBar().showMessage("📂 自動載入 config.json")
        except FileNotFoundError:
            if not auto:
                QtWidgets.QMessageBox.warning(self, "設定檔不存在", "未找到 config.json")

    def load_config_dialog(self):
        fname, _ = QtWidgets.QFileDialog.getOpenFileName(self, "載入設定", "", "JSON Files (*.json)")
        if fname:
            with open(fname, "r", encoding="utf-8") as f:
                cfg = json.load(f)
            self._apply_config(cfg)
            self.statusBar().showMessage(f"📂 已載入 {fname}")

    def _apply_config(self, cfg):
        # clear all tag plot items
        for tag in self.tag_manager:
            if tag.plot_item:
                tag.plot_item.setData([], [])
                tag.last_seen = 0.0
        # clear all anchor labels
        for anchor in self.anchor_manager:
            if anchor.label_item:
                self.plot_widget.removeItem(anchor.label_item)
                anchor.label_item = None
        self.anchor_manager.load_config(cfg.get("anchors", []))
        self.tag_manager.load_config(cfg.get("tags", []))
        self.update_tables()

    # --------------------- 定期更新 Tag 位置 --------------
    def update_position(self):
        """定期更新 Tag 位置"""
        
        try:
            act_anchors: list[Anchor] = [anchor for anchor in self.anchor_manager if anchor.enable]
            act_tags: list[Tag] = [tag for tag in self.tag_manager if tag.enable]

            if len(act_anchors) < 3:
                self.statusBar().showMessage("⚠️ 啟用中的 Anchor 少於 3 個，無法定位 Tag。")
                return


            rssi_rows = []
            for tag in act_tags:
                reports = self.controller.trigger_multiple(tag.id, [anchor.id for anchor in act_anchors])
                dist = [r.distance_m if r else -1 for r in reports]
                for anchor, report in zip(act_anchors, reports):
                    distance = report.distance_m if report else None
                    rssi_value = report.rssi_dbm if report else None
                    rssi_rows.append((
                        f"0x{tag.id:04X}",
                        f"0x{anchor.id:04X}",
                        f"{distance:.2f}" if distance is not None else "--",
                        f"{rssi_value:.1f}" if rssi_value is not None else "--",
                    ))
                # [([x, y, z], distance), ...]
                valid_pairs = [(act_anchors[i].pos, dist[i]) for i in range(len(dist)) if dist[i] > 0]

                if len(valid_pairs) >= 3:
                    anchor_pos, dist = zip(*valid_pairs)
                    tag_pos = TrilaterationSolver3D(list(anchor_pos)).solve(list(dist), known_z=tag.z)
                    tag_pos_f = tag.filter.filter(tag_pos)
                    x, y, z = tag_pos_f
                    tag.last_seen = time.time()
                    # tag.plot_item
                    if tag.plot_item is None:
                        tag.plot_item = self.plot_widget.plot([], [], pen=None, symbol="o", symbolSize=12)
                        color = self._color_for_tag(tag.id)
                        tag.plot_item.setSymbolBrush(color)
                    tag.plot_item.setData([x], [y])
                    self._append_tag_path_point(tag, x, y)
                    self.statusBar().showMessage(f"Tag 0x{tag.id:04X}: x={x:.2f}, y={y:.2f}, z={z:.2f}")
                else:
                    # 超過 2 秒沒看到就隱藏
                    if tag.last_seen and time.time() - tag.last_seen > 2.0:
                        if tag.plot_item:
                            tag.plot_item.setData([], [])
                        tag.last_seen = 0.0
            self._update_rssi_table(rssi_rows)
        except Exception as e:
            self.statusBar().showMessage(f"⚠️ {e}")
                        


if __name__ == "__main__":
    import os
    os.environ["QT_QPA_PLATFORM"] = "windows:darkmode=0"
    
    app = QtWidgets.QApplication(sys.argv)
    window = UWBMainWindow()
    window.show()
    sys.exit(app.exec())
