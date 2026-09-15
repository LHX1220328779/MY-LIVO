#!/usr/bin/env python3
"""
RTK Trajectory & Orientation 3D Viewer
=======================================
从 ROS2 MCAP bag 中读取 imu_data/odometry，可视化 RTK 位置轨迹和三维姿态轴。

左侧窗口：完整 3D 轨迹，可拖拽旋转/缩放/平移
底部窗口：2D 俯视图，支持矩形框选
右侧窗口：选中段轨迹点 + 三维姿态轴（RGB=XYZ 体轴）
"""

import os
import sys
import argparse
from pathlib import Path

# 确保使用系统 Python 以兼容 rosbag2_py (ABI 绑定)
if sys.executable != "/usr/bin/python3":
    clean_env = os.environ.copy()
    for var in ("CONDA_PREFIX", "CONDA_DEFAULT_ENV", "PYTHONHOME", "PYTHONPATH"):
        clean_env.pop(var, None)
    clean_env["PATH"] = (
        "/opt/ros/humble/bin:/usr/local/sbin:/usr/local/bin:"
        "/usr/sbin:/usr/bin:/sbin:/bin")
    clean_env["AMENT_PREFIX_PATH"] = "/opt/ros/humble"
    clean_env["PYTHONPATH"] = (
        "/opt/ros/humble/lib/python3.10/site-packages:"
        "/opt/ros/humble/local/lib/python3.10/dist-packages")
    os.execve("/usr/bin/python3", ["/usr/bin/python3", *sys.argv], clean_env)

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from nav_msgs.msg import Odometry
from scipy.spatial.transform import Rotation as R

import matplotlib
matplotlib.use('Qt5Agg')
from matplotlib.backends.backend_qt5agg import (
    FigureCanvasQTAgg, NavigationToolbar2QT)
from matplotlib.figure import Figure
from matplotlib.widgets import RectangleSelector
from matplotlib import cm
from PyQt5 import QtWidgets, QtCore

AXIS_LENGTH = 0.8
ORIENTATION_SAMPLE_STEP = 15
AXIS_COLORS = ['#FF3333', '#33FF33', '#3388FF']
AXIS_LABELS = ['X (右, RFU)', 'Y (前, RFU)', 'Z (上, RFU)']


def rfu_heading_degrees(quaternions):
    """North-clockwise heading of body +Y, derived from INS, NOT independent GNSS."""
    forward = R.from_quat(quaternions).as_matrix()[..., :, 1]
    heading = np.degrees(np.arctan2(forward[..., 0], forward[..., 1])) % 360
    return np.where(np.linalg.norm(forward[..., :2], axis=-1) > 1e-8, heading, np.nan)


def angle_difference_degrees(after, before):
    return (np.asarray(after) - before + 180) % 360 - 180

def load_odometry(bag_dir):
    bag_path = Path(bag_dir)
    if not bag_path.exists():
        raise SystemExit(f"bag 目录不存在: {bag_path}")
    metadata = bag_path / "metadata.yaml"
    if not metadata.exists():
        raise SystemExit(f"metadata.yaml 不存在: {metadata}")

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_path), storage_id="mcap"),
        rosbag2_py.ConverterOptions("", ""))

    topic_type = None
    for meta in reader.get_all_topics_and_types():
        if meta.name == "imu_data/odometry":
            topic_type = meta.type
            break
    if topic_type is None:
        raise SystemExit("未找到 imu_data/odometry 主题")

    positions = []
    orientations = []
    timestamps = []

    print("正在读取 odometry 数据...")
    while reader.has_next():
        topic, data, timestamp = reader.read_next()
        if topic != "imu_data/odometry":
            continue
        msg = deserialize_message(data, Odometry)
        if not msg.child_frame_id.endswith('_rfu'):
            raise ValueError(f"Expected an explicitly RFU body frame, got {msg.child_frame_id!r}")
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        positions.append([pos.x, pos.y, pos.z])
        orientations.append([ori.x, ori.y, ori.z, ori.w])
        timestamps.append(msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9)

    del reader

    positions = np.array(positions)
    orientations = np.array(orientations)
    timestamps = np.array(timestamps)

    print(f"读取完成: {len(positions)} 条 odometry 记录")
    print(f"时间范围: {timestamps[0]:.3f}s ~ {timestamps[-1]:.3f}s")
    print(f"位置范围 X: [{positions[:,0].min():.2f}, {positions[:,0].max():.2f}]")
    print(f"位置范围 Y: [{positions[:,1].min():.2f}, {positions[:,1].max():.2f}]")
    print(f"位置范围 Z: [{positions[:,2].min():.2f}, {positions[:,2].max():.2f}]")

    return positions, orientations, timestamps

class SharedState(QtCore.QObject):
    selection_changed = QtCore.pyqtSignal()

    def __init__(self, positions, orientations, timestamps):
        super().__init__()
        self.positions = positions
        self.orientations = orientations
        self.timestamps = timestamps
        self.n = len(positions)

        self.time_norm = (timestamps - timestamps[0]) / (
            timestamps[-1] - timestamps[0] + 1e-12)
        self.colors = cm.viridis(self.time_norm)

        rpy = R.from_quat(orientations).as_euler('xyz', degrees=True)
        self.roll = rpy[:, 0]
        self.pitch = rpy[:, 1]
        self.yaw = rpy[:, 2]
        self.heading = rfu_heading_degrees(orientations)

        self.selected_mask = np.ones(self.n, dtype=bool)
        self.selected_indices = np.arange(self.n)

    def set_selection(self, mask, indices):
        self.selected_mask = mask
        self.selected_indices = indices
        self.selection_changed.emit()

    def reset_selection(self):
        self.selected_mask = np.ones(self.n, dtype=bool)
        self.selected_indices = np.arange(self.n)
        self.selection_changed.emit()

    def clear_selection(self):
        self.selected_mask = np.zeros(self.n, dtype=bool)
        self.selected_indices = np.array([], dtype=int)
        self.selection_changed.emit()


class Trajectory3DWindow(QtWidgets.QMainWindow):
    def __init__(self, state):
        super().__init__()
        self.state = state
        self._init_ui()
        self._plot()

    def _init_ui(self):
        self.setWindowTitle("窗口1: 完整 RTK 轨迹 (3D)")
        self.resize(900, 750)

        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        layout = QtWidgets.QVBoxLayout(central)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(0)

        self.figure = Figure(dpi=100)
        self.canvas = FigureCanvasQTAgg(self.figure)
        self.canvas.setSizePolicy(
            QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
        layout.addWidget(self.canvas, stretch=1)

        self.toolbar = NavigationToolbar2QT(self.canvas, self)
        layout.addWidget(self.toolbar)

        self.ax = self.figure.add_subplot(111, projection='3d')
        self.figure.subplots_adjust(left=0.0, right=1.0, bottom=0.0, top=0.92)

    def resizeEvent(self, event):
        super().resizeEvent(event)
        dpi = self.figure.dpi
        w = max(self.canvas.width() / dpi, 4.0)
        h = max(self.canvas.height() / dpi, 3.0)
        self.figure.set_size_inches(w, h, forward=False)
        self.figure.subplots_adjust(left=0.0, right=1.0, bottom=0.0, top=0.92)
        self.canvas.draw_idle()

    def _plot(self):
        self.ax.clear()
        self.ax.set_title("完整 RTK 轨迹 (3D)", fontsize=13, fontweight='bold')
        self.ax.set_xlabel("X (m)")
        self.ax.set_ylabel("Y (m)")
        self.ax.set_zlabel("Z (m)")

        pos = self.state.positions
        self.ax.scatter(
            pos[:, 0], pos[:, 1], pos[:, 2],
            c=self.state.colors, s=2, alpha=0.7, rasterized=True)

        self.ax.scatter(*pos[0], c='lime', s=80, marker='o',
                        edgecolors='black', linewidths=1, zorder=5)
        self.ax.scatter(*pos[-1], c='red', s=80, marker='s',
                        edgecolors='black', linewidths=1, zorder=5)

        _set_axes_equal(self.ax, pos)
        self.figure.subplots_adjust(left=0.0, right=1.0, bottom=0.0, top=0.92)
        self.canvas.draw_idle()


class Selection2DWindow(QtWidgets.QMainWindow):
    def __init__(self, state):
        super().__init__()
        self.state = state
        self._init_ui()
        self._plot()

    def _init_ui(self):
        self.setWindowTitle("窗口2: 俯视框选 (XY 平面)")
        self.resize(1000, 500)

        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        layout = QtWidgets.QVBoxLayout(central)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(0)

        self.figure = Figure(dpi=100)
        self.canvas = FigureCanvasQTAgg(self.figure)
        self.canvas.setSizePolicy(
            QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
        layout.addWidget(self.canvas, stretch=1)

        self.toolbar = NavigationToolbar2QT(self.canvas, self)
        layout.addWidget(self.toolbar)

        info_bar = QtWidgets.QHBoxLayout()
        info_bar.setContentsMargins(6, 2, 6, 2)
        self.info_label = QtWidgets.QLabel("就绪 — 拖动鼠标框选轨迹段")
        self.info_label.setStyleSheet("font-size: 13px; padding: 2px;")
        info_bar.addWidget(self.info_label)
        info_bar.addStretch()

        reset_btn = QtWidgets.QPushButton("重置选择 (R)")
        reset_btn.clicked.connect(self._reset_selection)
        info_bar.addWidget(reset_btn)

        clear_btn = QtWidgets.QPushButton("清除选择 (C)")
        clear_btn.clicked.connect(self._clear_selection)
        info_bar.addWidget(clear_btn)

        layout.addLayout(info_bar)

        self.ax = self.figure.add_subplot(111)
        self.figure.subplots_adjust(left=0.06, right=0.98, bottom=0.08, top=0.90)

        self.rect_selector = RectangleSelector(
            self.ax, self._on_rect_select,
            useblit=True, button=[1],
            minspanx=5, minspany=5,
            spancoords='pixels', interactive=True,
            props=dict(facecolor='cyan', edgecolor='cyan',
                       alpha=0.25, fill=True))
        self.rect_selector.set_active(True)

        self.canvas.mpl_connect('key_press_event', self._on_key)

    def resizeEvent(self, event):
        super().resizeEvent(event)
        dpi = self.figure.dpi
        w = max(self.canvas.width() / dpi, 4.0)
        h = max(self.canvas.height() / dpi, 2.0)
        self.figure.set_size_inches(w, h, forward=False)
        self.figure.subplots_adjust(left=0.06, right=0.98, bottom=0.08, top=0.90)
        self.canvas.draw_idle()

    def _plot(self):
        self.ax.clear()
        self.ax.set_title("俯视框选 (XY 平面) — 拖动鼠标框选", fontsize=12)
        self.ax.set_xlabel("X (m)")
        self.ax.set_ylabel("Y (m)")
        self.ax.set_aspect('equal')

        pos = self.state.positions
        self.ax.scatter(
            pos[:, 0], pos[:, 1],
            c=self.state.colors, s=2, alpha=0.7, rasterized=True)

        self.ax.scatter(
            *pos[0, :2], c='lime', s=80, marker='o',
            edgecolors='black', linewidths=1, zorder=5)
        self.ax.scatter(
            *pos[-1, :2], c='red', s=80, marker='s',
            edgecolors='black', linewidths=1, zorder=5)

        if not np.all(self.state.selected_mask):
            sel = pos[self.state.selected_mask]
            self.ax.scatter(
                sel[:, 0], sel[:, 1],
                c='cyan', s=12, alpha=0.9, edgecolors='black',
                linewidths=0.5, zorder=4)

        self.figure.subplots_adjust(left=0.06, right=0.98, bottom=0.08, top=0.90)
        self.canvas.draw_idle()

    def _on_rect_select(self, eclick, erelease):
        x1, y1 = eclick.xdata, eclick.ydata
        x2, y2 = erelease.xdata, erelease.ydata
        if None in (x1, y1, x2, y2):
            return

        xmin, xmax = sorted([x1, x2])
        ymin, ymax = sorted([y1, y2])

        xy = self.state.positions[:, :2]
        mask = (xy[:, 0] >= xmin) & (xy[:, 0] <= xmax) & \
               (xy[:, 1] >= ymin) & (xy[:, 1] <= ymax)
        indices = np.where(mask)[0]

        self.state.set_selection(mask, indices)

        n_sel = len(indices)
        if n_sel > 0:
            t_sel = self.state.timestamps[indices]
            self.info_label.setText(
                f"选中 {n_sel} 个点  |  "
                f"时间: {t_sel[0]:.3f}s ~ {t_sel[-1]:.3f}s  |  "
                f"框选范围: X[{xmin:.2f}, {xmax:.2f}] Y[{ymin:.2f}, {ymax:.2f}]")
        else:
            self.info_label.setText("框选区域内无轨迹点")

        self._plot()

    def _reset_selection(self):
        self.state.reset_selection()
        self.info_label.setText(f"已重置 — 全部 {self.state.n} 个点")
        self._plot()

    def _clear_selection(self):
        self.state.clear_selection()
        self.info_label.setText("已清除选择")
        self._plot()

    def _on_key(self, event):
        if event.key == 'r':
            self._reset_selection()
        elif event.key == 'c':
            self._clear_selection()


class Orientation3DWindow(QtWidgets.QMainWindow):
    def __init__(self, state):
        super().__init__()
        self.state = state
        self._init_ui()
        self._plot()
        self.state.selection_changed.connect(self._plot)

    def _init_ui(self):
        self.setWindowTitle("窗口3: 选中段姿态轴 (3D)")
        self.resize(900, 750)

        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        layout = QtWidgets.QVBoxLayout(central)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(0)

        self.figure = Figure(dpi=100)
        self.canvas = FigureCanvasQTAgg(self.figure)
        self.canvas.setSizePolicy(
            QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
        layout.addWidget(self.canvas, stretch=1)

        self.toolbar = NavigationToolbar2QT(self.canvas, self)
        layout.addWidget(self.toolbar)

        self.ax = self.figure.add_subplot(111, projection='3d')
        self.figure.subplots_adjust(left=0.0, right=1.0, bottom=0.0, top=0.92)

    def resizeEvent(self, event):
        super().resizeEvent(event)
        dpi = self.figure.dpi
        w = max(self.canvas.width() / dpi, 4.0)
        h = max(self.canvas.height() / dpi, 3.0)
        self.figure.set_size_inches(w, h, forward=False)
        self.figure.subplots_adjust(left=0.0, right=1.0, bottom=0.0, top=0.92)
        self.canvas.draw_idle()

    def _plot(self):
        self.ax.clear()
        self.ax.set_title("选中段姿态轴 (3D)", fontsize=13, fontweight='bold')
        self.ax.set_xlabel("X (m)")
        self.ax.set_ylabel("Y (m)")
        self.ax.set_zlabel("Z (m)")

        idx = self.state.selected_indices
        if len(idx) == 0:
            self.ax.text2D(0.5, 0.5, "请先在窗口2框选轨迹段",
                           transform=self.ax.transAxes,
                           ha='center', va='center', fontsize=14, color='gray')
            self.canvas.draw_idle()
            return

        sel_pos = self.state.positions[idx]
        sel_colors = self.state.colors[idx]

        self.ax.scatter(
            sel_pos[:, 0], sel_pos[:, 1], sel_pos[:, 2],
            c=sel_colors, s=6, alpha=0.8, rasterized=True)

        _set_axes_equal(self.ax, sel_pos)

        x_range = self.ax.get_xlim()[1] - self.ax.get_xlim()[0]
        y_range = self.ax.get_ylim()[1] - self.ax.get_ylim()[0]
        z_range = self.ax.get_zlim()[1] - self.ax.get_zlim()[0]
        view_range = max(x_range, y_range, z_range)
        dynamic_len = min(view_range * 0.04, AXIS_LENGTH * 2.0)

        sample_step = max(1, int(ORIENTATION_SAMPLE_STEP))
        sample_idx = idx[::sample_step]
        if len(sample_idx) < 2:
            sample_idx = idx
        max_samples = 80
        if len(sample_idx) > max_samples:
            step = max(1, len(sample_idx) // max_samples)
            sample_idx = sample_idx[::step]

        rot = R.from_quat(self.state.orientations[sample_idx])
        mats = rot.as_matrix()

        axis_vectors = np.array([
            [1, 0, 0],
            [0, 1, 0],
            [0, 0, 1],
        ]) * dynamic_len

        for i, si in enumerate(sample_idx):
            origin = self.state.positions[si]
            mat = mats[i]
            for axis_idx in range(3):
                direction = mat @ axis_vectors[axis_idx]
                self.ax.quiver(
                    origin[0], origin[1], origin[2],
                    direction[0], direction[1], direction[2],
                    color=AXIS_COLORS[axis_idx],
                    alpha=0.7, linewidth=1.0,
                    arrow_length_ratio=0.25)

        for axis_idx in range(3):
            self.ax.plot([], [], [],
                         color=AXIS_COLORS[axis_idx],
                         label=AXIS_LABELS[axis_idx], linewidth=2)
        self.ax.legend(loc='upper right', fontsize=9)

        self.figure.subplots_adjust(left=0.0, right=1.0, bottom=0.0, top=0.92)
        self.canvas.draw_idle()


class Attitude3DWindow(QtWidgets.QMainWindow):
    def __init__(self, state):
        super().__init__()
        self.state = state
        self._init_ui()
        self._plot()
        self.state.selection_changed.connect(self._plot)

    def _init_ui(self):
        self.setWindowTitle("窗口4: 姿态角 2D 视图")
        self.resize(1000, 900)

        self.measure_mode = False
        self.measure_picks = []
        self._measure_artists = []

        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        layout = QtWidgets.QVBoxLayout(central)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(0)

        self.figure = Figure(dpi=100)
        self.canvas = FigureCanvasQTAgg(self.figure)
        self.canvas.setSizePolicy(
            QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
        layout.addWidget(self.canvas, stretch=1)

        self.toolbar = NavigationToolbar2QT(self.canvas, self)
        layout.addWidget(self.toolbar)

        bar = QtWidgets.QHBoxLayout()
        bar.setContentsMargins(6, 2, 6, 2)

        self.measure_btn = QtWidgets.QPushButton("🔍 测量 (M)")
        self.measure_btn.setCheckable(True)
        self.measure_btn.clicked.connect(self._toggle_measure)
        bar.addWidget(self.measure_btn)

        self.measure_label = QtWidgets.QLabel("")
        self.measure_label.setStyleSheet("font-size: 13px; padding: 2px;")
        bar.addWidget(self.measure_label, stretch=1)

        layout.addLayout(bar)

        self.ax_yaw = self.figure.add_subplot(411)
        self.ax_heading = self.figure.add_subplot(412)
        self.ax_pitch = self.figure.add_subplot(413)
        self.ax_roll = self.figure.add_subplot(414)
        self.figure.subplots_adjust(
            left=0.06, right=0.98, bottom=0.04, top=0.96, hspace=0.32)

        self._scatter_xy = {}
        self._cid_click = self.canvas.mpl_connect(
            'button_press_event', self._on_click)
        self._cid_key = self.canvas.mpl_connect(
            'key_press_event', self._on_key)

    def resizeEvent(self, event):
        super().resizeEvent(event)
        dpi = self.figure.dpi
        w = max(self.canvas.width() / dpi, 4.0)
        h = max(self.canvas.height() / dpi, 3.0)
        self.figure.set_size_inches(w, h, forward=False)
        self.figure.subplots_adjust(
            left=0.06, right=0.98, bottom=0.04, top=0.96, hspace=0.32)
        self.canvas.draw_idle()

    def _plot(self):
        for ax in (self.ax_yaw, self.ax_heading, self.ax_pitch, self.ax_roll):
            ax.clear()

        idx = self.state.selected_indices
        if len(idx) == 0:
            self.ax_yaw.text(0.5, 0.5, "请先在窗口2框选轨迹段",
                           transform=self.ax_yaw.transAxes,
                           ha='center', va='center', fontsize=12, color='gray')
            self.canvas.draw_idle()
            return

        pos = self.state.positions[idx]
        colors = self.state.colors[idx]
        roll = self.state.roll[idx]
        pitch = self.state.pitch[idx]
        yaw = self.state.yaw[idx]

        x = pos[:, 0]

        self._plot_one(self.ax_yaw, x, yaw, colors, "ENU ZYX yaw：RFU +X 右轴方位", "Yaw (°)", idx)
        self._plot_one(self.ax_heading, x, self.state.heading[idx], colors,
                       "INS 推导 heading：+Y 前向，北起顺时针（非独立 GNSS 航向）", "Heading (°)", idx)
        self._plot_one(self.ax_pitch, x, pitch, colors, "ZYX 分量 Ry（RFU 前轴）", "Ry (°)", idx)
        self._plot_one(self.ax_roll, x, roll, colors, "ZYX 分量 Rx（RFU 右轴）", "Rx (°)", idx)

        self._redraw_measure_markers()

        self.figure.subplots_adjust(
            left=0.06, right=0.98, bottom=0.04, top=0.96, hspace=0.32)
        self.canvas.draw_idle()

    def _toggle_measure(self):
        self.measure_mode = self.measure_btn.isChecked()
        self.measure_picks = []
        self._clear_measure_artists()
        if self.measure_mode:
            self.measure_label.setText("测量模式: 请点击第1个轨迹点")
            self.measure_label.setStyleSheet(
                "font-size: 13px; padding: 2px; color: #FF6600; font-weight: bold;")
            self.measure_btn.setText("🔍 测量中... (M)")
        else:
            self.measure_label.setText("")
            self.measure_label.setStyleSheet("font-size: 13px; padding: 2px;")
            self.measure_btn.setText("🔍 测量 (M)")
        self.canvas.draw_idle()

    def _on_click(self, event):
        if not self.measure_mode:
            return
        if event.inaxes not in (self.ax_yaw, self.ax_heading, self.ax_pitch, self.ax_roll):
            return
        if event.xdata is None or event.ydata is None:
            return

        ax = event.inaxes
        if ax not in self._scatter_xy:
            return

        x_arr, y_arr, global_idx = self._scatter_xy[ax]
        dists = (x_arr - event.xdata) ** 2 + (y_arr - event.ydata) ** 2
        nearest = int(np.argmin(dists))
        nearest_global = global_idx[nearest]
        px, py = float(x_arr[nearest]), float(y_arr[nearest])

        angle_names = {self.ax_yaw: 'Yaw', self.ax_heading: 'Heading', self.ax_pitch: 'Ry', self.ax_roll: 'Rx'}
        angle_name = angle_names.get(ax, '?')

        pick = (ax, nearest_global, nearest, px, py, angle_name)

        if len(self.measure_picks) >= 2:
            self.measure_picks = []

        self.measure_picks.append(pick)

        if len(self.measure_picks) == 1:
            self.measure_label.setText(
                f"测量模式: 已选点1 ({angle_name}={py:.2f}°) — 请点击第2个点")
        elif len(self.measure_picks) == 2:
            p1, p2 = self.measure_picks
            g1, g2 = p1[1], p2[1]
            if g1 == g2:
                self.measure_picks.pop()
                self.measure_label.setText("同一轨迹点，请重新选择第2个点")
            else:
                d_roll = angle_difference_degrees(self.state.roll[g2], self.state.roll[g1])
                d_pitch = angle_difference_degrees(self.state.pitch[g2], self.state.pitch[g1])
                d_yaw = angle_difference_degrees(self.state.yaw[g2], self.state.yaw[g1])
                d_heading = angle_difference_degrees(self.state.heading[g2], self.state.heading[g1])
                self.measure_label.setText(
                    f"ΔRx={d_roll:+.2f}°  |  ΔRy={d_pitch:+.2f}°  |  "
                    f"ΔYaw={d_yaw:+.2f}°  |  ΔHeading={d_heading:+.2f}°")

        self._redraw_measure_markers()
        self.canvas.draw_idle()

    def _clear_measure_artists(self):
        for artist in self._measure_artists:
            artist.remove()
        self._measure_artists.clear()

    def _redraw_measure_markers(self):
        self._clear_measure_artists()
        if not self.measure_picks:
            return
        for i, (ax_target, _, _, px, py, _) in enumerate(self.measure_picks):
            artist = ax_target.scatter([px], [py],
                                       facecolors='none', s=160, marker='o',
                                       edgecolors='#FF00FF', linewidths=3.0,
                                       zorder=10)
            self._measure_artists.append(artist)

    def _on_key(self, event):
        if event.key == 'm':
            self.measure_btn.setChecked(not self.measure_mode)
            self._toggle_measure()

    def _plot_one(self, ax, x, y, colors, title, ylabel, global_idx):
        ax.set_title(title, fontsize=12, fontweight='bold')
        ax.set_xlabel("X (m)")
        ax.set_ylabel(ylabel)
        ax.grid(True, alpha=0.3, linestyle='--')

        sc = ax.scatter(x, y, c=colors, s=4, alpha=0.8,
                        rasterized=True)
        self._scatter_xy[ax] = (x, y, global_idx)


def _set_axes_equal(ax, points):
    x, y, z = points[:, 0], points[:, 1], points[:, 2]
    max_range = max(
        x.ptp() or 1.0, y.ptp() or 1.0, z.ptp() or 1.0) * 0.55
    mid_x, mid_y, mid_z = x.mean(), y.mean(), z.mean()
    ax.set_xlim(mid_x - max_range, mid_x + max_range)
    ax.set_ylim(mid_y - max_range, mid_y + max_range)
    ax.set_zlim(mid_z - max_range, mid_z + max_range)

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--bag", type=Path,
        default=Path(
            "/home/project/data/haibo/wuhu_livo/ros2bag/Perception_wuhu_29_ConchPilot6.1_2026.07.18_V6.1.1_hotfix01_2026-08-05-14-10-01_549"),
        help="ROS2 MCAP bag 目录路径")
    parser.add_argument(
        "--axis-length", type=float, default=0.8,
        help="姿态轴箭头长度 (默认 0.8m)")
    parser.add_argument(
        "--sample-step", type=int, default=15,
        help="姿态轴显示采样间隔 (默认每 15 个点显示一个)")
    args = parser.parse_args()

    global AXIS_LENGTH, ORIENTATION_SAMPLE_STEP
    AXIS_LENGTH = args.axis_length
    ORIENTATION_SAMPLE_STEP = args.sample_step

    positions, orientations, timestamps = load_odometry(args.bag)

    app = QtWidgets.QApplication(sys.argv)
    app.setStyle('Fusion')

    state = SharedState(positions, orientations, timestamps)

    win1 = Trajectory3DWindow(state)
    win2 = Selection2DWindow(state)
    win3 = Orientation3DWindow(state)
    win4 = Attitude3DWindow(state)

    screen_geom = app.primaryScreen().availableGeometry()
    sw, sh = screen_geom.width(), screen_geom.height()

    win_w = int(sw * 0.32)
    win_h = int(sh * 0.55)
    win2_h = int(sh * 0.38)

    win1.setGeometry(int(sw * 0.01), int(sh * 0.02), win_w, win_h)
    win2.setGeometry(int(sw * 0.01), int(sh * 0.60), int(sw * 0.65), win2_h)
    win3.setGeometry(int(sw * 0.35), int(sh * 0.02), win_w, win_h)
    win4.setGeometry(int(sw * 0.68), int(sh * 0.02), win_w, win_h)

    win1.show()
    win2.show()
    win3.show()
    win4.show()

    print("\n操作说明:")
    print("  窗口1 (完整3D轨迹):    鼠标拖拽旋转 | 右键拖拽缩放 | 中键拖拽平移")
    print("  窗口2 (俯视框选):      鼠标拖拽框选轨迹段 | R=重置  C=清除")
    print("  窗口3 (选中段姿态轴):  显示选中段的轨迹点和姿态轴 (RGB=XYZ)")
    print("  窗口4 (姿态角3D视图):  选中段轨迹 XY + Roll/Pitch/Yaw 角度")
    print("  所有窗口均可独立全屏 (F11 或双击标题栏)")

    sys.exit(app.exec_())

if __name__ == "__main__":
    main()
