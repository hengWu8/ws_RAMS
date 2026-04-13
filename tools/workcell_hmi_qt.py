#!/usr/bin/env python3
"""Qt-based read-only HMI for the ws_RAMS ABB workcell."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import struct
import time
from typing import Any

from PySide2 import QtCore, QtGui, QtWidgets

import workcell_dashboard as backend


_GEOMETRY_CACHE: dict[str, dict[str, Any]] = {}
_LINK_COLORS = {
    "base_link": "#4b5563",
    "link_1": "#d9dee7",
    "link_2": "#d9dee7",
    "link_3": "#cfd6e0",
    "link_4": "#d9dee7",
    "link_5": "#cfd6e0",
    "link_6": "#d9dee7",
    "cylinder": "#f59e0b",
    "piston": "#94a3b8",
}

_MOTION_DIRECTIONS = (
    ("+Y left", (0.0, 1.0, 0.0)),
    ("-Y right", (0.0, -1.0, 0.0)),
    ("+X forward", (1.0, 0.0, 0.0)),
    ("-X backward", (-1.0, 0.0, 0.0)),
    ("+Z up", (0.0, 0.0, 1.0)),
    ("-Z down", (0.0, 0.0, -1.0)),
)


class MonitorThread(QtCore.QThread):
    status_ready = QtCore.Signal(dict)
    image_ready = QtCore.Signal(str, bytes)
    worker_error = QtCore.Signal(str)

    def __init__(self, config: dict[str, Any], *, status_interval_s: float = 2.0, image_interval_s: float = 0.7):
        super().__init__()
        self._config = config
        self._status_interval_s = status_interval_s
        self._image_interval_s = image_interval_s
        self._ros_runtime: dict[str, Any] | None = None
        self._last_camera_error_text = ""
        self._last_camera_error_time = 0.0

    def run(self) -> None:
        last_status = 0.0
        last_images = 0.0
        use_ros_camera_topics = self._setup_ros_camera_subscriptions()
        allow_v4l2_fallback = bool(self._config.get("allow_v4l2_camera_fallback", False))
        try:
            while not self.isInterruptionRequested():
                now = time.time()
                try:
                    if now - last_status >= self._status_interval_s:
                        self.status_ready.emit(backend.build_status(self._config))
                        last_status = now
                    if use_ros_camera_topics:
                        self._spin_ros_once(timeout_sec=0.05)
                    elif allow_v4l2_fallback and now - last_images >= self._image_interval_s:
                        for name, device in (
                            ("front", self._config["front_camera_device"]),
                            ("wrist", self._config["wrist_camera_device"]),
                        ):
                            image_bytes, _content_type = backend.capture_camera_jpeg(
                                device,
                                width=self._config["camera_width"],
                                height=self._config["camera_height"],
                                quality=self._config["camera_jpeg_quality"],
                            )
                            self.image_ready.emit(name, image_bytes)
                        last_images = now
                except Exception as exc:  # pragma: no cover - GUI loop only
                    self.worker_error.emit(str(exc))
                self.msleep(50)
        finally:
            self._teardown_ros_camera_subscriptions()

    def _setup_ros_camera_subscriptions(self) -> bool:
        front_topic = str(self._config.get("front_camera_topic", "") or "").strip()
        wrist_topic = str(self._config.get("wrist_camera_topic", "") or "").strip()
        if not front_topic or not wrist_topic:
            return False
        try:
            import cv2
            import rclpy
            from cv_bridge import CvBridge
            from rclpy.node import Node
            from rclpy.qos import qos_profile_sensor_data
            from sensor_msgs.msg import Image

            owns_init = False
            if not rclpy.ok():
                rclpy.init(args=None)
                owns_init = True

            node = Node("ws_rams_hmi_camera_preview")
            bridge = CvBridge()
            subscriptions = []
            for camera_name, topic_name in (("front", front_topic), ("wrist", wrist_topic)):
                subscriptions.append(
                    node.create_subscription(
                        Image,
                        topic_name,
                        lambda msg, name=camera_name: self._handle_ros_image(name, msg),
                        qos_profile_sensor_data,
                    )
                )

            self._ros_runtime = {
                "cv2": cv2,
                "rclpy": rclpy,
                "node": node,
                "bridge": bridge,
                "subscriptions": subscriptions,
                "owns_init": owns_init,
            }
            self.worker_error.emit(
                f"Camera preview source: ROS topics ({front_topic}, {wrist_topic})"
            )
            return True
        except Exception as exc:
            if self._config.get("allow_v4l2_camera_fallback", False):
                self.worker_error.emit(f"ROS preview unavailable, falling back to V4L2: {exc}")
            else:
                self.worker_error.emit(
                    "ROS preview unavailable and V4L2 fallback disabled. "
                    f"Start HMI after sourcing ROS/workspace setup. Detail: {exc}"
                )
            self._ros_runtime = None
            return False

    def _spin_ros_once(self, timeout_sec: float) -> None:
        if not self._ros_runtime:
            return
        self._ros_runtime["rclpy"].spin_once(self._ros_runtime["node"], timeout_sec=timeout_sec)

    def _teardown_ros_camera_subscriptions(self) -> None:
        runtime = self._ros_runtime
        self._ros_runtime = None
        if not runtime:
            return
        try:
            runtime["node"].destroy_node()
        except Exception:
            pass
        if runtime.get("owns_init"):
            try:
                runtime["rclpy"].shutdown()
            except Exception:
                pass

    def _handle_ros_image(self, camera_name: str, message: Any) -> None:
        runtime = self._ros_runtime
        if not runtime:
            return
        try:
            frame = runtime["bridge"].imgmsg_to_cv2(message, desired_encoding="bgr8")
            cv2 = runtime["cv2"]
            width = int(self._config["camera_width"])
            height = int(self._config["camera_height"])
            if frame.shape[1] != width or frame.shape[0] != height:
                interpolation = cv2.INTER_AREA if frame.shape[1] >= width and frame.shape[0] >= height else cv2.INTER_LINEAR
                frame = cv2.resize(frame, (width, height), interpolation=interpolation)
            encoded_ok, encoded = cv2.imencode(
                ".jpg",
                frame,
                [int(cv2.IMWRITE_JPEG_QUALITY), int(self._config["camera_jpeg_quality"])],
            )
            if encoded_ok:
                self.image_ready.emit(camera_name, encoded.tobytes())
        except Exception as exc:
            self._report_camera_error(f"{camera_name} ROS frame decode failed: {exc}")

    def _report_camera_error(self, text: str) -> None:
        now = time.time()
        if text != self._last_camera_error_text or (now - self._last_camera_error_time) > 5.0:
            self._last_camera_error_text = text
            self._last_camera_error_time = now
            self.worker_error.emit(text)


class StatusBadge(QtWidgets.QFrame):
    def __init__(self, title: str):
        super().__init__()
        self.setObjectName("StatusBadge")
        layout = QtWidgets.QHBoxLayout(self)
        layout.setContentsMargins(12, 10, 12, 10)
        layout.setSpacing(10)

        self.dot = QtWidgets.QLabel()
        self.dot.setFixedSize(14, 14)
        self.dot.setObjectName("BadgeDot")
        self.title_label = QtWidgets.QLabel(title)
        self.title_label.setObjectName("BadgeTitle")
        self.value_label = QtWidgets.QLabel("-")
        self.value_label.setObjectName("BadgeValue")
        self.value_label.setAlignment(QtCore.Qt.AlignRight | QtCore.Qt.AlignVCenter)

        layout.addWidget(self.dot)
        layout.addWidget(self.title_label)
        layout.addStretch(1)
        layout.addWidget(self.value_label)
        self.set_status("neutral", "-")

    def set_status(self, level: str, text: str) -> None:
        colors = {
            "good": "#10b981",
            "warn": "#f59e0b",
            "bad": "#ef4444",
            "neutral": "#64748b",
        }
        color = colors.get(level, colors["neutral"])
        self.dot.setStyleSheet(f"background:{color}; border-radius:0px;")
        self.value_label.setText(text)


class InfoPanel(QtWidgets.QFrame):
    def __init__(self, title: str):
        super().__init__()
        self.setObjectName("InfoPanel")
        self.layout_ = QtWidgets.QVBoxLayout(self)
        self.layout_.setContentsMargins(18, 16, 18, 16)
        self.layout_.setSpacing(12)
        title_label = QtWidgets.QLabel(title)
        title_label.setObjectName("PanelTitle")
        self.layout_.addWidget(title_label)


class CameraPanel(InfoPanel):
    def __init__(self, title: str):
        super().__init__(title)
        self.device_label = QtWidgets.QLabel("-")
        self.device_label.setObjectName("MetaLabel")
        self.layout_.addWidget(self.device_label)
        self.status_label = QtWidgets.QLabel("No live frame yet")
        self.status_label.setObjectName("MonitorDetail")
        self.layout_.addWidget(self.status_label)
        self.image_label = QtWidgets.QLabel()
        self.image_label.setMinimumSize(480, 270)
        self.image_label.setAlignment(QtCore.Qt.AlignCenter)
        self.image_label.setObjectName("CameraImage")
        self.layout_.addWidget(self.image_label, 1)

    def set_device(self, text: str) -> None:
        self.device_label.setText(text)

    def set_status_text(self, text: str) -> None:
        self.status_label.setText(text)

    def set_image_bytes(self, payload: bytes) -> None:
        image = QtGui.QImage.fromData(payload, "JPG")
        if image.isNull():
            return
        pixmap = QtGui.QPixmap.fromImage(image)
        scaled = pixmap.scaled(
            self.image_label.size(),
            QtCore.Qt.KeepAspectRatioByExpanding,
            QtCore.Qt.SmoothTransformation,
        )
        self.image_label.setPixmap(scaled)

    def resizeEvent(self, event: QtGui.QResizeEvent) -> None:
        if self.image_label.pixmap() is not None:
            self.image_label.setPixmap(
                self.image_label.pixmap().scaled(
                    self.image_label.size(),
                    QtCore.Qt.KeepAspectRatioByExpanding,
                    QtCore.Qt.SmoothTransformation,
                )
            )
        super().resizeEvent(event)


class RobotCanvas(QtWidgets.QWidget):
    def __init__(self):
        super().__init__()
        self._scene: dict[str, Any] | None = None
        self._scene_meshes: list[dict[str, Any]] = []
        self._tcp_text = "TCP unavailable"
        self._mesh_stats = "Waiting for robot scene..."
        self._workspace_text = "Workspace radius unavailable"
        self._tip_frame = None
        self._base_frame = None
        self._workspace_radius_m = 0.5
        self.setMinimumSize(620, 420)
        self._yaw = math.radians(42.0)
        self._pitch = math.radians(22.0)
        self._distance = 5.4
        self._orbit_target = [0.8, 0.0, 1.1]
        self._last_mouse_pos: QtCore.QPoint | None = None
        self._mesh_error = ""
        self.setMouseTracking(True)

    def update_model(self, scene: dict[str, Any] | None, tcp_text: str, *, workspace_radius_m: float) -> None:
        import numpy as np

        self._scene = scene
        self._tcp_text = tcp_text
        self._workspace_radius_m = float(workspace_radius_m)
        self._workspace_text = f"Workspace cube +/- {self._workspace_radius_m:.2f} m around live TCP"
        self._mesh_error = ""
        self._scene_meshes = []
        self._mesh_stats = "No scene data"
        self._tip_frame = None
        self._base_frame = np.eye(4, dtype=float)
        try:
            if scene and scene.get("ok"):
                self._scene_meshes = self._build_scene_meshes(scene)
                tip_frame = scene.get("tip_frame")
                if tip_frame is not None:
                    self._tip_frame = np.asarray(tip_frame, dtype=float)
                base_frame = (scene.get("link_frames") or {}).get(scene.get("base_link", "base_link"))
                if base_frame is not None:
                    self._base_frame = np.asarray(base_frame, dtype=float)
                self._fit_orbit_target()
                visual_count = len(scene.get("visuals", []))
                triangle_count = sum(int(mesh["triangles_world"].shape[0]) for mesh in self._scene_meshes)
                self._mesh_stats = f"{visual_count} visuals  |  {triangle_count} tris"
            elif scene and scene.get("error"):
                self._mesh_stats = str(scene["error"])
        except Exception as exc:
            self._mesh_error = str(exc)
            self._mesh_stats = f"Scene update failed: {exc}"
        self.update()

    def paintEvent(self, event: QtGui.QPaintEvent) -> None:
        import numpy as np

        painter = QtGui.QPainter(self)
        try:
            painter.setRenderHint(QtGui.QPainter.Antialiasing)
            rect = self.rect().adjusted(10, 10, -10, -10)
            if rect.width() <= 4 or rect.height() <= 4:
                return

            painter.fillRect(rect, QtGui.QColor("#09111f"))
            painter.setPen(QtGui.QPen(QtGui.QColor("#2b3b5d"), 1))
            painter.drawRect(rect)

            viewport = rect.adjusted(14, 42, -14, -14)
            painter.fillRect(viewport, QtGui.QColor("#050b16"))
            self._draw_grid(painter, viewport)
            self._draw_hud(painter, rect)

            if not self._scene or not self._scene.get("ok") or not self._scene_meshes:
                painter.setPen(QtGui.QColor("#cbd5e1"))
                painter.setFont(QtGui.QFont("Sans Serif", 12, QtGui.QFont.Bold))
                error_text = self._mesh_error or (self._scene.get("error", "No URDF mesh data") if self._scene else "No URDF mesh data")
                painter.drawText(viewport, QtCore.Qt.AlignCenter, error_text)
                return

            eye, right_axis, up_axis, forward_axis = self._camera_axes()
            light_dir = _normalize_vector(np.asarray([-0.5, -0.2, 0.84], dtype=float))
            focal = min(viewport.width(), viewport.height()) * 0.92
            center_x = viewport.center().x()
            center_y = viewport.center().y()
            near_plane = 0.08
            far_plane = 50.0

            polygons = []
            for mesh in self._scene_meshes:
                triangles_world = mesh["triangles_world"]
                normals_world = mesh["normals_world"]
                camera_triangles = self._world_to_camera(triangles_world, eye, right_axis, up_axis, forward_axis)
                face_centers = camera_triangles.mean(axis=1)
                positive_depth = np.all(camera_triangles[:, :, 2] > near_plane, axis=1) & (face_centers[:, 2] < far_plane)
                if not positive_depth.any():
                    continue

                camera_triangles = camera_triangles[positive_depth]
                normals_world = normals_world[positive_depth]
                face_centers_world = triangles_world[positive_depth].mean(axis=1)
                to_camera_world = _normalize_rows(eye - face_centers_world)
                facing = (normals_world * to_camera_world).sum(axis=1) > 0.0
                if not facing.any():
                    continue

                camera_triangles = camera_triangles[facing]
                normals_world = normals_world[facing]
                if camera_triangles.size == 0:
                    continue

                projected_x = center_x + (camera_triangles[:, :, 0] / camera_triangles[:, :, 2]) * focal
                projected_y = center_y - (camera_triangles[:, :, 1] / camera_triangles[:, :, 2]) * focal
                intensities = 0.26 + 0.74 * np.clip((normals_world * light_dir).sum(axis=1), 0.0, 1.0)
                depths = camera_triangles[:, :, 2].mean(axis=1)
                finite_mask = np.isfinite(projected_x).all(axis=1) & np.isfinite(projected_y).all(axis=1) & np.isfinite(depths)
                if not finite_mask.any():
                    continue
                projected_x = projected_x[finite_mask]
                projected_y = projected_y[finite_mask]
                intensities = intensities[finite_mask]
                depths = depths[finite_mask]
                base_color = QtGui.QColor(mesh["color"])

                for tri_x, tri_y, intensity, depth in zip(projected_x, projected_y, intensities, depths):
                    polygons.append(
                        (
                            float(depth),
                            QtGui.QPolygonF(
                                [
                                    QtCore.QPointF(float(tri_x[0]), float(tri_y[0])),
                                    QtCore.QPointF(float(tri_x[1]), float(tri_y[1])),
                                    QtCore.QPointF(float(tri_x[2]), float(tri_y[2])),
                                ]
                            ),
                            _shade_color(base_color, float(intensity)),
                        )
                    )

            polygons.sort(key=lambda item: item[0], reverse=True)
            painter.setPen(QtGui.QPen(QtGui.QColor(8, 16, 28, 90), 0.8))
            for _depth, polygon, color in polygons:
                painter.setBrush(color)
                painter.drawPolygon(polygon)
            self._draw_workspace_box(painter, viewport, eye, right_axis, up_axis, forward_axis, focal)
            self._draw_frame_axes(
                painter,
                viewport,
                self._base_frame,
                eye,
                right_axis,
                up_axis,
                forward_axis,
                focal,
                axis_length=0.24,
                label_prefix="BASE",
                color_override=None,
            )
            self._draw_frame_axes(
                painter,
                viewport,
                self._tip_frame,
                eye,
                right_axis,
                up_axis,
                forward_axis,
                focal,
                axis_length=0.18,
                label_prefix="TCP",
                color_override=QtGui.QColor("#f59e0b"),
            )
            self._draw_axis_triad(painter, viewport)
        except Exception as exc:
            self._mesh_error = f"3D render error: {exc}"
            painter.setClipping(False)
            painter.fillRect(self.rect(), QtGui.QColor("#09111f"))
            painter.setPen(QtGui.QColor("#ef4444"))
            painter.setFont(QtGui.QFont("Sans Serif", 11, QtGui.QFont.Bold))
            painter.drawText(self.rect().adjusted(24, 24, -24, -24), QtCore.Qt.AlignCenter, self._mesh_error)
        finally:
            painter.end()

    def mousePressEvent(self, event: QtGui.QMouseEvent) -> None:
        if event.button() == QtCore.Qt.LeftButton:
            self._last_mouse_pos = event.pos()
        super().mousePressEvent(event)

    def mouseMoveEvent(self, event: QtGui.QMouseEvent) -> None:
        if self._last_mouse_pos is None:
            return super().mouseMoveEvent(event)
        delta = event.pos() - self._last_mouse_pos
        self._last_mouse_pos = event.pos()
        self._yaw -= delta.x() * 0.008
        self._pitch = max(math.radians(-80.0), min(math.radians(80.0), self._pitch + delta.y() * 0.006))
        self.update()
        super().mouseMoveEvent(event)

    def mouseReleaseEvent(self, event: QtGui.QMouseEvent) -> None:
        self._last_mouse_pos = None
        super().mouseReleaseEvent(event)

    def wheelEvent(self, event: QtGui.QWheelEvent) -> None:
        delta_steps = event.angleDelta().y() / 120.0
        zoom = math.pow(0.88, delta_steps)
        self._distance = max(2.0, min(10.0, self._distance * zoom))
        self.update()
        super().wheelEvent(event)

    def _draw_hud(self, painter: QtGui.QPainter, rect: QtCore.QRect) -> None:
        painter.setPen(QtGui.QColor("#e2e8f0"))
        painter.setFont(QtGui.QFont("Sans Serif", 11, QtGui.QFont.Bold))
        painter.drawText(rect.adjusted(0, 0, -16, 0), QtCore.Qt.AlignTop | QtCore.Qt.AlignRight, self._tcp_text)
        painter.setPen(QtGui.QColor("#94a3b8"))
        painter.setFont(QtGui.QFont("Sans Serif", 9))
        painter.drawText(rect.adjusted(14, 8, 0, 0), "URDF 3D view  |  drag to orbit  |  wheel to zoom")
        painter.drawText(rect.adjusted(14, 24, 0, 0), self._mesh_stats)
        painter.drawText(rect.adjusted(14, 40, 0, 0), self._workspace_text)

    def _draw_grid(self, painter: QtGui.QPainter, viewport: QtCore.QRect) -> None:
        import numpy as np

        eye, right_axis, up_axis, forward_axis = self._camera_axes()
        painter.save()
        painter.setClipRect(viewport)
        painter.setPen(QtGui.QPen(QtGui.QColor(43, 59, 93, 70), 1))
        focal = min(viewport.width(), viewport.height()) * 0.92
        center_x = viewport.center().x()
        center_y = viewport.center().y()

        def project(point_world):
            delta = point_world - eye
            cam = np.asarray(
                [
                    float(delta @ right_axis),
                    float(delta @ up_axis),
                    float(delta @ forward_axis),
                ],
                dtype=float,
            )
            if cam[2] <= 0.08:
                return None
            return QtCore.QPointF(
                float(center_x + (cam[0] / cam[2]) * focal),
                float(center_y - (cam[1] / cam[2]) * focal),
            )

        for offset in [v * 0.5 for v in range(-5, 6)]:
            p1 = project(np.asarray([-2.5, offset, 0.0], dtype=float))
            p2 = project(np.asarray([2.5, offset, 0.0], dtype=float))
            if p1 is not None and p2 is not None:
                painter.drawLine(p1, p2)
            p3 = project(np.asarray([offset, -2.5, 0.0], dtype=float))
            p4 = project(np.asarray([offset, 2.5, 0.0], dtype=float))
            if p3 is not None and p4 is not None:
                painter.drawLine(p3, p4)
        painter.restore()

    def _draw_axis_triad(self, painter: QtGui.QPainter, viewport: QtCore.QRect) -> None:
        import numpy as np

        eye, right_axis, up_axis, forward_axis = self._camera_axes()
        focal = min(viewport.width(), viewport.height()) * 0.92
        anchor = np.asarray(self._orbit_target, dtype=float) + np.asarray([-0.7, -0.7, 0.25], dtype=float)
        axis_points = {
            "X": (anchor + np.asarray([0.18, 0.0, 0.0], dtype=float), QtGui.QColor("#ef4444")),
            "Y": (anchor + np.asarray([0.0, 0.18, 0.0], dtype=float), QtGui.QColor("#22c55e")),
            "Z": (anchor + np.asarray([0.0, 0.0, 0.18], dtype=float), QtGui.QColor("#38bdf8")),
        }

        def project(point_world):
            delta = point_world - eye
            cam = np.asarray(
                [
                    float(delta @ right_axis),
                    float(delta @ up_axis),
                    float(delta @ forward_axis),
                ],
                dtype=float,
            )
            if cam[2] <= 0.08:
                return None
            return QtCore.QPointF(
                float(viewport.center().x() + (cam[0] / cam[2]) * focal),
                float(viewport.center().y() - (cam[1] / cam[2]) * focal),
            )

        origin_2d = project(anchor)
        if origin_2d is None:
            return
        painter.save()
        painter.setFont(QtGui.QFont("Sans Serif", 8, QtGui.QFont.Bold))
        for label, (point_world, color) in axis_points.items():
            point_2d = project(point_world)
            if point_2d is None:
                continue
            painter.setPen(QtGui.QPen(color, 2))
            painter.drawLine(origin_2d, point_2d)
            painter.drawText(QtCore.QRectF(point_2d.x() + 4, point_2d.y() - 8, 18, 16), label)
        painter.restore()

    def _draw_workspace_box(
        self,
        painter: QtGui.QPainter,
        viewport: QtCore.QRect,
        eye,
        right_axis,
        up_axis,
        forward_axis,
        focal: float,
    ) -> None:
        import numpy as np

        if self._tip_frame is None:
            return
        center = self._tip_frame[:3, 3]
        radius = max(0.05, float(self._workspace_radius_m))
        offsets = np.asarray(
            [
                [-radius, -radius, -radius],
                [radius, -radius, -radius],
                [radius, radius, -radius],
                [-radius, radius, -radius],
                [-radius, -radius, radius],
                [radius, -radius, radius],
                [radius, radius, radius],
                [-radius, radius, radius],
            ],
            dtype=float,
        )
        points = [self._project_point(center + offset, viewport, eye, right_axis, up_axis, forward_axis, focal) for offset in offsets]
        if any(point is None for point in points):
            return
        edges = (
            (0, 1), (1, 2), (2, 3), (3, 0),
            (4, 5), (5, 6), (6, 7), (7, 4),
            (0, 4), (1, 5), (2, 6), (3, 7),
        )
        painter.save()
        painter.setPen(QtGui.QPen(QtGui.QColor(56, 189, 248, 150), 1.5, QtCore.Qt.DashLine))
        for start, end in edges:
            painter.drawLine(points[start], points[end])
        painter.setPen(QtGui.QColor("#7dd3fc"))
        label_anchor = points[6]
        painter.drawText(
            QtCore.QRectF(label_anchor.x() + 8, label_anchor.y() - 18, 160, 18),
            f"WORKSPACE +/-{radius:.2f}m",
        )
        painter.restore()

    def _draw_frame_axes(
        self,
        painter: QtGui.QPainter,
        viewport: QtCore.QRect,
        transform,
        eye,
        right_axis,
        up_axis,
        forward_axis,
        focal: float,
        *,
        axis_length: float,
        label_prefix: str,
        color_override: QtGui.QColor | None,
    ) -> None:
        import numpy as np

        if transform is None:
            return
        origin = np.asarray(transform[:3, 3], dtype=float)
        rotation = np.asarray(transform[:3, :3], dtype=float)
        origin_2d = self._project_point(origin, viewport, eye, right_axis, up_axis, forward_axis, focal)
        if origin_2d is None:
            return
        axes = (
            ("X", rotation[:, 0], color_override or QtGui.QColor("#ef4444")),
            ("Y", rotation[:, 1], color_override or QtGui.QColor("#22c55e")),
            ("Z", rotation[:, 2], color_override or QtGui.QColor("#38bdf8")),
        )
        painter.save()
        painter.setFont(QtGui.QFont("Sans Serif", 8, QtGui.QFont.Bold))
        for label, vector, color in axes:
            end_point = origin + _normalize_vector(vector) * axis_length
            end_2d = self._project_point(end_point, viewport, eye, right_axis, up_axis, forward_axis, focal)
            if end_2d is None:
                continue
            painter.setPen(QtGui.QPen(color, 2))
            painter.drawLine(origin_2d, end_2d)
            painter.drawText(QtCore.QRectF(end_2d.x() + 4, end_2d.y() - 8, 46, 16), f"{label_prefix}.{label}")
        painter.restore()

    @staticmethod
    def _project_point(point_world, viewport, eye, right_axis, up_axis, forward_axis, focal: float):
        import numpy as np

        delta = np.asarray(point_world, dtype=float) - np.asarray(eye, dtype=float)
        cam = np.asarray(
            [
                float(delta @ right_axis),
                float(delta @ up_axis),
                float(delta @ forward_axis),
            ],
            dtype=float,
        )
        if cam[2] <= 0.08:
            return None
        return QtCore.QPointF(
            float(viewport.center().x() + (cam[0] / cam[2]) * focal),
            float(viewport.center().y() - (cam[1] / cam[2]) * focal),
        )

    def _build_scene_meshes(self, scene: dict[str, Any]) -> list[dict[str, Any]]:
        import numpy as np

        scene_meshes = []
        for visual in scene.get("visuals", []):
            geometry = _load_geometry(visual["geometry"])
            transform = np.asarray(visual["transform"], dtype=float)
            triangles_local = geometry["triangles"]
            triangles_world = _apply_transform_to_triangles(triangles_local, transform)
            rotation = transform[:3, :3]
            normals_world = _normalize_rows(geometry["normals"] @ rotation.T)
            scene_meshes.append(
                {
                    "link": visual["link"],
                    "triangles_world": triangles_world,
                    "normals_world": normals_world,
                    "color": _LINK_COLORS.get(visual["link"], "#cbd5e1"),
                }
            )
        return scene_meshes

    def _fit_orbit_target(self) -> None:
        import numpy as np

        if not self._scene_meshes:
            return
        points = np.concatenate([mesh["triangles_world"].reshape(-1, 3) for mesh in self._scene_meshes], axis=0)
        mins = points.min(axis=0)
        maxs = points.max(axis=0)
        center = (mins + maxs) * 0.5
        center[2] = max(center[2], 0.8)
        radius = float(np.linalg.norm(maxs - mins))
        self._orbit_target = [float(v) for v in center]
        self._distance = max(3.6, min(8.0, radius * 1.15))

    def _camera_axes(self):
        import numpy as np

        target = np.asarray(self._orbit_target, dtype=float)
        eye = target + np.asarray(
            [
                self._distance * math.cos(self._pitch) * math.cos(self._yaw),
                self._distance * math.cos(self._pitch) * math.sin(self._yaw),
                self._distance * math.sin(self._pitch),
            ],
            dtype=float,
        )
        forward = _normalize_vector(target - eye)
        world_up = np.asarray([0.0, 0.0, 1.0], dtype=float)
        right_axis = _normalize_vector(np.cross(forward, world_up))
        if float(np.linalg.norm(right_axis)) < 1e-6:
            right_axis = np.asarray([1.0, 0.0, 0.0], dtype=float)
        up_axis = _normalize_vector(np.cross(right_axis, forward))
        return eye, right_axis, up_axis, forward

    @staticmethod
    def _world_to_camera(triangles_world, eye, right_axis, up_axis, forward_axis):
        import numpy as np

        deltas = triangles_world - eye.reshape(1, 1, 3)
        camera_x = np.tensordot(deltas, right_axis, axes=([2], [0]))
        camera_y = np.tensordot(deltas, up_axis, axes=([2], [0]))
        camera_z = np.tensordot(deltas, forward_axis, axes=([2], [0]))
        return np.stack([camera_x, camera_y, camera_z], axis=2)


def _load_geometry(geometry_spec: dict[str, Any]) -> dict[str, Any]:
    cache_key = json.dumps(geometry_spec, sort_keys=True)
    cached = _GEOMETRY_CACHE.get(cache_key)
    if cached is not None:
        return cached

    geometry_type = geometry_spec["type"]
    if geometry_type == "mesh":
        triangles = _load_stl_triangles(
            geometry_spec["path"],
            scale=tuple(float(v) for v in geometry_spec.get("scale", [1.0, 1.0, 1.0])),
            max_faces=2400,
        )
    elif geometry_type == "box":
        triangles = _make_box_triangles(geometry_spec["size"])
    elif geometry_type == "cylinder":
        triangles = _make_cylinder_triangles(geometry_spec["radius"], geometry_spec["length"])
    elif geometry_type == "sphere":
        triangles = _make_sphere_triangles(geometry_spec["radius"])
    else:
        raise ValueError(f"unsupported geometry type: {geometry_type}")

    normals = _compute_face_normals(triangles)
    cached = {"triangles": triangles, "normals": normals}
    _GEOMETRY_CACHE[cache_key] = cached
    return cached


def _load_stl_triangles(path: str, *, scale: tuple[float, float, float], max_faces: int):
    import numpy as np

    with open(path, "rb") as stream:
        payload = stream.read()

    triangles = None
    if len(payload) >= 84:
        face_count = struct.unpack("<I", payload[80:84])[0]
        expected_size = 84 + face_count * 50
        if expected_size == len(payload):
            dtype = np.dtype(
                [
                    ("normal", "<f4", (3,)),
                    ("v0", "<f4", (3,)),
                    ("v1", "<f4", (3,)),
                    ("v2", "<f4", (3,)),
                    ("attr", "<u2"),
                ]
            )
            records = np.frombuffer(payload, dtype=dtype, offset=84, count=face_count)
            triangles = np.stack([records["v0"], records["v1"], records["v2"]], axis=1).astype(np.float64)

    if triangles is None:
        triangles = _load_ascii_stl_triangles(payload.decode("utf-8", errors="ignore"))

    if triangles.shape[0] > max_faces:
        step = int(math.ceil(triangles.shape[0] / max_faces))
        triangles = triangles[::step]

    scale_array = np.asarray(scale, dtype=float).reshape(1, 1, 3)
    return triangles * scale_array


def _load_ascii_stl_triangles(payload_text: str):
    import numpy as np

    vertices = []
    for line in payload_text.splitlines():
        line = line.strip()
        if not line.startswith("vertex "):
            continue
        _, x, y, z = line.split()
        vertices.append([float(x), float(y), float(z)])
    if len(vertices) % 3 != 0 or not vertices:
        raise ValueError("failed to parse ASCII STL")
    return np.asarray(vertices, dtype=float).reshape(-1, 3, 3)


def _make_box_triangles(size_xyz):
    import numpy as np

    sx, sy, sz = [float(v) * 0.5 for v in size_xyz]
    vertices = np.asarray(
        [
            [-sx, -sy, -sz],
            [sx, -sy, -sz],
            [sx, sy, -sz],
            [-sx, sy, -sz],
            [-sx, -sy, sz],
            [sx, -sy, sz],
            [sx, sy, sz],
            [-sx, sy, sz],
        ],
        dtype=float,
    )
    faces = [
        [0, 1, 2], [0, 2, 3],
        [4, 6, 5], [4, 7, 6],
        [0, 4, 5], [0, 5, 1],
        [1, 5, 6], [1, 6, 2],
        [2, 6, 7], [2, 7, 3],
        [3, 7, 4], [3, 4, 0],
    ]
    return vertices[np.asarray(faces, dtype=int)]


def _make_cylinder_triangles(radius: float, length: float, *, segments: int = 18):
    import numpy as np

    triangles = []
    half = float(length) * 0.5
    for index in range(segments):
        angle_a = 2.0 * math.pi * index / segments
        angle_b = 2.0 * math.pi * (index + 1) / segments
        xa, ya = float(radius) * math.cos(angle_a), float(radius) * math.sin(angle_a)
        xb, yb = float(radius) * math.cos(angle_b), float(radius) * math.sin(angle_b)
        triangles.append([[xa, ya, -half], [xb, yb, -half], [xb, yb, half]])
        triangles.append([[xa, ya, -half], [xb, yb, half], [xa, ya, half]])
        triangles.append([[0.0, 0.0, -half], [xb, yb, -half], [xa, ya, -half]])
        triangles.append([[0.0, 0.0, half], [xa, ya, half], [xb, yb, half]])
    return np.asarray(triangles, dtype=float)


def _make_sphere_triangles(radius: float, *, lat_steps: int = 10, lon_steps: int = 16):
    import numpy as np

    triangles = []
    for lat in range(lat_steps):
        phi_a = math.pi * lat / lat_steps - math.pi * 0.5
        phi_b = math.pi * (lat + 1) / lat_steps - math.pi * 0.5
        for lon in range(lon_steps):
            theta_a = 2.0 * math.pi * lon / lon_steps
            theta_b = 2.0 * math.pi * (lon + 1) / lon_steps
            p00 = _sphere_point(radius, phi_a, theta_a)
            p01 = _sphere_point(radius, phi_a, theta_b)
            p10 = _sphere_point(radius, phi_b, theta_a)
            p11 = _sphere_point(radius, phi_b, theta_b)
            triangles.append([p00, p10, p11])
            triangles.append([p00, p11, p01])
    return np.asarray(triangles, dtype=float)


def _sphere_point(radius: float, phi: float, theta: float) -> list[float]:
    cos_phi = math.cos(phi)
    return [
        float(radius) * cos_phi * math.cos(theta),
        float(radius) * cos_phi * math.sin(theta),
        float(radius) * math.sin(phi),
    ]


def _compute_face_normals(triangles):
    import numpy as np

    edge_a = triangles[:, 1, :] - triangles[:, 0, :]
    edge_b = triangles[:, 2, :] - triangles[:, 0, :]
    normals = np.cross(edge_a, edge_b)
    return _normalize_rows(normals)


def _apply_transform_to_triangles(triangles, transform):
    import numpy as np

    flat_points = triangles.reshape(-1, 3)
    homogeneous = np.concatenate([flat_points, np.ones((flat_points.shape[0], 1), dtype=float)], axis=1)
    transformed = homogeneous @ np.asarray(transform, dtype=float).T
    return transformed[:, :3].reshape(-1, 3, 3)


def _normalize_rows(vectors):
    import numpy as np

    norms = np.linalg.norm(vectors, axis=1, keepdims=True)
    norms[norms < 1e-9] = 1.0
    return vectors / norms


def _normalize_vector(vector):
    import numpy as np

    norm = float(np.linalg.norm(vector))
    if norm < 1e-9:
        return vector
    return vector / norm


def _shade_color(base_color: QtGui.QColor, intensity: float) -> QtGui.QColor:
    hsv = list(base_color.getHsvF())
    hsv[2] = max(0.12, min(1.0, 0.22 + intensity * 0.88))
    hsv[1] = max(0.05, min(1.0, hsv[1] * 0.72 + 0.18))
    shaded = QtGui.QColor()
    shaded.setHsvF(hsv[0], hsv[1], hsv[2], 1.0)
    return shaded


def _format_camera_label(device: str, info: dict[str, Any]) -> str:
    product = info.get("usb_product")
    if product:
        return f"{device}  |  {product}"
    return device


class MainWindow(QtWidgets.QMainWindow):
    def __init__(self, config: dict[str, Any], *, headless_smoke_test: bool = False):
        super().__init__()
        self._config = config
        self._headless_smoke_test = headless_smoke_test
        self._received_status = False
        self._last_image_timestamp = {"front": 0.0, "wrist": 0.0}
        self._motion_process: QtCore.QProcess | None = None
        self.setWindowTitle("ws_RAMS Industrial HMI")
        self.resize(1660, 980)
        self._build_ui()
        self._apply_style()
        self._start_monitor()

    def _build_ui(self) -> None:
        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        root = QtWidgets.QVBoxLayout(central)
        root.setContentsMargins(18, 18, 18, 18)
        root.setSpacing(14)

        header = QtWidgets.QFrame()
        header.setObjectName("Header")
        header_layout = QtWidgets.QVBoxLayout(header)
        header_layout.setContentsMargins(22, 20, 22, 20)
        self.title_label = QtWidgets.QLabel("ws_RAMS Industrial HMI")
        self.title_label.setObjectName("HeaderTitle")
        subtitle = QtWidgets.QLabel("ABB + dual RealSense + remote pi0 monitor with guarded limited-motion test controls")
        subtitle.setObjectName("HeaderSubtitle")
        badge_row = QtWidgets.QHBoxLayout()
        badge_row.setSpacing(10)
        self.robot_badge = StatusBadge("Robot RWS")
        self.camera_badge = StatusBadge("Cameras")
        self.policy_badge = StatusBadge("Policy")
        self.safety_badge = StatusBadge("Safety")
        for badge in (self.robot_badge, self.camera_badge, self.policy_badge, self.safety_badge):
            badge_row.addWidget(badge)
        badge_row.addStretch(1)
        header_layout.addWidget(self.title_label)
        header_layout.addWidget(subtitle)
        header_layout.addLayout(badge_row)
        root.addWidget(header)

        overview_row = QtWidgets.QHBoxLayout()
        overview_row.setSpacing(12)
        self.robot_overview = self._make_monitor_panel(
            "ABB Read-Only",
            "RWS jointtarget, robtarget, TCP summary, no motion commands issued.",
        )
        self.camera_overview = self._make_monitor_panel(
            "Dual RealSense",
            "Device identity, live RGB feed heartbeat, and front/wrist assignment.",
        )
        self.policy_overview = self._make_monitor_panel(
            "Remote pi0",
            "Hold adapter, official adapter, and Tailscale reachability.",
        )
        self.safety_overview = self._make_monitor_panel(
            "Safety / Bringup",
            "Bridge defaults to publish_commands:=false unless you explicitly arm it.",
        )
        for panel in (self.robot_overview, self.camera_overview, self.policy_overview, self.safety_overview):
            overview_row.addWidget(panel, 1)
        root.addLayout(overview_row)

        splitter = QtWidgets.QSplitter(QtCore.Qt.Horizontal)
        root.addWidget(splitter, 1)

        left_widget = QtWidgets.QWidget()
        left_layout = QtWidgets.QVBoxLayout(left_widget)
        left_layout.setContentsMargins(0, 0, 0, 0)
        left_layout.setSpacing(12)
        robot_panel = InfoPanel("Robot Pose")
        self.robot_summary = QtWidgets.QLabel("Waiting for RWS...")
        self.robot_summary.setObjectName("MetaLabel")
        robot_panel.layout_.addWidget(self.robot_summary)
        self.robot_canvas = RobotCanvas()
        robot_panel.layout_.addWidget(self.robot_canvas, 1)
        left_layout.addWidget(robot_panel, 1)

        joints_panel = InfoPanel("Joint State")
        self.joint_table = QtWidgets.QTableWidget(6, 3)
        self.joint_table.setHorizontalHeaderLabels(["Joint", "deg", "rad"])
        self.joint_table.verticalHeader().setVisible(False)
        self.joint_table.horizontalHeader().setStretchLastSection(True)
        self.joint_table.horizontalHeader().setSectionResizeMode(QtWidgets.QHeaderView.ResizeToContents)
        self.joint_table.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        self.joint_table.setSelectionMode(QtWidgets.QAbstractItemView.NoSelection)
        self.joint_table.setFocusPolicy(QtCore.Qt.NoFocus)
        joints_panel.layout_.addWidget(self.joint_table)
        left_layout.addWidget(joints_panel, 0)

        right_widget = QtWidgets.QWidget()
        right_layout = QtWidgets.QVBoxLayout(right_widget)
        right_layout.setContentsMargins(0, 0, 0, 0)
        right_layout.setSpacing(12)

        cameras_row = QtWidgets.QHBoxLayout()
        self.front_camera = CameraPanel("Fixed / Front Camera")
        self.wrist_camera = CameraPanel("Wrist Camera")
        cameras_row.addWidget(self.front_camera, 1)
        cameras_row.addWidget(self.wrist_camera, 1)
        right_layout.addLayout(cameras_row, 1)

        policy_panel = InfoPanel("Policy + Network")
        self.policy_table = QtWidgets.QTableWidget(4, 2)
        self.policy_table.setHorizontalHeaderLabels(["Signal", "Value"])
        self.policy_table.verticalHeader().setVisible(False)
        self.policy_table.horizontalHeader().setStretchLastSection(True)
        self.policy_table.horizontalHeader().setSectionResizeMode(QtWidgets.QHeaderView.ResizeToContents)
        self.policy_table.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        self.policy_table.setSelectionMode(QtWidgets.QAbstractItemView.NoSelection)
        self.policy_table.setFocusPolicy(QtCore.Qt.NoFocus)
        policy_panel.layout_.addWidget(self.policy_table)
        right_layout.addWidget(policy_panel)

        motion_panel = InfoPanel("Limited Motion Control")
        motion_panel.layout_.addWidget(self._build_motion_controls())
        right_layout.addWidget(motion_panel, 0)

        diagnostics = QtWidgets.QTabWidget()
        self.raw_text = QtWidgets.QPlainTextEdit()
        self.raw_text.setReadOnly(True)
        diagnostics.addTab(self.raw_text, "Raw Snapshot")
        self.events_text = QtWidgets.QPlainTextEdit()
        self.events_text.setReadOnly(True)
        diagnostics.addTab(self.events_text, "Events")
        right_layout.addWidget(diagnostics, 1)

        splitter.addWidget(left_widget)
        splitter.addWidget(right_widget)
        splitter.setSizes([860, 760])

    def _build_motion_controls(self) -> QtWidgets.QWidget:
        widget = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(widget)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(10)

        form = QtWidgets.QGridLayout()
        form.setHorizontalSpacing(12)
        form.setVerticalSpacing(8)

        self.motion_direction_combo = QtWidgets.QComboBox()
        for label, vector in _MOTION_DIRECTIONS:
            self.motion_direction_combo.addItem(label, vector)

        self.motion_speed_spin = QtWidgets.QDoubleSpinBox()
        self.motion_speed_spin.setDecimals(1)
        self.motion_speed_spin.setRange(0.2, 3.0)
        self.motion_speed_spin.setSingleStep(0.2)
        self.motion_speed_spin.setSuffix(" mm/s")
        self.motion_speed_spin.setValue(2.0)

        self.motion_duration_spin = QtWidgets.QDoubleSpinBox()
        self.motion_duration_spin.setDecimals(1)
        self.motion_duration_spin.setRange(0.5, 120.0)
        self.motion_duration_spin.setSingleStep(1.0)
        self.motion_duration_spin.setSuffix(" s")
        self.motion_duration_spin.setValue(5.0)

        self.motion_distance_label = QtWidgets.QLabel("-")
        self.motion_distance_label.setObjectName("MotionDistance")

        self.motion_confirm = QtWidgets.QCheckBox(
            "I confirm the ABB RAPID EGM program is running at the operator-defined egm_ready point."
        )
        self.motion_confirm.setObjectName("MotionConfirm")

        form.addWidget(QtWidgets.QLabel("Direction"), 0, 0)
        form.addWidget(self.motion_direction_combo, 0, 1)
        form.addWidget(QtWidgets.QLabel("Speed"), 1, 0)
        form.addWidget(self.motion_speed_spin, 1, 1)
        form.addWidget(QtWidgets.QLabel("Duration"), 2, 0)
        form.addWidget(self.motion_duration_spin, 2, 1)
        form.addWidget(QtWidgets.QLabel("Expected travel"), 3, 0)
        form.addWidget(self.motion_distance_label, 3, 1)
        layout.addLayout(form)
        layout.addWidget(self.motion_confirm)

        button_row = QtWidgets.QHBoxLayout()
        self.motion_start_button = QtWidgets.QPushButton("START LIMITED MOTION")
        self.motion_start_button.setObjectName("MotionStart")
        self.motion_stop_button = QtWidgets.QPushButton("STOP / DISARM")
        self.motion_stop_button.setObjectName("MotionStop")
        self.motion_stop_button.setEnabled(False)
        button_row.addWidget(self.motion_start_button, 1)
        button_row.addWidget(self.motion_stop_button, 1)
        layout.addLayout(button_row)

        self.motion_status_label = QtWidgets.QLabel(
            "Uses tools/run_cartesian_left_test.sh with auto-disarm and safe pi05 restore."
        )
        self.motion_status_label.setWordWrap(True)
        self.motion_status_label.setObjectName("MonitorDetail")
        layout.addWidget(self.motion_status_label)

        self.motion_log = QtWidgets.QPlainTextEdit()
        self.motion_log.setReadOnly(True)
        self.motion_log.setMaximumHeight(120)
        self.motion_log.setPlaceholderText("Motion script output will appear here.")
        layout.addWidget(self.motion_log)

        self.motion_speed_spin.valueChanged.connect(self._update_motion_distance)
        self.motion_duration_spin.valueChanged.connect(self._update_motion_distance)
        self.motion_start_button.clicked.connect(self._start_limited_motion)
        self.motion_stop_button.clicked.connect(self._stop_limited_motion)
        self._update_motion_distance()
        return widget

    def _make_monitor_panel(self, title: str, detail: str) -> InfoPanel:
        panel = InfoPanel(title)
        state = QtWidgets.QLabel("Waiting for data...")
        state.setObjectName("MonitorState")
        state.setWordWrap(True)
        detail_label = QtWidgets.QLabel(detail)
        detail_label.setObjectName("MonitorDetail")
        detail_label.setWordWrap(True)
        panel.layout_.addWidget(state)
        panel.layout_.addWidget(detail_label)
        panel.state_label = state  # type: ignore[attr-defined]
        panel.detail_label = detail_label  # type: ignore[attr-defined]
        return panel

    def _apply_style(self) -> None:
        self.setStyleSheet(
            """
            QWidget { background: #0b1220; color: #e5edf7; font-family: 'Segoe UI', 'Noto Sans', sans-serif; }
            QMainWindow { background: #08101c; }
            #Header, #InfoPanel, #StatusBadge { background: #0f1726; border: 1px solid #2b3b5d; border-radius: 0px; }
            #HeaderTitle { font-size: 30px; font-weight: 900; color: #f8fafc; }
            #HeaderSubtitle { color: #94a3b8; font-size: 14px; }
            #BadgeTitle { color: #cbd5e1; font-weight: 700; }
            #BadgeValue { color: #f8fafc; font-weight: 900; }
            #PanelTitle { font-size: 18px; font-weight: 800; color: #f8fafc; }
            #MetaLabel { color: #94a3b8; font-size: 12px; }
            #MonitorState { color: #f8fafc; font-size: 18px; font-weight: 900; }
            #MonitorDetail { color: #8ea3bb; font-size: 12px; }
            #MotionDistance { color: #fbbf24; font-weight: 900; }
            #MotionConfirm { color: #cbd5e1; font-weight: 700; }
            #MotionStart { background: #166534; color: #f8fafc; border: 1px solid #22c55e; padding: 12px; font-weight: 900; border-radius: 0px; }
            #MotionStart:disabled { background: #1f2937; color: #64748b; border-color: #334155; }
            #MotionStop { background: #7f1d1d; color: #f8fafc; border: 1px solid #ef4444; padding: 12px; font-weight: 900; border-radius: 0px; }
            #MotionStop:disabled { background: #1f2937; color: #64748b; border-color: #334155; }
            QComboBox, QDoubleSpinBox { background: #0d1728; color: #f8fafc; border: 1px solid #2b3b5d; padding: 8px; border-radius: 0px; }
            QCheckBox::indicator { width: 16px; height: 16px; border: 1px solid #64748b; background: #0d1728; border-radius: 0px; }
            QCheckBox::indicator:checked { background: #22c55e; border: 1px solid #86efac; }
            #CameraImage { background: #030712; border: 1px solid #2b3b5d; border-radius: 0px; }
            QHeaderView::section { background: #16233f; color: #cbd5e1; border: 1px solid #22314f; padding: 8px; font-weight: 800; }
            QTableWidget { background: #0d1728; border: 1px solid #2b3b5d; border-radius: 0px; gridline-color: #22314f; }
            QTableWidget::item { padding: 8px; }
            QPlainTextEdit, QTabWidget::pane { background: #0d1728; border: 1px solid #2b3b5d; border-radius: 0px; }
            QTabBar::tab { background: #111b2e; padding: 10px 16px; border: 1px solid #2b3b5d; border-bottom: 0; border-radius: 0px; }
            QTabBar::tab:selected { background: #1b2a44; color: #f8fafc; }
            """
        )

    def _start_monitor(self) -> None:
        self._thread = MonitorThread(self._config)
        self._thread.status_ready.connect(self._apply_status)
        self._thread.image_ready.connect(self._apply_image)
        self._thread.worker_error.connect(self._append_event)
        self._thread.start()

    def shutdown_monitor(self) -> None:
        if hasattr(self, "_thread"):
            self._thread.requestInterruption()
            self._thread.wait(1500)

    def closeEvent(self, event: QtGui.QCloseEvent) -> None:
        if self._motion_process is not None and self._motion_process.state() != QtCore.QProcess.NotRunning:
            self._motion_process.terminate()
        self.shutdown_monitor()
        super().closeEvent(event)

    def _update_motion_distance(self) -> None:
        distance_mm = self.motion_speed_spin.value() * self.motion_duration_spin.value()
        distance_m = distance_mm / 1000.0
        limit_text = "OK" if distance_m <= 0.25 else "ABOVE HMI LIMIT"
        self.motion_distance_label.setText(f"{distance_mm:.1f} mm  |  {limit_text}")

    def _start_limited_motion(self) -> None:
        if self._motion_process is not None and self._motion_process.state() != QtCore.QProcess.NotRunning:
            self._append_motion_log("Motion command already running.")
            return
        if not self.motion_confirm.isChecked():
            self.motion_status_label.setText("Start blocked: confirm RAPID EGM is running at the operator-defined egm_ready point.")
            self._append_motion_log("Start blocked: confirmation checkbox is not checked.")
            return

        speed_mps = self.motion_speed_spin.value() / 1000.0
        duration_s = self.motion_duration_spin.value()
        distance_m = speed_mps * duration_s
        if distance_m > 0.25:
            self.motion_status_label.setText(
                f"Start blocked: requested travel {distance_m * 1000.0:.1f} mm exceeds the HMI limit of 250 mm."
            )
            self._append_motion_log("Reduce speed or duration before starting.")
            return

        direction = self.motion_direction_combo.currentData()
        direction_xyz = ",".join(f"{float(value):.6g}" for value in direction)
        repo_root = Path(__file__).resolve().parents[1]
        script = repo_root / "tools" / "run_cartesian_left_test.sh"
        args = [
            "bash",
            str(script),
            "--execute",
            "--policy-backend",
            "cartesian_left_true_servo",
            "--duration-sec",
            f"{duration_s:.3f}",
            "--speed-mps",
            f"{speed_mps:.6f}",
            "--tracking-gain",
            "1.0",
            "--max-cartesian-error-m",
            "0.02",
            "--max-joint-delta-rad",
            "0.008",
            "--bridge-max-position-step-rad",
            "0.1",
            "--bridge-joint-state-topic",
            "/joint_states",
            "--bridge-feedback-joint-state-topic",
            "/abb_rws/joint_states",
            "--wait-for-egm-ready-sec",
            "180",
            "--direction-xyz",
            direction_xyz,
            "--runtime-dir",
            "/tmp/ws_rams_hmi_motion_control",
        ]

        self.motion_log.clear()
        self._append_motion_log("Starting limited motion command.")
        self._append_motion_log(f"Direction: {self.motion_direction_combo.currentText()} [{direction_xyz}]")
        self._append_motion_log(f"Speed: {speed_mps:.6f} m/s, duration: {duration_s:.3f} s")
        self.motion_status_label.setText("Motion script running. If EGM is not ready, restart ABB RAPID while the script waits.")
        self.motion_start_button.setEnabled(False)
        self.motion_stop_button.setEnabled(True)

        process = QtCore.QProcess(self)
        process.setWorkingDirectory(str(repo_root))
        process.setProcessChannelMode(QtCore.QProcess.MergedChannels)
        process.readyReadStandardOutput.connect(self._read_motion_process_output)
        process.finished.connect(self._motion_process_finished)
        process.errorOccurred.connect(self._motion_process_error)
        self._motion_process = process
        process.start("/usr/bin/env", args)

    def _stop_limited_motion(self) -> None:
        process = self._motion_process
        if process is None or process.state() == QtCore.QProcess.NotRunning:
            self._append_motion_log("No motion command is running.")
            return
        self.motion_status_label.setText("Stop requested. Waiting for script cleanup and auto-disarm.")
        self._append_motion_log("STOP requested from HMI; terminating motion script.")
        process.terminate()
        QtCore.QTimer.singleShot(20000, self._kill_motion_process_if_needed)

    def _kill_motion_process_if_needed(self) -> None:
        process = self._motion_process
        if process is not None and process.state() != QtCore.QProcess.NotRunning:
            self._append_motion_log("Motion script did not exit after 20s; killing process.")
            process.kill()

    def _read_motion_process_output(self) -> None:
        process = self._motion_process
        if process is None:
            return
        payload = bytes(process.readAllStandardOutput()).decode("utf-8", errors="replace")
        for line in payload.splitlines():
            self._append_motion_log(line)

    def _motion_process_finished(self, exit_code: int, exit_status: QtCore.QProcess.ExitStatus) -> None:
        self._read_motion_process_output()
        status_text = "completed" if exit_code == 0 and exit_status == QtCore.QProcess.NormalExit else "ended with error"
        self._append_motion_log(f"Motion script {status_text}; exit_code={exit_code}, exit_status={int(exit_status)}")
        self.motion_status_label.setText(f"Motion script {status_text}. Bridge should have restored publish_commands:=false.")
        self.motion_start_button.setEnabled(True)
        self.motion_stop_button.setEnabled(False)
        if self._motion_process is not None:
            self._motion_process.deleteLater()
            self._motion_process = None

    def _motion_process_error(self, error: QtCore.QProcess.ProcessError) -> None:
        self._append_motion_log(f"QProcess error: {int(error)}")
        self.motion_status_label.setText("Motion process failed to start or crashed before cleanup.")
        self.motion_start_button.setEnabled(True)
        self.motion_stop_button.setEnabled(False)

    def _append_motion_log(self, text: str) -> None:
        stamp = time.strftime("%H:%M:%S")
        if hasattr(self, "motion_log"):
            self.motion_log.appendPlainText(f"[{stamp}] {text}")
        if hasattr(self, "events_text"):
            self.events_text.appendPlainText(f"[{stamp}] MOTION {text}")

    @QtCore.Slot(dict)
    def _apply_status(self, data: dict[str, Any]) -> None:
        self._received_status = True
        joint = data["robot"]["jointtarget"]
        cart = data["robot"]["cartesian"]
        scene_model = data["robot"]["scene_model"]
        self.robot_badge.set_status("good" if joint.get("ok") and cart.get("ok") else "bad", "OK" if joint.get("ok") and cart.get("ok") else "ISSUE")
        self.camera_badge.set_status("good" if data["cameras"].get("realsense_count", 0) >= 2 else "warn", str(data["cameras"].get("realsense_count", 0)))
        policy_ok = bool(data["policy"]["hold"].get("ok") or data["policy"]["official"].get("ok"))
        self.policy_badge.set_status("good" if policy_ok else "bad", "ONLINE" if policy_ok else "DOWN")
        self.safety_badge.set_status("good" if data["control_processes"].get("ok") else "warn", "READ ONLY" if data["control_processes"].get("ok") else "CHECK")

        if cart.get("position_mm"):
            p = cart["position_mm"]
            tcp_text = f"TCP x={p['x']:.1f} y={p['y']:.1f} z={p['z']:.1f} mm"
        else:
            tcp_text = "TCP unavailable"
        self.robot_summary.setText(f"{data['timestamp_text']}  |  {tcp_text}")
        self.robot_canvas.update_model(
            scene_model,
            tcp_text,
            workspace_radius_m=float(data["config"].get("workspace_radius_m", 0.5)),
        )

        degrees = joint.get("degrees") or [0.0] * 6
        radians = joint.get("radians") or [0.0] * 6
        for row in range(6):
            self._set_table_item(self.joint_table, row, 0, f"J{row + 1}")
            self._set_table_item(self.joint_table, row, 1, f"{degrees[row]:+.3f}")
            self._set_table_item(self.joint_table, row, 2, f"{radians[row]:+.6f}")

        front_info = data["cameras"].get("front_device_info") or {}
        wrist_info = data["cameras"].get("wrist_device_info") or {}
        self.front_camera.set_device(_format_camera_label(data["cameras"].get("front_device", "-"), front_info))
        self.wrist_camera.set_device(_format_camera_label(data["cameras"].get("wrist_device", "-"), wrist_info))
        self.front_camera.set_status_text(self._format_camera_status("front", front_info))
        self.wrist_camera.set_status_text(self._format_camera_status("wrist", wrist_info))

        policy_rows = [
            ("ABB RWS", self._format_rws_row(joint, cart)),
            ("Hold adapter", self._format_policy_row(data["policy"]["hold"])),
            ("Official adapter", self._format_policy_row(data["policy"]["official"])),
            ("Official infer rate", self._format_policy_stats_row(data["policy"]["official"])),
            ("Tailscale", " | ".join(data["tailscale"].get("remote_lines") or ["not found"])),
            ("RealSense groups", str(data["cameras"].get("realsense_count", 0))),
            ("Front camera", front_info.get("usb_product") or front_info.get("name") or "unknown"),
            ("Wrist camera", wrist_info.get("usb_product") or wrist_info.get("name") or "unknown"),
            ("Control watch", "clean" if data["control_processes"].get("ok") else f"{len(data['control_processes'].get('processes', []))} proc"),
        ]
        self.policy_table.setRowCount(len(policy_rows))
        for row, (key, value) in enumerate(policy_rows):
            self._set_table_item(self.policy_table, row, 0, key)
            self._set_table_item(self.policy_table, row, 1, value)

        self._apply_overview_panels(data, tcp_text)
        self.raw_text.setPlainText(json.dumps(data, indent=2, ensure_ascii=False))
        if self._headless_smoke_test:
            print("HMI_SMOKE_OK", joint.get("ok"), data["cameras"].get("realsense_count", 0), policy_ok)
            QtCore.QTimer.singleShot(100, QtWidgets.QApplication.instance().quit)

    @QtCore.Slot(str, bytes)
    def _apply_image(self, name: str, payload: bytes) -> None:
        self._last_image_timestamp[name] = time.time()
        if name == "front":
            self.front_camera.set_image_bytes(payload)
        elif name == "wrist":
            self.wrist_camera.set_image_bytes(payload)

    @QtCore.Slot(str)
    def _append_event(self, text: str) -> None:
        stamp = time.strftime("%H:%M:%S")
        self.events_text.appendPlainText(f"[{stamp}] {text}")

    def _apply_overview_panels(self, data: dict[str, Any], tcp_text: str) -> None:
        joint = data["robot"]["jointtarget"]
        cart = data["robot"]["cartesian"]
        cameras = data["cameras"]
        hold = data["policy"]["hold"]
        official = data["policy"]["official"]
        control = data["control_processes"]

        self.robot_overview.state_label.setText("ONLINE" if joint.get("ok") and cart.get("ok") else "ISSUE")
        self.robot_overview.detail_label.setText(
            f"{tcp_text}\nJoints: {'ready' if joint.get('ok') else 'missing'}  |  URDF: {'ready' if data['robot']['scene_model'].get('ok') else 'missing'}"
        )
        self.camera_overview.state_label.setText(f"{cameras.get('realsense_count', 0)} CAMERA GROUPS")
        self.camera_overview.detail_label.setText(
            f"front={cameras.get('front_device', '-')}\nwrist={cameras.get('wrist_device', '-')}\nfeeds: {self._image_heartbeat('front')} / {self._image_heartbeat('wrist')}"
        )
        self.policy_overview.state_label.setText("REACHABLE" if (hold.get("ok") or official.get("ok")) else "OFFLINE")
        self.policy_overview.detail_label.setText(
            f"hold: {self._format_policy_row(hold)}\n"
            f"official: {self._format_policy_row(official)}\n"
            f"official infer: {self._format_policy_stats_row(official)}\n"
            f"{(' | '.join(data['tailscale'].get('remote_lines') or ['tailscale not found']))}"
        )
        self.safety_overview.state_label.setText("READ ONLY" if control.get("ok") else "CHECK PROCESSES")
        self.safety_overview.detail_label.setText(
            f"publish_commands defaults to false\nworkspace radius: +/-{float(data['config'].get('workspace_radius_m', 0.5)):.2f} m\nwatchlist: {'clean' if control.get('ok') else str(len(control.get('processes', []))) + ' process(es)'}"
        )

    def _image_heartbeat(self, name: str) -> str:
        last = self._last_image_timestamp.get(name, 0.0)
        if last <= 0.0:
            return "no frame yet"
        age = max(0.0, time.time() - last)
        return f"{age:.1f}s ago"

    def _format_camera_status(self, name: str, info: dict[str, Any]) -> str:
        product = info.get("usb_product") or info.get("name") or "camera"
        return f"{product}  |  ROS topic frame {self._image_heartbeat(name)}"

    @staticmethod
    def _format_policy_row(payload: dict[str, Any]) -> str:
        ok = payload.get("ok")
        elapsed = payload.get("elapsed_ms")
        loader = ((payload.get("payload") or {}).get("policy_loader")) or "unknown"
        if ok:
            return f"OK | {loader} | {elapsed} ms"
        return f"DOWN | {elapsed} ms"

    @staticmethod
    def _format_rws_row(joint: dict[str, Any], cart: dict[str, Any]) -> str:
        if joint.get("ok") and cart.get("ok"):
            return f"OK | {joint.get('elapsed_ms')} / {cart.get('elapsed_ms')} ms"
        return "ISSUE"

    @staticmethod
    def _policy_stats(payload: dict[str, Any]) -> dict[str, Any]:
        return (payload.get("payload") or {}).get("infer_stats") or {}

    @classmethod
    def _format_policy_stats_row(cls, payload: dict[str, Any]) -> str:
        stats = cls._policy_stats(payload)
        hz = float(stats.get("recent_success_hz_30s", 0.0) or 0.0)
        count = int(stats.get("recent_success_count_30s", 0) or 0)
        total = int(stats.get("success_count_total", 0) or 0)
        return f"{hz:.2f} Hz | {count}/30s | total {total}"

    @staticmethod
    def _set_table_item(table: QtWidgets.QTableWidget, row: int, column: int, text: str) -> None:
        item = table.item(row, column)
        if item is None:
            item = QtWidgets.QTableWidgetItem()
            table.setItem(row, column, item)
        item.setText(text)


def parse_args(args: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Qt-based read-only ws_RAMS workcell HMI.")
    parser.add_argument("--rws-ip", default=backend.DEFAULT_RWS_IP)
    parser.add_argument("--rws-port", default="80")
    parser.add_argument("--rws-user", default=backend.DEFAULT_RWS_USER)
    parser.add_argument("--rws-password", default=backend.DEFAULT_RWS_PASSWORD)
    parser.add_argument("--rws-timeout-s", type=float, default=1.5)
    parser.add_argument("--remote-host", default=backend.DEFAULT_REMOTE_HOST)
    parser.add_argument("--hold-policy-health-url", default=backend.DEFAULT_HOLD_POLICY_URL)
    parser.add_argument("--official-policy-health-url", default=backend.DEFAULT_OFFICIAL_POLICY_URL)
    parser.add_argument("--front-camera-topic", default="/camera/fixed_cam/color/image_raw")
    parser.add_argument("--wrist-camera-topic", default="/camera/wrist_cam/color/image_raw")
    parser.add_argument("--front-camera-device", default=backend.DEFAULT_FRONT_CAMERA_DEVICE)
    parser.add_argument("--wrist-camera-device", default=backend.DEFAULT_WRIST_CAMERA_DEVICE)
    parser.add_argument(
        "--allow-v4l2-camera-fallback",
        action="store_true",
        help="Allow direct /dev/video preview if ROS image topics are unavailable. Disabled by default to avoid fighting realsense2_camera.",
    )
    parser.add_argument("--camera-width", type=int, default=424)
    parser.add_argument("--camera-height", type=int, default=240)
    parser.add_argument("--camera-jpeg-quality", type=int, default=80)
    parser.add_argument("--workcell-urdf", default=backend.DEFAULT_WORKCELL_URDF)
    parser.add_argument("--urdf-base-link", default=backend.DEFAULT_URDF_BASE_LINK)
    parser.add_argument("--urdf-tip-link", default=backend.DEFAULT_URDF_TIP_LINK)
    parser.add_argument("--workspace-radius-m", type=float, default=backend.DEFAULT_WORKSPACE_RADIUS_M)
    parser.add_argument("--headless-smoke-test", action="store_true")
    return parser.parse_args(args=args)


def main(args: list[str] | None = None) -> None:
    parsed = parse_args(args)
    config = backend.make_dashboard_config(
        rws_ip=parsed.rws_ip,
        rws_port=parsed.rws_port,
        rws_user=parsed.rws_user,
        rws_password=parsed.rws_password,
        rws_timeout_s=parsed.rws_timeout_s,
        remote_host=parsed.remote_host,
        hold_policy_health_url=parsed.hold_policy_health_url,
        official_policy_health_url=parsed.official_policy_health_url,
        front_camera_topic=parsed.front_camera_topic,
        wrist_camera_topic=parsed.wrist_camera_topic,
        front_camera_device=parsed.front_camera_device,
        wrist_camera_device=parsed.wrist_camera_device,
        allow_v4l2_camera_fallback=parsed.allow_v4l2_camera_fallback,
        camera_width=parsed.camera_width,
        camera_height=parsed.camera_height,
        camera_jpeg_quality=parsed.camera_jpeg_quality,
        workcell_urdf=parsed.workcell_urdf,
        urdf_base_link=parsed.urdf_base_link,
        urdf_tip_link=parsed.urdf_tip_link,
        workspace_radius_m=parsed.workspace_radius_m,
    )

    app = QtWidgets.QApplication([])
    app.setApplicationName("ws_RAMS Industrial HMI")
    window = MainWindow(config, headless_smoke_test=parsed.headless_smoke_test)
    app.aboutToQuit.connect(window.shutdown_monitor)
    if not parsed.headless_smoke_test:
        window.show()
    else:
        QtCore.QTimer.singleShot(5000, app.quit)
    app.exec_()


if __name__ == "__main__":
    main()
