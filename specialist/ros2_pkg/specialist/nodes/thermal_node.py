#!/usr/bin/env python3
"""Specialist thermal processing node with simulation fallback."""
import os, sys
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "../../../../../shared"))
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String
from qos_profiles import QOS_SENSOR

try:
    import cv2

    CV_OK = True
except ImportError:
    CV_OK = False

try:
    from cv_bridge import CvBridge
    from sensor_msgs.msg import Image

    IMAGE_OK = True
except ImportError:
    CvBridge = None
    Image = None
    IMAGE_OK = False


class ThermalNode(Node):
    HUMAN_TEMP_MIN = 34.0
    HUMAN_TEMP_MAX = 39.5
    FIRE_TEMP_MIN = 60.0

    def __init__(self):
        super().__init__("specialist_thermal")
        self.declare_parameter("device_id", 0)
        self.declare_parameter("fps", 9)
        self.declare_parameter("temp_min", 20.0)
        self.declare_parameter("temp_max", 50.0)
        self.declare_parameter("use_sim", True)

        self._use_sim = bool(self.get_parameter("use_sim").value)
        self._bridge = CvBridge() if IMAGE_OK else None
        self._cap = None
        self._pub_img = (
            self.create_publisher(Image, "/specialist/thermal_image", QOS_SENSOR) if IMAGE_OK else None
        )
        self._pub_persons = self.create_publisher(Float32MultiArray, "/specialist/thermal_persons", QOS_SENSOR)
        self._pub_fire = self.create_publisher(Float32MultiArray, "/specialist/fire_points", QOS_SENSOR)
        self._pub_status = self.create_publisher(String, "/specialist/thermal_status", 10)
        self.create_timer(1.0 / float(self.get_parameter("fps").value), self._process_frame)

    def _get_frame(self):
        if self._use_sim or not CV_OK:
            frame = np.random.uniform(20.0, 30.0, (120, 160)).astype(np.float32)
            frame[40:70, 60:90] = np.random.uniform(35.0, 38.0, (30, 30))
            frame[90:110, 130:155] = np.random.uniform(70.0, 90.0, (20, 25))
            return frame

        if self._cap is None:
            self._cap = cv2.VideoCapture(int(self.get_parameter("device_id").value))
        ret, frame = self._cap.read()
        if not ret or frame is None:
            self._cap.release()
            self._cap = None
            return np.full((120, 160), 25.0, np.float32)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY).astype(np.float32)
        tmin = float(self.get_parameter("temp_min").value)
        tmax = float(self.get_parameter("temp_max").value)
        return gray / 255.0 * (tmax - tmin) + tmin

    def _process_frame(self):
        temp = self._get_frame()
        persons = self._regions(temp, self.HUMAN_TEMP_MIN, self.HUMAN_TEMP_MAX)
        fires = self._regions(temp, self.FIRE_TEMP_MIN, 200.0)

        self._pub_persons.publish(Float32MultiArray(data=self._flatten_regions(persons)))
        self._pub_fire.publish(Float32MultiArray(data=self._flatten_regions(fires)))

        status = String()
        status.data = f"persons={len(persons)} fires={len(fires)} max_temp={float(np.max(temp)):.1f}C"
        self._pub_status.publish(status)

        if self._pub_img and CV_OK:
            tmin = float(self.get_parameter("temp_min").value)
            tmax = float(self.get_parameter("temp_max").value)
            norm = np.clip((temp - tmin) / (tmax - tmin), 0.0, 1.0)
            img = (norm * 255.0).astype(np.uint8)
            color = cv2.applyColorMap(img, cv2.COLORMAP_INFERNO)
            self._pub_img.publish(self._bridge.cv2_to_imgmsg(color, encoding="bgr8"))

    def _regions(self, temp, lo, hi):
        mask = (temp >= lo) & (temp <= hi)
        return self._connected_components(temp, mask)

    @staticmethod
    def _connected_components(temp, mask, min_pixels=4):
        h, w = mask.shape
        visited = np.zeros((h, w), dtype=np.bool_)
        regions = []
        for y0 in range(h):
            for x0 in range(w):
                if not mask[y0, x0] or visited[y0, x0]:
                    continue
                stack = [(y0, x0)]
                visited[y0, x0] = True
                pixels_y, pixels_x = [], []
                while stack:
                    y, x = stack.pop()
                    pixels_y.append(y)
                    pixels_x.append(x)
                    for dy, dx in [(-1,0),(1,0),(0,-1),(0,1)]:
                        ny, nx = y + dy, x + dx
                        if 0 <= ny < h and 0 <= nx < w:
                            if mask[ny, nx] and not visited[ny, nx]:
                                visited[ny, nx] = True
                                stack.append((ny, nx))
                if len(pixels_y) >= min_pixels:
                    ys_arr = np.array(pixels_y)
                    xs_arr = np.array(pixels_x)
                    temps = temp[ys_arr, xs_arr]
                    regions.append((
                        float(np.mean(xs_arr)),
                        float(np.mean(ys_arr)),
                        float(np.mean(temps)),
                        len(pixels_y),
                    ))
        return regions

    @staticmethod
    def _flatten_regions(regions):
        out = []
        for region in regions:
            out.extend(region)
        return out


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(ThermalNode())
    rclpy.shutdown()
