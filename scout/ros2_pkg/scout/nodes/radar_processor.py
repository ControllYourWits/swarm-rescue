#!/usr/bin/env python3
"""AWR1642 life-sign processor with simulation fallback."""
import math
import threading
import time

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32MultiArray, String

try:
    import serial
except ImportError:
    serial = None

try:
    from swarm_msgs.msg import LifeSign

    CUSTOM_MSG = True
except ImportError:
    LifeSign = None
    CUSTOM_MSG = False


class RadarProcessor(Node):
    FPS = 30
    DURATION = 3.0
    BREATH_LO = 0.1
    BREATH_HI = 0.5
    HEART_LO = 1.0
    HEART_HI = 2.0
    BREATH_SNR_TH = 10.0
    HEART_SNR_TH = 8.0

    def __init__(self):
        super().__init__("radar_processor")
        self.declare_parameter("port", "/dev/ttyUSB1")
        self.declare_parameter("baud", 921600)
        self.declare_parameter("max_range", 5.0)
        self._max_r = float(self.get_parameter("max_range").value)
        self._ser = self._open_serial()
        self._scanning = False
        self._trigger_event = threading.Event()

        self.create_subscription(Bool, "/scout/radar_trigger", self._trigger_cb, 10)
        if CUSTOM_MSG:
            self._pub = self.create_publisher(LifeSign, "/scout/life_detections", 10)
        else:
            self._pub = self.create_publisher(Float32MultiArray, "/scout/life_detections", 10)
        self._pub_debug = self.create_publisher(String, "/scout/radar_debug", 10)
        threading.Thread(target=self._scan_loop, daemon=True).start()

    def _open_serial(self):
        if serial is None:
            self.get_logger().warn("pyserial not installed; radar simulation enabled")
            return None
        try:
            return serial.Serial(
                self.get_parameter("port").value,
                int(self.get_parameter("baud").value),
                timeout=0.1,
            )
        except Exception as exc:
            self.get_logger().warn(f"Radar serial unavailable; simulation enabled: {exc}")
            return None

    def _trigger_cb(self, msg: Bool):
        if bool(msg.data) and not self._scanning:
            self._trigger_event.set()

    def _scan_loop(self):
        while rclpy.ok():
            if not self._trigger_event.wait(timeout=0.05):
                continue
            self._trigger_event.clear()
            self._scanning = True
            frames = self._collect_frames(self.DURATION)
            detections = self._process(frames) if len(frames) > 10 else []
            self._publish(detections)
            self._scanning = False

    def _collect_frames(self, duration):
        frames = []
        t0 = time.time()
        if self._ser is None:
            while time.time() - t0 < duration:
                t = time.time() - t0
                phase = (
                    math.sin(2 * math.pi * 0.25 * t) * 0.50
                    + math.sin(2 * math.pi * 1.20 * t) * 0.20
                    + float(np.random.normal(0.0, 0.05))
                )
                frames.append({"range": 1.5, "phase": phase, "snr": 25.0})
                time.sleep(1.0 / self.FPS)
            return frames

        while time.time() - t0 < duration:
            try:
                line = self._ser.readline().decode("utf-8", errors="ignore").strip()
                if line.startswith("OBJ"):
                    parts = line.split(",")
                    if len(parts) >= 4:
                        frames.append(
                            {
                                "range": float(parts[1]),
                                "phase": float(parts[2]),
                                "snr": float(parts[3]),
                            }
                        )
            except Exception:
                continue
        return frames

    def _process(self, frames):
        ranges = np.asarray([f["range"] for f in frames], dtype=np.float32)
        phases = np.asarray([f["phase"] for f in frames], dtype=np.float32)
        valid = ranges <= self._max_r
        if not np.any(valid):
            return []

        phases = phases[valid]
        median_range = float(np.median(ranges[valid]))
        freq = np.fft.rfftfreq(len(phases), d=1.0 / self.FPS)
        fft = np.abs(np.fft.rfft(phases - np.mean(phases)))
        b_mask = (freq >= self.BREATH_LO) & (freq <= self.BREATH_HI)
        h_mask = (freq >= self.HEART_LO) & (freq <= self.HEART_HI)
        if not np.any(b_mask) or not np.any(h_mask):
            return []

        b_fft = fft[b_mask]
        h_fft = fft[h_mask]
        noise = float(np.median(fft)) + 1e-6
        b_peak = float(np.max(b_fft))
        h_peak = float(np.max(h_fft))
        b_snr = 20.0 * math.log10(b_peak / noise + 1e-9)
        h_snr = 20.0 * math.log10(h_peak / noise + 1e-9)
        b_freq = float(freq[b_mask][np.argmax(b_fft)])
        h_freq = float(freq[h_mask][np.argmax(h_fft)])
        b_conf = min(1.0, max(0.0, (b_snr - 5.0) / 15.0))
        h_conf = min(1.0, max(0.0, (h_snr - 5.0) / 10.0))
        if b_snr > self.BREATH_SNR_TH and h_snr > self.HEART_SNR_TH:
            life_conf = 0.6 * b_conf + 0.4 * h_conf
        else:
            life_conf = 0.4 * max(b_conf, h_conf)

        det = {
            "range": median_range,
            "breath_freq": b_freq,
            "heart_freq": h_freq,
            "breath_snr": b_snr,
            "heart_snr": h_snr,
            "life_conf": life_conf,
        }
        debug = String()
        debug.data = (
            f"range={det['range']:.2f}m breath={b_freq:.2f}Hz(SNR{b_snr:.1f}dB) "
            f"heart={h_freq:.2f}Hz(SNR{h_snr:.1f}dB) life_conf={life_conf:.2f}"
        )
        self._pub_debug.publish(debug)
        return [det] if life_conf > 0.3 else []

    def _publish(self, detections):
        for det in detections:
            if CUSTOM_MSG:
                msg = LifeSign()
                msg.range = det["range"]
                msg.breath_freq = det["breath_freq"]
                msg.heart_freq = det["heart_freq"]
                msg.confidence = det["life_conf"]
            else:
                msg = Float32MultiArray()
                msg.data = [det["range"], det["breath_freq"], det["heart_freq"], det["life_conf"]]
            self._pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(RadarProcessor())
    rclpy.shutdown()
