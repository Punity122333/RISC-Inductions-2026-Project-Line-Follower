#!/usr/bin/env python3
"""Down-facing camera line detector (camera fixed to body, looks straight down).

Image top = robot forward, image left = robot left.

Publishes /line_error: [error, status, lean, corner_side, ahead]
    error        lateral error, ~[-2, 2]; + = line is image-right
    status       0 tracking, 1 temporarily lost, 2 corner, 3 persistently lost
    lean         line angle vs heading (rad), + = line tilts image-right forward
    corner_side  -1 line exits image-left, +1 exits image-right, 0 none
    ahead        1 if the line is still visible in the top (lookahead) rows
"""

import math
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge, CvBridgeError
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray


class CameraLineDetector(Node):
    def __init__(self):
        super().__init__("line_detector")

        def param(name, default):
            self.declare_parameter(name, default)
            return self.get_parameter(name).value

        self.image_topic = str(param("image_topic", "/camera/image"))
        self.debug_topic = str(param("debug_topic", "/camera/debug_image"))
        self.dark_threshold = float(param("dark_threshold", 90))
        self.min_run_width = int(param("min_run_width", 3))
        self.max_run_fraction = float(param("max_run_fraction", 0.6))
        self.corner_run_fraction = float(param("corner_run_fraction", 0.25))
        self.lost_limit = int(param("lost_frames_before_status3", 40))
        self.alpha = min(1.0, max(0.0, float(param("error_filter_alpha", 0.7))))
        self.error_sign = 1.0 if float(param("error_sign", 1.0)) >= 0 else -1.0

        self.row_fractions = [0.15, 0.30, 0.45, 0.60, 0.75, 0.90]
        self.row_weights = [0.05, 0.10, 0.15, 0.20, 0.25, 0.25]
        self.ahead_rows = 2  # top N rows define "line still ahead"

        self.bridge = CvBridge()
        self.kernel = np.ones((3, 3), np.uint8)
        self.last_valid_error = 0.0
        self.last_side = 0.0
        self.filtered_error = 0.0
        self.filtered_lean = 0.0
        self.lost_frames = 0
        self.last_warn = 0.0

        self.error_pub = self.create_publisher(Float32MultiArray, "/line_error", 10)
        self.debug_pub = self.create_publisher(Image, self.debug_topic, 2)
        self.create_subscription(
            Image, self.image_topic, self.image_callback, qos_profile_sensor_data
        )
        self.get_logger().info("Downfacing line detector started.")

    @staticmethod
    def clamp(v, lo, hi):
        return max(lo, min(hi, v))

    def publish_error(self, error, status, lean=0.0, side=0.0, ahead=0.0):
        msg = Float32MultiArray()
        msg.data = [float(error), float(status), float(lean), float(side), float(ahead)]
        self.error_pub.publish(msg)

    def get_runs(self, row, width):
        active = np.flatnonzero(row > 0)
        if active.size == 0:
            return []
        groups = np.split(active, np.where(np.diff(active) > 1)[0] + 1)
        runs = []
        for g in groups:
            s, e = int(g[0]), int(g[-1])
            w = e - s + 1
            touches_l = s <= 1
            touches_r = e >= width - 2
            if w < self.min_run_width:
                continue
            if w > self.max_run_fraction * width and not (touches_l or touches_r):
                continue
            runs.append((s, e, w, touches_l, touches_r))
        return runs

    def pick(self, runs, anchor, width):
        """Return (target_x, edge_side). Edge-touching wide runs are corner arms:
        aim at the image edge they run to."""
        if not runs:
            return None, 0.0
        s, e, w, tl, tr = min(runs, key=lambda r: abs(0.5 * (r[0] + r[1]) - anchor))
        if (tl or tr) and w >= self.corner_run_fraction * width:
            if tl and not tr:
                return 0.0, -1.0
            if tr and not tl:
                return width - 1.0, 1.0
        return 0.5 * (s + e), 0.0

    def lost(self, frame, mask, rows_y):
        self.lost_frames += 1
        status = 3.0 if self.lost_frames > self.lost_limit else 1.0
        if abs(self.last_valid_error) >= 0.3:
            side = 1.0 if self.last_valid_error > 0 else -1.0
        elif self.last_side != 0.0:
            side = self.last_side
        else:
            side = 0.0
        raw = 2.0 * self.error_sign * side
        self.filtered_error = raw if status == 3.0 else (
            self.alpha * raw + (1 - self.alpha) * self.filtered_error
        )
        self.publish_error(self.filtered_error, status, 0.0, side, 0.0)
        self.publish_debug(frame, mask, rows_y, [], self.filtered_error, status)
        now = time.monotonic()
        if now - self.last_warn >= 2.0:
            self.get_logger().warning(f"Line not found; status={int(status)}")
            self.last_warn = now

    def publish_debug(self, frame, mask, rows_y, sel, error, status):
        h, w = frame.shape[:2]
        dbg = cv2.addWeighted(frame, 0.75, cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR), 0.25, 0)
        cv2.line(dbg, (w // 2, 0), (w // 2, h - 1), (255, 100, 0), 1)
        for i, y in enumerate(rows_y):
            cv2.line(dbg, (0, y), (w - 1, y), (90, 180, 90), 1)
            if i < len(sel) and sel[i] is not None:
                cv2.circle(dbg, (int(round(sel[i])), y), 4, (0, 0, 255), -1)
        cv2.putText(dbg, f"e={error:+.2f} s={int(status)}", (6, 18),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                    (0, 255, 0) if status in (0.0, 2.0) else (0, 0, 255), 1, cv2.LINE_AA)
        try:
            self.debug_pub.publish(self.bridge.cv2_to_imgmsg(dbg, encoding="bgr8"))
        except CvBridgeError:
            pass

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except CvBridgeError as exc:
            self.get_logger().error(f"Camera conversion failed: {exc}")
            return
        h, w = frame.shape[:2]
        if h < 20 or w < 20:
            return

        blurred = cv2.GaussianBlur(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY), (3, 3), 0)
        otsu, _ = cv2.threshold(blurred, 0, 255, cv2.THRESH_BINARY_INV | cv2.THRESH_OTSU)
        thr = min(float(otsu), self.dark_threshold, 0.5 * float(np.median(blurred)))
        _, mask = cv2.threshold(blurred, thr, 255, cv2.THRESH_BINARY_INV)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.kernel)

        rows_y = [int(f * (h - 1)) for f in self.row_fractions]
        n = len(rows_y)
        sel = [None] * n
        sides = [0.0] * n

        anchor = (w - 1) / 2.0
        for i in range(n - 1, -1, -1):
            x, side = self.pick(self.get_runs(mask[rows_y[i], :], w), anchor, w)
            if x is not None:
                sel[i], sides[i] = x, side
                anchor = x

        valid = [(c, self.row_weights[i]) for i, c in enumerate(sel) if c is not None]
        if len(valid) < 2:
            self.lost(frame, mask, rows_y)
            return

        center = sum(c * wt for c, wt in valid) / sum(wt for _, wt in valid)
        raw = self.clamp(self.error_sign * 4.0 * (center / max(1.0, w - 1.0) - 0.5), -2.0, 2.0)

        ahead = any(sel[i] is not None for i in range(self.ahead_rows))
        corner_side = 0.0
        for s in sides:
            if s != 0.0:
                corner_side = s
        is_corner = corner_side != 0.0 and not ahead
        if is_corner:
            raw = 2.0 * self.error_sign * corner_side
            self.last_side = corner_side

        self.lost_frames = 0
        self.last_valid_error = raw
        self.filtered_error = self.alpha * raw + (1 - self.alpha) * self.filtered_error

        lean = 0.0
        idx = [i for i, c in enumerate(sel) if c is not None]
        far, near = idx[0], idx[-1]
        if rows_y[near] - rows_y[far] >= 0.2 * h and not is_corner:
            lean = self.clamp(
                math.atan2(sel[far] - sel[near], rows_y[near] - rows_y[far]), -1.05, 1.05
            )
        self.filtered_lean = self.alpha * lean + (1 - self.alpha) * self.filtered_lean

        status = 2.0 if is_corner else 0.0
        self.publish_error(self.filtered_error, status, self.filtered_lean,
                           corner_side if is_corner else 0.0, 1.0 if ahead else 0.0)
        self.publish_debug(frame, mask, rows_y, sel, self.filtered_error, status)


def main(args=None):
    rclpy.init(args=args)
    node = CameraLineDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()