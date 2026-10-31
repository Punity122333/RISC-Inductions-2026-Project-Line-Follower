#!/usr/bin/env python3

import math
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray


class PIDController(Node):
    def __init__(self):
        super().__init__("pid_controller")

        # Start conservatively. Enable integral gain only after PD is stable.
        self.declare_parameter("kp", 0.8)
        self.declare_parameter("ki", 0.0)
        self.declare_parameter("kd", 0.02)
        self.declare_parameter("v_base", 0.12)
        self.declare_parameter("v_min", 0.04)
        self.declare_parameter("max_w", 1.5)

        self.kp = float(self.get_parameter("kp").value)
        self.ki = float(self.get_parameter("ki").value)
        self.kd = float(self.get_parameter("kd").value)
        self.v_base = float(self.get_parameter("v_base").value)
        self.v_min = float(self.get_parameter("v_min").value)
        self.max_w = float(self.get_parameter("max_w").value)

        self.error = 0.0
        self.status = 1.0
        self.last_valid_error = 0.0
        self.integral = 0.0
        self.prev_error = 0.0
        self.filtered_derivative = 0.0
        self.last_time = None
        self.has_error = False

        self.create_subscription(
            Float32MultiArray, "/line_error", self.error_cb, 10
        )
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.timer = self.create_timer(0.05, self.control_loop)

        self.get_logger().info(
            f"PD controller: kp={self.kp}, ki={self.ki}, kd={self.kd}, "
            f"v_base={self.v_base} m/s"
        )

    def error_cb(self, msg):
        if len(msg.data) < 2:
            return

        error, status = float(msg.data[0]), float(msg.data[1])
        if not math.isfinite(error) or not math.isfinite(status):
            return

        self.error = error
        self.status = status
        self.has_error = True

        # Statuses 1 and 3 indicate that the front line has been lost.
        if status not in (1.0, 3.0, 4.0):
            self.last_valid_error = error

    def publish_cmd(self, linear, angular):
        msg = Twist()
        msg.linear.x = float(linear)
        msg.angular.z = float(angular)
        self.cmd_pub.publish(msg)

    def control_loop(self):
        if not self.has_error:
            self.publish_cmd(0.0, 0.0)
            return

        now = time.monotonic()
        dt = 0.05 if self.last_time is None else max(
            0.01, min(now - self.last_time, 0.10)
        )
        self.last_time = now

        # Fail safe during initial line-loss debugging. No blind spinning.
        if self.status in (1.0, 3.0):
            self.integral = 0.0
            self.filtered_derivative = 0.0
            self.prev_error = self.error
            self.publish_cmd(0.0, 0.0)
            return

        if self.status == 4.0:
            # Rear sensor sees the line. Creep forward to try to reacquire it.
            angular = max(
                -0.6, min(0.6, -self.kp * self.last_valid_error)
            )
            self.publish_cmd(0.04, angular)
            self.integral = 0.0
            self.prev_error = self.error
            return

        err = self.error
        derivative = (err - self.prev_error) / dt
        derivative = max(-10.0, min(10.0, derivative))

        # Smooth derivative to reduce sensor-noise-induced steering spikes.
        self.filtered_derivative = (
            0.7 * self.filtered_derivative + 0.3 * derivative
        )

        # Anti-windup, even though ki starts at zero.
        proposed_integral = max(
            -1.0, min(1.0, self.integral + err * dt)
        )

        raw_w = -(
            self.kp * err
            + self.ki * proposed_integral
            + self.kd * self.filtered_derivative
        )

        if abs(raw_w) <= self.max_w or raw_w * err <= 0.0:
            self.integral = proposed_integral

        angular = max(-self.max_w, min(self.max_w, raw_w))

        # Slow down as the line moves farther from the centre sensor.
        linear = max(
            self.v_min,
            self.v_base * (1.0 - 0.5 * min(abs(err), 2.5) / 2.5),
        )

        self.prev_error = err
        self.publish_cmd(linear, angular)

    def stop(self):
        self.publish_cmd(0.0, 0.0)


def main(args=None):
    rclpy.init(args=args)
    node = PIDController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        time.sleep(0.05)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
