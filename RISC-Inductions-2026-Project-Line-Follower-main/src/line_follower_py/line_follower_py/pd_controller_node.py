#!/usr/bin/env python3

"""Camera PID controller for triangular line following (merged version).

Subscribes:
    /line_error: Float32MultiArray [error, status]

Publishes:
    /cmd_vel: geometry_msgs/Twist

Detector statuses:
    0 = tracking
    1 = temporarily lost
    2 = corner / branch / intersection
    3 = persistently lost

Features:
    * tuned defaults (slow, tight gains) for the 120-degree triangle corners
    * optional nonlinear proportional gain (kp_nl; set 0.0 for pure linear P)
    * derivative computed per camera frame from measurement time, with a
      time-aware low-pass fliter;
    * conditional-integration anti-windup plus an integral zone
    * speed scheduling (error, error rate, commanded turn rate) and linear
      acceleration limits
    * corner recovery: momentum turn toward the side the line was last seen,
      then a bounded pivot search (enable_search:=false = stop when lost)
    * 1 Hz debug log while tracking
"""

import math
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
import statistics

class CameraPIDController(Node):
    def __init__(self):
        super().__init__("camera_pid_controller")

        def param(name, default):
            self.declare_parameter(name, default)
            return self.get_parameter(name).value

        self.kp = float(param("kp", 1.35))
        self.kp_nl = float(param("kp_nl", 0.90))
        self.ki = float(param("ki", 0.005))
        self.kd = float(param("kd", 0.055))
        self.integral_limit = max(0.0, float(param("integral_limit", 0.35)))
        self.i_zone = max(0.0, float(param("i_zone", 0.6)))

        self.v_base = max(0.0, float(param("v_base", 0.15)))
        self.v_min = max(0.0, min(self.v_base, float(param("v_min", 0.010))))
        self.max_w = max(0.0, float(param("max_w", 3.0)))
        self.max_w_rate = max(0.0, float(param("max_w_rate", 40.0)))
        self.max_v_accel = max(0.0, float(param("max_v_accel", 0.4)))
        self.max_v_decel = max(0.0, float(param("max_v_decel", 1.5)))

        self.error_slowdown = float(param("error_slowdown", 0.45))
        self.rate_slowdown = float(param("rate_slowdown", 0.15))
        self.turn_slowdown = float(param("turn_slowdown", 0.40))
        self.corner_speed_scale = max(
            0.1, min(1.0, float(param("corner_speed_scale", 0.50)))
        )

        self.sensor_timeout = max(0.05, float(param("sensor_timeout", 0.45)))
        self.derivative_tau = max(0.001, float(param("derivative_tau", 0.06)))
        self.max_error = max(0.1, float(param("max_error", 2.0)))
        self.max_derivative = max(0.1, float(param("max_derivative", 8.0)))

        self.lost_grace_sec = max(0.0, float(param("lost_grace_sec", 0.50)))
        self.recovery_w = max(0.0, float(param("recovery_w", 2.5)))
        self.enable_search = bool(param("enable_search", True))
        self.search_w = max(0.0, float(param("search_w", 2.0)))
        self.search_timeout = max(0.0, float(param("search_timeout", 4.0)))
        self.turn_latch_threshold = max(
            0.0, float(param("turn_latch_threshold", 0.4))
        )
        self.turn_lookahead_sec = max(0.0, float(param("turn_lookahead_sec", 0.15)))
        self.corner_turn_sign = self.clamp(float(param("corner_turn_sign", 1.0)), -1.0, 1.0)

        self.error = 0.0
        self.status = -1
        self.last_valid_error = 0.0
        self.last_message_time = None
        self.lost_since = None
        self.valid_streak = 0
        self.has_seen_line = False
        self.turn_sign = 0.0

        self.integral = 0.0
        self.previous_error = None
        self.previous_measurement_time = None
        self.filtered_derivative = 0.0

        self.last_control_time = time.monotonic()
        self.last_angular = 0.0
        self.last_linear = 0.0

        self.last_warning_time = 0.0
        self.last_debug_log_time = 0.0
        self.phase = "idle"

        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.error_sub = self.create_subscription(
            Float32MultiArray, "/line_error", self.error_callback, 10
        )
        self.timer = self.create_timer(0.02, self.control_loop)

        self.get_logger().info(
            "Triangle caemra PID started: "
            f"Kp={self.kp:.3f}, Kp_nl={self.kp_nl:.3f}, Ki={self.ki:.3f}, "
            f"Kd={self.kd:.3f}, v_base={self.v_base:.3f} m/s, "
            f"max_w={self.max_w:.2f} rad/s, srarch={self.enable_search}."
        )

    @staticmethod
    def clamp(value, low, high):
        return max(low, min(high, value))

    def publish_cmd(self, linear, angular):
        cmd = Twist()
        cmd.linear.x = float(linear)
        cmd.angular.z = float(angular)
        self.cmd_pub.publish(cmd)

    def stop(self):
        self.publish_cmd(0.0, 0.0)
        self.last_angular = 0.0
        self.last_linear = 0.0

    def reset_derivative(self):
        self.previous_error = None
        self.previous_measurement_time = None
        self.filtered_derivative = 0.0

    def warn_throttled(self, message):
        now = time.monotonic()
        if now - self.last_warning_time >= 2.0:
            self.get_logger().warning(message)
            self.last_warning_time = now

    def recovery_sign(self):
        if self.corner_turn_sign != 0.0:
            return self.corner_turn_sign
        return self.turn_sign
    def set_phase(self, phase):
        if phase != self.phase:
            self.get_logger().info(
                f"Phase {self.phase} -> {phase} | status={self.status} "
                f"err={self.error:+.3f} last_valid={self.last_valid_error:+.3f} "
                f"d={self.filtered_derivative:+.3f} turn_sign={self.recovery_sign():+.0f} "
                f"w={self.last_angular:+.3f} v={self.last_linear:.3f}"
            )
            self.phase = phase

    def rate_limit_angular(self, target, dt):
        max_change = self.max_w_rate * dt
        result = self.clamp(
            target, self.last_angular - max_change, self.last_angular + max_change
        )
        self.last_angular = result
        return result

    def rate_limit_linear(self, target, dt):
        result = self.clamp(
            target,
            self.last_linear - self.max_v_decel * dt,
            self.last_linear + self.max_v_accel * dt,
        )
        self.last_linear = max(0.0, result)
        return self.last_linear

    def steering(self, error, integral, derivative):
        """PID output as angular.z (positive error = line image-right
        -> negative angular.z steers right)."""
        p_term = self.kp * error + self.kp_nl * error * abs(error) / self.max_error
        return -(p_term + self.ki * integral + self.kd * derivative)

    def error_callback(self, msg):
        if len(msg.data) < 2:
            return

        error = float(msg.data[0])
        raw_status = float(msg.data[1])

        if not math.isfinite(error) or not math.isfinite(raw_status):
            return

        status = int(round(raw_status))
        now = time.monotonic()
        self.last_message_time = now

        if abs(raw_status - status) > 0.01 or status not in (0, 1, 2, 3):
            self.status = -1
            self.valid_streak = 0
            self.lost_since = None
            self.reset_derivative()
            return

        previous_status = self.status
        self.status = status
        self.error = self.clamp(error, -self.max_error, self.max_error)

        if status in (0, 2):
            if previous_status not in (0, 2):
                self.reset_derivative()
                self.integral *= 0.5

            self.last_valid_error = self.error
            self.has_seen_line = True
            self.valid_streak += 1
            self.lost_since = None

            self.update_pid(self.error, now)

            predicted = self.error + self.turn_lookahead_sec * self.filtered_derivative
            if abs(predicted) >= self.turn_latch_threshold:
                self.turn_sign = -1.0 if predicted > 0.0 else 1.0
        else:
            self.valid_streak = 0
            self.reset_derivative()
            if self.lost_since is None:
                self.lost_since = now

    def update_pid(self, error, now):
        """Run once per usable camera frame, using measurement time."""
        if (
            self.previous_error is not None
            and self.previous_measurement_time is not None
        ):
            sample_dt = now - self.previous_measurement_time

            if 0.005 <= sample_dt <= self.sensor_timeout:
                derivative = self.clamp(
                    (error - self.previous_error) / sample_dt,
                    -self.max_derivative,
                    self.max_derivative,
                )
                alpha = sample_dt / (self.derivative_tau + sample_dt)
                self.filtered_derivative += alpha * (
                    derivative - self.filtered_derivative
                )
            elif sample_dt > self.sensor_timeout:
                self.filtered_derivative = 0.0
        else:
            sample_dt = 1.0 / 30.0
            self.filtered_derivative = 0.0

        if self.previous_error is not None and error * self.previous_error < 0.0:
            self.integral *= 0.5

        integral_dt = self.clamp(sample_dt, 0.005, 0.2)

        self.previous_error = error
        self.previous_measurement_time = now

        if self.status == 0 and abs(error) <= self.i_zone:
            if abs(error) < 0.04:
                self.integral *= max(0.0, 1.0 - 2.0 * integral_dt)
            else:
                candidate = self.clamp(
                    self.integral + error * integral_dt,
                    -self.integral_limit,
                    self.integral_limit,
                )
                output = self.steering(error, candidate, self.filtered_derivative)

                if abs(output) <= self.max_w or output * error > 0.0:
                    self.integral = candidate
        else:
            self.integral *= max(0.0, 1.0 - 4.0 * integral_dt)

    def control_loop(self):
        now = time.monotonic()
        dt = self.clamp(now - self.last_control_time, 0.001, 0.1)
        self.last_control_time = now

        if (
            self.last_message_time is None
            or now - self.last_message_time > self.sensor_timeout
        ):
            self.stop()
            self.integral = 0.0
            self.reset_derivative()
            self.set_phase("idle")
            self.warn_throttled("Camera detector data missing or stale; stopped.")
            return

        if self.status == -1:
            self.stop()
            self.integral = 0.0
            self.set_phase("idle")
            self.warn_throttled("Invalid detectr status; stopped.")
            return

        if self.status in (1, 3):
            self.recover(now, dt)
            return

        if self.valid_streak < 2:
            self.publish_cmd(self.last_linear, self.last_angular)
            return

        self.set_phase("tracking")
        self.track(now, dt)

    def track(self, now, dt):
        error = self.error

        target_w = self.clamp(
            self.steering(error, self.integral, self.filtered_derivative),
            -self.max_w,
            self.max_w,
        )
        angular = self.rate_limit_angular(target_w, dt)

        e_frac = min(abs(error) / self.max_error, 1.0)
        d_frac = min(abs(self.filtered_derivative) / self.max_derivative, 1.0)
        w_frac = min(abs(angular) / self.max_w, 1.0) if self.max_w > 0.0 else 0.0

        scale = max(
            0.0,
            1.0
            - self.error_slowdown * e_frac
            - self.rate_slowdown * d_frac
            - self.turn_slowdown * w_frac,
        )
        target_v = max(self.v_min, self.v_base * scale)

        if self.status == 2:
            target_v = max(self.v_min, target_v * self.corner_speed_scale)

        linear = self.rate_limit_linear(target_v, dt)
        self.publish_cmd(linear, angular)

        if now - self.last_debug_log_time >= 1.0:
            self.get_logger().info(
                f"status={self.status} error={error:+.3f} "
                f"d={self.filtered_derivative:+.3f} i={self.integral:+.3f} "
                f"v={linear:.3f} w={angular:+.3f}"
            )
            self.last_debug_log_time = now

    def recover(self, now, dt):
        elapsed = float("inf") if self.lost_since is None else now - self.lost_since

        side = self.recovery_sign()

        if not self.has_seen_line:
            self.stop()
            self.set_phase("idle")
            return

        if now - self.last_debug_log_time >= 0.25:
            self.get_logger().info(
                f"lost: status={self.status} elapsed={elapsed:.2f}s side={side:+.0f} "
                f"last_valid={self.last_valid_error:+.3f} w={self.last_angular:+.3f}"
            )
            self.last_debug_log_time = now

        if elapsed <= self.lost_grace_sec:
            self.set_phase("momentum")

            if side != 0.0:
                target_w = side * max(abs(self.last_angular), self.recovery_w)
            else:
                target_w = -self.kp * self.last_valid_error

            target_w = self.clamp(target_w, -self.max_w, self.max_w)
            angular = self.rate_limit_angular(target_w, dt)
            linear = self.rate_limit_linear(min(self.v_min, self.v_base * 0.25), dt)
            self.integral *= max(0.0, 1.0 - 4.0 * dt)
            self.publish_cmd(linear, angular)
            return

        if (
            self.enable_search
            and side != 0.0
            and elapsed <= self.lost_grace_sec + self.search_timeout
        ):
            self.set_phase("search")
            target_w = self.clamp(side * self.search_w, -self.max_w, self.max_w)
            angular = self.rate_limit_angular(target_w, dt)
            linear = self.rate_limit_linear(0.0, dt)
            self.publish_cmd(linear, angular)
            return

        self.stop()
        self.integral = 0.0
        self.set_phase("stopped")
        self.warn_throttled("Line lost beyond recovery window; stopped.")

    def destroy_node(self):
        self.stop()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraPIDController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        time.sleep(0.05)
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()