#!/usr/bin/env python3

"""Analog five-sensor simulator aligned to the hexagonal track.

Each sensor publishes a floating-point strength in [0, 1] rather than a
binary on/off value. The response is strongest near the line center and
falls smoothly with distance.
"""

import math

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from tf2_ros import TransformBroadcaster

from line_follower_py.track_geometry import (
    LINE_HALF_WIDTH_M,
    SENSOR_FRONT_X,
    SENSOR_YS,
    distance_to_track,
    odom_track_pts,
)


def wrap_angle(angle):
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def yaw_from_quat(q):
    sin_yaw = 2.0 * (q.w * q.z + q.x * q.y)
    cos_yaw = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(sin_yaw, cos_yaw)


class GazeboLineSimNode(Node):
    def __init__(self):
        super().__init__("gazebo_line_sim")

        self.declare_parameter(
            "odom_topic",
            "/model/line_follower_robot/odometry",
        )
        self.declare_parameter("track_spawn_x", 0.0)
        self.declare_parameter("track_spawn_y", 0.0)
        self.declare_parameter("track_spawn_yaw", 0.0)

        self.declare_parameter("sensor_sigma_m", 0.012)
        self.declare_parameter("sensor_max_range_m", 0.036)
        self.declare_parameter("sensor_min_output", 0.015)

        self.odom_topic = str(
            self.get_parameter("odom_topic").value
        )
        self.spawn_x = float(
            self.get_parameter("track_spawn_x").value
        )
        self.spawn_y = float(
            self.get_parameter("track_spawn_y").value
        )
        self.spawn_yaw = float(
            self.get_parameter("track_spawn_yaw").value
        )

        self.sensor_sigma = max(
            0.001,
            float(self.get_parameter("sensor_sigma_m").value),
        )
        self.sensor_max_range = max(
            0.001,
            float(self.get_parameter("sensor_max_range_m").value),
        )
        self.sensor_min_output = min(
            1.0,
            max(
                0.0,
                float(self.get_parameter("sensor_min_output").value),
            ),
        )

        if len(SENSOR_YS) != 5:
            raise ValueError(
                f"Expected exactly five sensor offsets, got {len(SENSOR_YS)}"
            )

        self.track_pts = odom_track_pts()

        self.origin_x = None
        self.origin_y = None
        self.origin_yaw = None
        self.previous_raw_yaw = None
        self.relative_yaw = 0.0

        self.x = self.spawn_x
        self.y = self.spawn_y
        self.heading = self.spawn_yaw
        self.v = 0.0
        self.w = 0.0
        self.has_pose = False

        self.sensor_pub = self.create_publisher(
            Float32MultiArray,
            "/sensor_raw",
            10,
        )
        self.tf_broadcaster = TransformBroadcaster(self)

        self.odom_sub = self.create_subscription(
            Odometry,
            self.odom_topic,
            self.odom_callback,
            10,
        )

        self.create_timer(0.05, self.update)
        self.create_timer(5.0, self.log_status)

        self.get_logger().info(
            f"odom={self.odom_topic}, "
            f"sigma={self.sensor_sigma * 1000:.1f} mm, "
            f"line half-width={LINE_HALF_WIDTH_M * 1000:.1f} mm, "
            "output=Float32MultiArray."
        )

    def odom_callback(self, msg: Odometry):
        pos = msg.pose.pose.position
        raw_yaw = yaw_from_quat(msg.pose.pose.orientation)

        if self.origin_x is None:
            self.origin_x = float(pos.x)
            self.origin_y = float(pos.y)
            self.origin_yaw = raw_yaw
            self.previous_raw_yaw = raw_yaw
            self.relative_yaw = 0.0

            self.get_logger().info(
                f"Odometry origin: ({pos.x:.4f}, {pos.y:.4f}), "
                f"yaw={raw_yaw:.4f}. "
                f"Track start=({self.spawn_x:.3f}, "
                f"{self.spawn_y:.3f}, {self.spawn_yaw:.3f})."
            )
        else:
            yaw_delta = wrap_angle(
                raw_yaw - self.previous_raw_yaw
            )
            self.relative_yaw += yaw_delta
            self.previous_raw_yaw = raw_yaw

        dx = float(pos.x) - self.origin_x
        dy = float(pos.y) - self.origin_y

        c0 = math.cos(self.origin_yaw)
        s0 = math.sin(self.origin_yaw)
        local_x = c0 * dx + s0 * dy
        local_y = -s0 * dx + c0 * dy

        cs = math.cos(self.spawn_yaw)
        ss = math.sin(self.spawn_yaw)

        self.x = self.spawn_x + cs * local_x - ss * local_y
        self.y = self.spawn_y + ss * local_x + cs * local_y
        self.heading = self.spawn_yaw + self.relative_yaw

        self.v = float(msg.twist.twist.linear.x)
        self.w = float(msg.twist.twist.angular.z)
        self.has_pose = True

    def analog_response(self, distance):
        """Gaussian sensor response, returned as a float from 0 to 1."""
        if distance >= self.sensor_max_range:
            return 0.0

        normalized_distance = distance / self.sensor_sigma
        strength = math.exp(
            -0.5 * normalized_distance * normalized_distance
        )

        if strength < self.sensor_min_output:
            return 0.0

        return float(min(1.0, max(0.0, strength)))

    def sensor_readings(self):
        """Return five continuous readings, ordered left to right."""
        c = math.cos(self.heading)
        s = math.sin(self.heading)

        rotation = np.array([
            [c, -s],
            [s, c],
        ])

        position = np.array([self.x, self.y])
        readings = []

        for y_offset in SENSOR_YS:
            sensor_local = np.array([
                SENSOR_FRONT_X,
                float(y_offset),
            ])
            sensor_position = position + rotation @ sensor_local

            distance = distance_to_track(
                self.track_pts,
                sensor_position,
            )

            readings.append(self.analog_response(distance))

        return readings

    def update(self):
        if not self.has_pose:
            return

        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = "odom"
        transform.child_frame_id = "base_link"

        transform.transform.translation.x = float(self.x)
        transform.transform.translation.y = float(self.y)
        transform.transform.translation.z = 0.0

        transform.transform.rotation.x = 0.0
        transform.transform.rotation.y = 0.0
        transform.transform.rotation.z = math.sin(
            self.heading / 2.0
        )
        transform.transform.rotation.w = math.cos(
            self.heading / 2.0
        )
        self.tf_broadcaster.sendTransform(transform)

        readings = self.sensor_readings()

        msg = Float32MultiArray()
        msg.data = [float(v) for v in readings]
        self.sensor_pub.publish(msg)

    def log_status(self):
        if not self.has_pose:
            self.get_logger().warning(
                f"Waiting for odometry on {self.odom_topic}."
            )
            return

        center_distance = distance_to_track(
            self.track_pts,
            np.array([self.x, self.y]),
        )


def main(args=None):
    rclpy.init(args=args)
    node = GazeboLineSimNode()

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