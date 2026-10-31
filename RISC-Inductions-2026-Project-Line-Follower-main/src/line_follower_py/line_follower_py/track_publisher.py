#!/usr/bin/env python3
"""Publish the Monza centerline as an RViz marker in the odom frame.

Coordinates come from track_geometry (metric, anchored at the spawn pose),
so the marker lines up with the painted line in the Gazebo world.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker

from line_follower_py.track_geometry import TRACK_WIDTH_M, odom_track_pts


class TrackPublisherNode(Node):
    def __init__(self):
        super().__init__('track_publisher')
        self.pub = self.create_publisher(Marker, '/line_track', 10)
        self.track_pts = odom_track_pts()
        self.get_logger().info('Track Publisher running.')
        self.timer = self.create_timer(1.0, self.publish_track)

    def publish_track(self):
        marker = Marker()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = "odom"
        marker.ns = "line_track"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = TRACK_WIDTH_M
        marker.scale.y = 0.0
        marker.scale.z = 0.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 1.0
        marker.color.a = 1.0
        marker.pose.orientation.w = 1.0
        marker.pose.position.z = 0.003

        pts = list(self.track_pts) + [self.track_pts[0]]
        marker.points = [Point(x=float(pt[0]), y=float(pt[1]), z=0.0) for pt in pts]
        self.pub.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    node = TrackPublisherNode()
    print("Testing")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
