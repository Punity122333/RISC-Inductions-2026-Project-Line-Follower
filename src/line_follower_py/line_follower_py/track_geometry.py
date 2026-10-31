# FILE 1: src/line_follower_py/line_follower_py/track_geometry.py

"""Shared equilateral-triangle track geometry in meters.

The robot starts at (0, 0), facing along +X.
The visible Gazebo track and all geometry consumers use the same
three vertices and closed centerline.
"""

import math
import numpy as np


# Side length, meters.
TRACK_SIZE_M = 3.0
TRACK_WIDTH_M = 0.022
LINE_HALF_WIDTH_M = TRACK_WIDTH_M / 2.0

# Retained for compatibility with other modules.
SENSOR_FRONT_X = 0.054
SENSOR_YS = np.array(
    [0.064, 0.032, 0.0, -0.032, -0.064],
    dtype=float,
)

SPAWN_Z = 0.02
SAMPLES_PER_SIDE = 120


def triangle_vertices():
    """Return the three vertices of an equilateral triangle.

    The bottom edge runs from (0, 0) to (3.0, 0).
    The third vertex is above the midpoint of that edge.
    """
    side = TRACK_SIZE_M
    height = math.sqrt(3.0) * side / 2.0

    return np.array(
        [
            [0.0, 0.0],
            [side, 0.0],
            [side / 2.0, height],
        ],
        dtype=float,
    )


def catmull_rom_spline(pts, num_samples=20):
    """Compatibility helper for other project modules."""
    pts = np.asarray(pts, dtype=float)
    n = len(pts)
    spline = []

    for i in range(n):
        p0 = pts[(i - 1 + n) % n]
        p1 = pts[i]
        p2 = pts[(i + 1) % n]
        p3 = pts[(i + 2) % n]

        for j in range(num_samples):
            t = j / float(num_samples)

            point = 0.5 * (
                2 * p1
                + (-p0 + p2) * t
                + (2 * p0 - 5 * p1 + 4 * p2 - p3) * t**2
                + (-p0 + 3 * p1 - 3 * p2 + p3) * t**3
            )

            spline.append(point)

    return np.asarray(spline, dtype=float)


def world_track_pts():
    """Return a closed, densely sampled triangular centerline."""
    vertices = triangle_vertices()
    points = []

    for i in range(3):
        start = vertices[i]
        end = vertices[(i + 1) % 3]

        for t in np.linspace(
            0.0,
            1.0,
            SAMPLES_PER_SIDE,
            endpoint=False,
        ):
            points.append(start + t * (end - start))

    # Close the triangle explicitly.
    points.append(vertices[0])

    return np.asarray(points, dtype=float)


def start_pose():
    """Robot spawn pose: first vertex, heading along +X."""
    return 0.0, 0.0, 0.0


def odom_track_pts():
    """Return the same triangle in the normalized track frame."""
    return world_track_pts().copy()


def distance_to_track(points, query):
    """Return the shortest distance to any segment of the closed track."""
    points = np.asarray(points, dtype=float)
    query = np.asarray(query, dtype=float)

    starts = points[:-1]
    ends = points[1:]

    segments = ends - starts
    lengths_sq = np.einsum("ij,ij->i", segments, segments)
    offsets = query - starts

    fractions = np.divide(
        np.einsum("ij,ij->i", offsets, segments),
        lengths_sq,
        out=np.zeros_like(lengths_sq),
        where=lengths_sq > 1e-12,
    )

    fractions = np.clip(fractions, 0.0, 1.0)
    closest = starts + fractions[:, None] * segments

    return float(
        np.min(np.linalg.norm(closest - query, axis=1))
    )