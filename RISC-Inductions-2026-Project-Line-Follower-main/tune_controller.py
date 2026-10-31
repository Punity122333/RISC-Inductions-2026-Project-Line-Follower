#!/usr/bin/env python3
"""Auto-tuner for the camera line follower, built around your SDF tracks.

Runs YOUR pd_controller_node.py and line_detector.py (ROS stubbed out, no
Gazebo) against a kinematic sim that renders the camera view of the real SDF
track geometry (real line width from each SDF, annuli for circles, overshooting rectangle corners).

MAIN tracks (loaded from --sdf-dir, default src/line_follower_py/worlds):
    triangle square square_2_5 hexagon hexagon_1_0 hexagon_0_5
    circle_0_5 circle_1_0 circle_2_5 line loop0 loop1 s_loop_open
GENERAL tracks (built in, for generality checks): wave notch oval
Any other SDF in --sdf-dir can be named by its file stem.

Camera: read from --urdf when given (camera_joint origin + sensor pitch);
otherwise --camera-x / --camera-z / --camera-pitch (defaults = your robot.urdf:
lens 0.115 m ahead of base_link, 0.04 m high, looking straight down, 100 deg
HFOV, 640x480, 30 Hz). Front axle (pivot) is at x=0.045.

Training uses features (sharpest corners / curves; evenly spaced stretches on
gentle tracks) of every MAIN track. The held-out set uses OTHER features and
variants of the MAIN tracks plus the GENERAL tracks. At the end you get a
per-track table for the baseline and the winner.

Search: (1+lambda) evolution strategy. Tuned: gains, v_base, speed limits,
slowdowns, recovery speeds, detector error_filter_alpha and max_run_fraction.
Score (lower better) per run:
    finish: time + 100*max_cross_track_m + 2*steering_reversals
    fail:   400 - 200*fraction_of_run_completed

Usage:
    python3 tune_controller.py \
        --controller src/line_follower_py/line_follower_py/pd_controller_node.py \
        --detector   src/line_follower_py/line_follower_py/line_detector.py \
        --hours 8 --out tune_out

Options: --tracks "triangle,circle_1_0"   --hold-tracks main,wave
         --turn-sign auto|left|right|guess   --v-base-min/--v-base-max
         --mirror   --resume   --quick

Turn direction (auto, decided PER TRACK, because the real robot is configured
for the track it is on): all turns left -> corner_turn_sign=+1; all right ->
-1; mixed (F1 tracks, s_loop_open, loop0/1, wave...) -> 0 (guess from the
last-seen side). Use --turn-sign left|right|guess to force one value for all.

NOTE: the SDF worlds have cast_shadows=true; the sim has no shadows. A robot
shadow can look like a line to the camera, so set cast_shadows to false in
the SDF lights (and the ground plane) when testing in Gazebo.

Outputs in --out: results.csv, best.json, best_params.yaml, best_cli.txt,
tuned_pd_controller_node.py, tune.log.
"""

import os

for _v in ("OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS", "MKL_NUM_THREADS"):
    os.environ.setdefault(_v, "1")

import argparse
import csv
import importlib.util
import json
import math
import multiprocessing as mp
import re
import sys
import time as _real_time
import types
import xml.etree.ElementTree as ET

import cv2
import numpy as np

cv2.setNumThreads(1)

class _NS(types.SimpleNamespace):
    pass


class _Logger:
    def info(self, m): pass
    def warning(self, m): pass
    def error(self, m): pass


class _Pub:
    def __init__(self): self.last = None
    def publish(self, m): self.last = m


class _Node:
    overrides = {}

    def __init__(self, name): self.params = {}
    def declare_parameter(self, n, v): self.params[n] = _Node.overrides.get(n, v)
    def get_parameter(self, n): return _NS(value=self.params[n])
    def create_publisher(self, *a, **k): return _Pub()
    def create_subscription(self, *a, **k): return None
    def create_timer(self, *a, **k): return None
    def get_logger(self): return _Logger()
    def destroy_node(self): pass


class _CvBridgeError(Exception):
    pass


class _CvBridge:
    def imgmsg_to_cv2(self, msg, desired_encoding=None): return msg
    def cv2_to_imgmsg(self, img, encoding=None): return img


class _Twist:
    def __init__(self):
        self.linear = _NS(x=0.0)
        self.angular = _NS(z=0.0)


class _F32:
    def __init__(self): self.data = []


def _mod(name, **kw):
    m = types.ModuleType(name)
    m.__dict__.update(kw)
    sys.modules[name] = m
    return m


_mod("rclpy", ok=lambda: True, init=lambda *a, **k: None,
     shutdown=lambda: None, spin=lambda n: None)
_mod("rclpy.node", Node=_Node)
_mod("rclpy.qos", qos_profile_sensor_data=None)
_mod("geometry_msgs")
_mod("geometry_msgs.msg", Twist=_Twist)
_mod("std_msgs")
_mod("std_msgs.msg", Float32MultiArray=_F32)
_mod("sensor_msgs")
_mod("sensor_msgs.msg", Image=object)
_mod("cv_bridge", CvBridge=_CvBridge, CvBridgeError=_CvBridgeError)


class _Clock:
    t = 0.0


_fake_time = types.ModuleType("time")
_fake_time.monotonic = lambda: _Clock.t
_fake_time.sleep = lambda x: None


def load_module(path, name):
    spec = importlib.util.spec_from_file_location(name, path)
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    m.time = _fake_time
    return m


def wrap_arr(a):
    return (a + np.pi) % (2 * np.pi) - np.pi

MAIN_TRACKS = ["triangle", "square", "square_2_5", "hexagon", "hexagon_1_0",
               "hexagon_0_5", "circle_0_5", "circle_1_0", "circle_2_5", "line",
               "loop0", "loop1", "s_loop_open"]
GENERAL_TRACKS = ["wave", "notch", "oval"]
F1_TRACKS = ["monza", "redbull", "silverstone"]   
SDF_DIR = "src/line_follower_py/worlds"
LINE_W = 0.015


def densify(pts, step=0.01, closed=True):
    pts = np.asarray(pts, float)
    out = []
    n = len(pts)
    last = n if closed else n - 1
    for i in range(last):
        a, b = pts[i], pts[(i + 1) % n]
        k = max(1, int(round(np.linalg.norm(b - a) / step)))
        for j in range(k):
            out.append(a + (b - a) * j / k)
    if not closed:
        out.append(pts[-1])
    return np.array(out)


def _pose(text):
    v = [float(t) for t in (text or "").split()]
    v += [0.0] * (6 - len(v))
    return v[0], v[1], v[5]


def _compose(parent, child):
    px, py, pyaw = parent
    cx, cy, cyaw = child
    c, s = math.cos(pyaw), math.sin(pyaw)
    return px + c * cx - s * cy, py + s * cx + c * cy, pyaw + cyaw


def _floor_from_ambient(g):
    return int(round(205 + (g - 0.8) * 150))


def parse_sdf(path):
    """Return dict(prims, center, closed, width, floor).

    Handles one-visual-per-link tracks (triangle, hexagon, ...), annuli
    (circles) and F1-style tracks: one link holding hundreds of box segments
    joined by small black cylinders (the cylinder centres are the vertices).
    """
    world = ET.parse(path).getroot().find("world")
    floor = 205
    lines, disks = [], []
    for m in world.findall("model"):
        if m.get("name") == "ground_plane":
            amb = m.find(".//visual/material/ambient")
            if amb is not None:
                floor = _floor_from_ambient(float(amb.text.split()[0]))
            continue
        mp_ = _pose(m.findtext("pose"))
        for l in m.findall("link"):
            lp = _compose(mp_, _pose(l.findtext("pose")))
            for vis in l.findall("visual"):
                geo = vis.find("geometry")
                if geo is None or len(list(geo)) == 0:
                    continue
                g = list(geo)[0]
                amb = vis.findtext("material/ambient") or "0 0 0 1"
                black = float(amb.split()[0]) < 0.3
                x, y, yaw = _compose(lp, _pose(vis.findtext("pose")))
                if g.tag in ("plane", "box"):
                    sx, sy = [float(t) for t in g.find("size").text.split()[:2]]
                    if black:
                        lines.append(dict(cx=x, cy=y, yaw=yaw, sx=sx, sy=sy))
                elif g.tag == "cylinder":
                    disks.append(dict(cx=x, cy=y, r=float(g.findtext("radius")),
                                      black=black))
    prims, closed = [], True
    black_disks = [d for d in disks if d["black"]]
    if lines and len(black_disks) >= 3 and len(lines) >= 3:
        verts = np.array([[d["cx"], d["cy"]] for d in black_disks])
        closed = len(lines) >= len(verts)          # one box per vertex -> loop
        for ln in lines:
            prims.append(("rect", ln["cx"], ln["cy"], ln["yaw"], ln["sx"], ln["sy"]))
        for d in black_disks:
            prims.append(("ann", d["cx"], d["cy"], 0.0, d["r"]))
        center = densify(verts, 0.01, closed)
        width = lines[0]["sy"]
    elif lines:
        for ln in lines:
            prims.append(("rect", ln["cx"], ln["cy"], ln["yaw"], ln["sx"], ln["sy"]))
        center, closed = _line_centerline(lines)
        width = lines[0]["sy"]
    elif disks:
        outer = max(black_disks, key=lambda d: d["r"])
        inner = [d for d in disks if not d["black"]]
        r_in = inner[0]["r"] if inner else 0.0
        prims.append(("ann", outer["cx"], outer["cy"], r_in, outer["r"]))
        rc = 0.5 * (outer["r"] + r_in)
        th = np.linspace(-math.pi / 2, 1.5 * math.pi, 720, endpoint=False)
        center = densify(np.c_[outer["cx"] + rc * np.cos(th),
                               outer["cy"] + rc * np.sin(th)], 0.01, True)
        width = outer["r"] - r_in
    else:
        raise ValueError(f"no black line geometry in {path}")
    return dict(prims=prims, center=center, closed=closed, width=width, floor=floor)


def _intersect(a, b, tol=0.5):
    ua = np.array([math.cos(a["yaw"]), math.sin(a["yaw"])])
    ub = np.array([math.cos(b["yaw"]), math.sin(b["yaw"])])
    pa = np.array([a["cx"], a["cy"]])
    pb = np.array([b["cx"], b["cy"]])
    cr = ua[0] * ub[1] - ua[1] * ub[0]
    if abs(cr) < 1e-6:
        return None
    dp = pb - pa
    t = (dp[0] * ub[1] - dp[1] * ub[0]) / cr
    u = (dp[0] * ua[1] - dp[1] * ua[0]) / cr
    if abs(t) > a["sx"] / 2 + tol or abs(u) > b["sx"] / 2 + tol:
        return None
    return pa + t * ua


def _line_centerline(lines):
    n = len(lines)
    inter = [_intersect(lines[i], lines[i + 1]) for i in range(n - 1)]
    wrap = _intersect(lines[-1], lines[0]) if n >= 3 else None
    if n >= 3 and wrap is not None and all(p is not None for p in inter):
        verts = [wrap] + inter
        return densify(verts, 0.01, True), True
    if any(p is None for p in inter):
        raise ValueError("cant chain line segments into a path")

    def ends(ln):
        u = np.array([math.cos(ln["yaw"]), math.sin(ln["yaw"])])
        c = np.array([ln["cx"], ln["cy"]])
        return c - u * ln["sx"] / 2, c + u * ln["sx"] / 2

    a0, a1 = ends(lines[0])
    if n == 1:
        pts = [a0, a1]
    else:
        start = a0 if np.linalg.norm(a0 - inter[0]) > np.linalg.norm(a1 - inter[0]) else a1
        b0, b1 = ends(lines[-1])
        last = inter[-1]
        end = b0 if np.linalg.norm(b0 - last) > np.linalg.norm(b1 - last) else b1
        pts = [start] + inter + [end]
    return densify(pts, 0.01, False), False


def _polygon_prims(verts, width):
    prims = []
    for i in range(len(verts)):
        a, b = np.array(verts[i], float), np.array(verts[(i + 1) % len(verts)], float)
        m, d = (a + b) / 2, b - a
        prims.append(("rect", m[0], m[1], math.atan2(d[1], d[0]),
                      float(np.linalg.norm(d)) + width, width))
    return prims


def _smooth_prims(center, width, stride=3):
    prims = []
    n = len(center)
    for i in range(0, n, stride):
        a, b = center[i], center[(i + stride) % n]
        m, d = (a + b) / 2, b - a
        prims.append(("rect", m[0], m[1], math.atan2(d[1], d[0]),
                      float(np.linalg.norm(d)) + 0.02, width))
    return prims


def general_track(name):
    if name == "notch":
        v = [(0, 0), (2, 0), (2, 1), (1, 1), (1, 2), (0, 2)]
        return dict(prims=_polygon_prims(v, LINE_W), center=densify(v, 0.01, True),
                    closed=True, width=LINE_W, floor=205)
    th = np.linspace(0, 2 * math.pi, 900, endpoint=False)
    if name == "oval":
        pts = np.c_[1.5 * np.cos(th), 0.7 * np.sin(th)]
    elif name == "wave":
        r = 1.0 + 0.25 * np.sin(3 * th)
        pts = np.c_[r * np.cos(th), r * np.sin(th)]
    else:
        raise ValueError(f"unknown track '{name}' (no SDF found in {SDF_DIR})")
    c = densify(pts, 0.01, True)
    return dict(prims=_smooth_prims(c, LINE_W), center=c, closed=True,
                width=LINE_W, floor=205)


def _find_sdf(name):
    for d in (SDF_DIR, ".", "worlds"):
        p = os.path.join(d, name + ".sdf")
        if os.path.exists(p):
            return p
    return None


class Track:
    def __init__(self, spec, mirror=False):
        pts = spec["center"].copy()
        prims = list(spec["prims"])
        if mirror:
            pts[:, 1] *= -1
            prims = [("rect", p[1], -p[2], -p[3], p[4], p[5]) if p[0] == "rect"
                     else ("ann", p[1], -p[2], p[3], p[4]) for p in prims]
        self.closed = spec["closed"]
        self.width = spec["width"]
        self.floor = spec["floor"]
        self.prims = prims
        self.p = pts
        self.n = len(pts)
        self.q = np.roll(pts, -1, axis=0)
        if not self.closed:
            self.q[-1] = self.p[-1]
        self.d = self.q - self.p
        self.len = np.hypot(self.d[:, 0], self.d[:, 1])
        self.len2 = np.maximum(self.len ** 2, 1e-12)
        self.s = np.concatenate([[0.0], np.cumsum(self.len)[:-1]])
        self.total = float(self.len.sum())
        self.h = np.arctan2(self.d[:, 1], self.d[:, 0])
        if not self.closed:
            self.h[-1] = self.h[-2]
        self.pc = np.array([[p[1], p[2]] for p in prims])
        self.pr = np.array([math.hypot(p[4], p[5]) / 2 if p[0] == "rect" else p[4]
                            for p in prims])

    def nearest(self, x, y, idxs):
        a = self.p[idxs]
        d = self.d[idxs]
        t = np.clip(((x - a[:, 0]) * d[:, 0] + (y - a[:, 1]) * d[:, 1])
                    / self.len2[idxs], 0.0, 1.0)
        dist = np.hypot(x - (a[:, 0] + t * d[:, 0]), y - (a[:, 1] + t * d[:, 1]))
        k = int(np.argmin(dist))
        i = int(idxs[k])
        return i, float(dist[k]), float(self.s[i] + t[k] * self.len[i])

    def pose_at(self, s, lateral, heading_off_deg):
        s = (s % self.total) if self.closed else min(max(s, 0.0), self.total - 1e-3)
        i = int(np.searchsorted(self.s, s, side="right") - 1)
        i = min(max(i, 0), self.n - 1)
        t = (s - self.s[i]) / max(self.len[i], 1e-9)
        pt = self.p[i] + t * self.d[i]
        th = float(self.h[i])
        pt = pt + lateral * np.array([-math.sin(th), math.cos(th)])
        return float(pt[0]), float(pt[1]), th + math.radians(heading_off_deg)


_TRACKS = {}


def get_track(name, mirror):
    key = (name, bool(mirror))
    if key not in _TRACKS:
        path = None if name in GENERAL_TRACKS else _find_sdf(name)
        if name.startswith("sdf:"):
            path = name[4:]
        spec = parse_sdf(path) if path else general_track(name)
        _TRACKS[key] = Track(spec, mirror)
    return _TRACKS[key]


def find_features(tr, per_track, win=0.15, sep=0.8, thresh=0.15):
    """Ranked [(arclength, sign)]; sign +1 left, -1 right, 0 straight."""
    step = tr.total / tr.n
    w = max(2, int(round(win / step / 2)))
    kap = wrap_arr(np.roll(tr.h, -w) - np.roll(tr.h, w))
    score = np.abs(kap)
    if not tr.closed:
        score[:w + 1] = 0
        score[-(w + 1):] = 0
    feats = []
    for i in np.argsort(-score, kind="stable"):
        if score[i] < thresh or len(feats) >= 6:
            break
        si = tr.s[i]
        if any(min(abs(si - sf), tr.total - abs(si - sf)) < sep for sf, _ in feats):
            continue
        ties = [k for k in range(-w, w + 1)
                if score[(i + k) % tr.n] >= score[i] - 1e-6]
        j = (int(i) + int(np.median(ties))) % tr.n
        feats.append((float(tr.s[j]), 1.0 if kap[j] > 0 else -1.0))
    k = 1
    while len(feats) < 2 * per_track and k < 40:
        if tr.closed:
            s = tr.total * k / (2 * per_track + 1)
        else:
            s = 0.6 + (tr.total - 1.8) * k / (2 * per_track + 1)
        if all(min(abs(s - sf), tr.total - abs(s - sf)) >= 0.5 for sf, _ in feats):
            i = int(np.searchsorted(tr.s, s) % tr.n)
            kv = kap[i]
            feats.append((float(s), 0.0 if abs(kv) < 0.02 else (1.0 if kv > 0 else -1.0)))
        k += 1
    if not tr.closed:
        feats = [f for f in feats if 0.35 <= f[0] <= tr.total - 0.7]
    return feats

CAM = dict(x=0.115, z=0.04, pitch=90.0)
AXLE_X = 0.045
HFOV = 1.745329252
W_PX, H_PX = 640, 480
MAX_RANGE = 1.2
_RAYS = {}


def read_urdf_camera(path):
    """camera_joint origin (x, z) and sensor pitch (deg below horizontal)."""
    root = ET.parse(path).getroot()
    out = {}
    for j in root.findall("joint"):
        if j.get("name") == "camera_joint":
            xyz = [float(t) for t in j.find("origin").get("xyz").split()]
            out.update(x=xyz[0], z=xyz[2])
    for g in root.findall("gazebo"):
        pose = g.findtext("sensor/pose")
        if pose:
            out.update(pitch=math.degrees(float(pose.split()[4])))
    return out


def setup_camera():
    """Ground position (relative to the front axle) of every pixel."""
    a = math.radians(CAM["pitch"])
    fpx = (W_PX / 2) / math.tan(HFOV / 2)
    ii, jj = np.meshgrid(np.arange(W_PX), np.arange(H_PX))
    xn = (ii + 0.5 - W_PX / 2) / fpx
    yn = (H_PX / 2 - (jj + 0.5)) / fpx
    dx = math.cos(a) + yn * math.sin(a)
    dy = -xn
    dz = -math.sin(a) + yn * math.cos(a)
    ok = dz < -1e-6
    t = np.where(ok, CAM["z"] / np.where(ok, -dz, 1.0), 0.0)
    F = (CAM["x"] - AXLE_X) + t * dx
    L = t * dy
    ok &= np.hypot(t * dx, t * dy) < MAX_RANGE
    _RAYS["F"] = np.where(ok, F, 0.0).astype(np.float32)
    _RAYS["L"] = np.where(ok, L, 0.0).astype(np.float32)
    _RAYS["ok"] = ok
    _RAYS["cam_fwd"] = CAM["x"] - AXLE_X


def render(tr, x, y, th, line_gray, rng):
    c, s = math.cos(th), math.sin(th)
    F, L = _RAYS["F"], _RAYS["L"]
    px = (x + c * F - s * L).astype(np.float32)
    py = (y + s * F + c * L).astype(np.float32)
    cxy = np.array([x + c * _RAYS["cam_fwd"], y + s * _RAYS["cam_fwd"]])
    near = np.flatnonzero(np.hypot(tr.pc[:, 0] - cxy[0], tr.pc[:, 1] - cxy[1])
                          < tr.pr + MAX_RANGE + 0.1)
    img = np.full((H_PX, W_PX), float(tr.floor), np.float32)
    mask = np.zeros((H_PX, W_PX), bool)
    for k in near:
        p = tr.prims[k]
        if p[0] == "rect":
            _, cx, cy, yaw, sx, sy = p
            cr, sr = math.cos(yaw), math.sin(yaw)
            dx, dy = px - cx, py - cy
            lx = cr * dx + sr * dy
            ly = -sr * dx + cr * dy
            mask |= (np.abs(lx) <= sx / 2) & (np.abs(ly) <= sy / 2)
        else:
            _, cx, cy, r_in, r_out = p
            rr = np.hypot(px - cx, py - cy)
            mask |= (rr <= r_out) & (rr >= r_in)
    mask &= _RAYS["ok"]
    img[mask] = float(line_gray)
    img += rng.normal(0, 2.0, img.shape).astype(np.float32)
    g = np.clip(img, 0, 255).astype(np.uint8)
    return cv2.merge([g, g, g])

FAIL_BASE, FAIL_SPAN = 400.0, 200.0

def simulate(ctrl_path, det_path, params, scen, T):
    """One run. Returns (ok, time_s, max_cross_m, reversals, progress_frac)."""
    _Node.overrides = {**scen.get("fixed", {}), **params}
    cm = load_module(ctrl_path, "ctrl_mod")
    dm = load_module(det_path, "det_mod")
    det = dm.CameraLineDetector()
    ctl = cm.CameraPIDController()
    _Clock.t = 0.0

    tr = get_track(scen["track"], scen["mirror"])
    x, y, th = tr.pose_at(scen["s_feat"] - scen["dist"],
                          scen["lateral"], scen["heading_off"])
    length = scen["length"]
    fail_cd = max(0.06, 0.5 * tr.width + 0.04)
    rng = np.random.default_rng(scen["seed"])
    lag = max(1e-3, scen["lag"])
    v = w = cv = cw = 0.0
    dt = 1.0 / 150.0
    step = 0
    pending = []
    maxcross = 0.0
    flips, last_sign = 0, 0

    all_idx = np.arange(tr.n)
    i_last, cd, s_last = tr.nearest(x, y, all_idx)
    progress = 0.0

    while _Clock.t < T:
        _Clock.t += dt
        step += 1
        if step % 5 == 0:
            pending.append(render(tr, x, y, th, scen["gray"], rng))
            if len(pending) > 1:
                det.image_callback(pending.pop(0))
                m = det.error_pub.last
                if m is not None:
                    ctl.error_callback(m)
                    det.error_pub.last = None
        if step % 3 == 0:
            ctl.control_loop()
            if ctl.cmd_pub.last is not None:
                cv = ctl.cmd_pub.last.linear.x
                cw = ctl.cmd_pub.last.angular.z
                if abs(cw) > 0.4:
                    sg = 1 if cw > 0 else -1
                    if last_sign and sg != last_sign:
                        flips += 1
                    last_sign = sg
        v += (cv - v) * dt / lag
        w += (cw - w) * dt / lag
        x += v * math.cos(th) * dt
        y += v * math.sin(th) * dt
        th += w * dt

        if step % 5 == 0:
            idxs = i_last + np.arange(-40, 101)
            idxs = (idxs % tr.n) if tr.closed else np.clip(idxs, 0, tr.n - 1)
            i_last, cd, s_cur = tr.nearest(x, y, idxs)
            ds = s_cur - s_last
            if tr.closed:
                ds = ((ds + tr.total / 2) % tr.total) - tr.total / 2
            progress += ds
            s_last = s_cur
            maxcross = max(maxcross, cd)
            if cd > fail_cd or progress < -0.15:
                return False, _Clock.t, maxcross, flips, max(0.0, progress) / length
            if progress >= length:
                return True, _Clock.t, maxcross, flips, 1.0
    return False, _Clock.t, maxcross, flips, max(0.0, progress) / length


def run_score(ok, t, maxcross, flips, frac):
    if not ok:
        return FAIL_BASE - FAIL_SPAN * min(1.0, frac)
    return t + 100.0 * maxcross + 2.0 * flips


TRAIN_VARIANTS = [(0.0, 0.0, 15, 0.05), (0.015, 5.0, 55, 0.05)]
HOLD_VARIANTS = [(0.010, -4.0, 30, 0.10), (-0.010, 4.0, 75, 0.10)]


def track_turn_fixed(name, mirror):
    """corner_turn_sign / preferred_branch_side for one track: +1 (left) or
    -1 (right) if every turn on it goes the same way, else 0 (guess)."""
    tr = get_track(name, mirror)
    dirs = {sg for _, sg in find_features(tr, 6) if sg != 0.0}
    if dirs == {1.0}:
        return {"corner_turn_sign": 1.0, "preferred_branch_side": -1.0}
    if dirs == {-1.0}:
        return {"corner_turn_sign": -1.0, "preferred_branch_side": 1.0}
    return {"corner_turn_sign": 0.0}


def build_scenarios(keys, kind, per_track, quick):
    scens, signs = [], []
    variants = TRAIN_VARIANTS if kind == "train" else HOLD_VARIANTS
    if quick:
        per_track = 1
    for (name, mirror) in keys:
        tr = get_track(name, mirror)
        feats = find_features(tr, per_track)
        if kind == "train" or name in GENERAL_TRACKS:
            sel = feats[:per_track]
        else:
            sel = feats[per_track:2 * per_track] or feats[:per_track]
        for fi, (sf, sg) in enumerate(sel):
            lat, hd, gray, lag = variants[fi % len(variants)]
            scens.append(dict(track=name, mirror=mirror, s_feat=sf, dist=0.3,
                              fixed=track_turn_fixed(name, mirror),
                              length=0.9, lateral=lat, heading_off=hd, gray=gray,
                              lag=lag, seed=(1000 if kind == "hold" else 0) + fi))
            if sg != 0.0:
                signs.append(sg)
    return scens, signs


def evaluate(ctrl, det, params, scenarios, T, cutoff=None):
    total, n_ok, aborted = 0.0, 0, False
    for sc in scenarios:
        ok, t, mc, fl, fr = simulate(ctrl, det, params, sc, T)
        total += run_score(ok, t, mc, fl, fr)
        n_ok += ok
        if cutoff is not None and total > cutoff:
            aborted = True
            break
    return total, n_ok, aborted

def build_space(a):
    return [
        ("kp", 0.5, 2.5),
        ("kp_nl", 0.0, 1.5),
        ("ki", 0.0, 0.02),
        ("kd", 0.01, 0.20),
        ("kh", 0.0, 2.5),
        ("derivative_tau", 0.03, 0.15),
        ("max_w", 1.5, 4.0),
        ("max_w_rate", 15.0, 60.0),
        ("v_base", a.v_base_min, a.v_base_max),
        ("v_min", 0.005, 0.02),
        ("max_v_accel", 0.1, 1.0),
        ("error_slowdown", 0.0, 0.7),
        ("rate_slowdown", 0.0, 0.4),
        ("turn_slowdown", 0.0, 0.6),
        ("corner_speed_scale", 0.2, 1.0),
        ("lost_grace_sec", 0.1, 0.8),
        ("recovery_w", 1.0, 3.5),
        ("search_w", 0.8, 3.0),
        ("error_filter_alpha", 0.4, 0.9), 
        ("max_run_fraction", 0.40, 0.85),
    ]


DETECTOR_PARAMS = {"error_filter_alpha", "max_run_fraction", "preferred_branch_side"}


def to_unit(space, params):
    return np.array([(min(hi, max(lo, float(params[n]))) - lo) / (hi - lo)
                     for n, lo, hi in space])


def from_unit(space, x):
    p = {}
    for (n, lo, hi), u in zip(space, x):
        p[n] = round(lo + min(1.0, max(0.0, float(u))) * (hi - lo), 5)
    if "v_min" in p and "v_base" in p:
        p["v_min"] = min(p["v_min"], p["v_base"])
    return p


def read_defaults(ctrl, det, names):
    _Node.overrides = {}
    cn = load_module(ctrl, "ctrl_def").CameraPIDController()
    dn = load_module(det, "det_def").CameraLineDetector()
    d = {}
    for n in names:
        v = cn.params.get(n, dn.params.get(n))
        if v is not None:
            d[n] = float(v)
    return d


_G = {}


def _init(ctrl, det, T, fixed):
    _G.update(ctrl=ctrl, det=det, T=T, fixed=fixed)


def _work(args):
    params, cutoff, scen = args
    return evaluate(_G["ctrl"], _G["det"], {**params, **_G["fixed"]}, scen,
                    _G["T"], cutoff)


def _one(args):
    params, scen = args
    r = simulate(_G["ctrl"], _G["det"], {**params, **_G["fixed"]}, scen, _G["T"])
    return scen["track"], scen["mirror"], r[0], r[1]

def write_outputs(out, ctrl_path, params, info):
    params = {k: v for k, v in params.items()}
    cp = {k: v for k, v in params.items() if k not in DETECTOR_PARAMS}
    dp = {k: v for k, v in params.items() if k in DETECTOR_PARAMS}
    with open(os.path.join(out, "best_params.yaml"), "w") as f:
        f.write("camera_pid_controller:\n  ros__parameters:\n")
        for k, v in cp.items():
            f.write(f"    {k}: {v}\n")
        f.write("line_detector:\n  ros__parameters:\n")
        for k, v in dp.items():
            f.write(f"    {k}: {v}\n")
    with open(os.path.join(out, "best_cli.txt"), "w") as f:
        f.write(" ".join(f"-p {k}:={v}" for k, v in params.items()) + "\n")
    src = open(ctrl_path).read()
    for k, v in cp.items():
        pat = re.compile(r'(param\(\s*"%s"\s*,\s*)([^)]*?)(\))' % re.escape(k))
        src = pat.sub(lambda m: f"{m.group(1)}{v}{m.group(3)}", src, count=1)
    with open(os.path.join(out, "tuned_pd_controller_node.py"), "w") as f:
        f.write(src)
    with open(os.path.join(out, "best.json"), "w") as f:
        json.dump(info, f, indent=2)


def parse_tracks(text, mirror, allow_general=True):
    names = []
    for t in [t.strip() for t in text.split(",") if t.strip()]:
        names += MAIN_TRACKS if t == "main" else (F1_TRACKS if t == "f1" else [t])
    keys = []
    for name in names:
        keys.append((name, False))
        if mirror:
            keys.append((name, True))
    return keys


def main():
    global SDF_DIR, LINE_W
    ap = argparse.ArgumentParser()
    ap.add_argument("--controller", required=True)
    ap.add_argument("--detector", required=True)
    ap.add_argument("--out", default="tune_out")
    ap.add_argument("--sdf-dir", default=SDF_DIR)
    ap.add_argument("--hours", type=float, default=8.0)
    ap.add_argument("--workers", type=int, default=max(1, (os.cpu_count() or 2) - 1))
    ap.add_argument("--sim-seconds", type=float, default=40.0)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--tracks", default="main")
    ap.add_argument("--hold-tracks", default="main,f1,wave,notch,oval")
    ap.add_argument("--mirror", action="store_true")
    ap.add_argument("--corners-per-track", type=int, default=2)
    ap.add_argument("--turn-sign", choices=["auto", "left", "right", "guess"],
                    default="auto")
    ap.add_argument("--v-base-min", type=float, default=0.02)
    ap.add_argument("--v-base-max", type=float, default=0.15)
    ap.add_argument("--v-base-max", type=float, default=0.08)
    ap.add_argument("--camera-x", type=float, default=CAM["x"],
                    help="lens x in base_link (front axle is x=0.045)")
    ap.add_argument("--camera-z", type=float, default=CAM["z"])
    ap.add_argument("--camera-pitch", type=float, default=CAM["pitch"],
                    help="degrees below horizontal; 90 = straight down")
    ap.add_argument("--urdf", default=None,
                    help="robot.urdf: read camera_joint origin and sensor pitch")
    ap.add_argument("--resume", action="store_true")
    ap.add_argument("--quick", action="store_true")
    a = ap.parse_args()

    SDF_DIR = a.sdf_dir
    CAM.update(x=a.camera_x, z=a.camera_z, pitch=a.camera_pitch)
    if a.urdf:
        CAM.update(read_urdf_camera(a.urdf))
    setup_camera()
    try:
        mp.set_start_method("fork")
    except (RuntimeError, ValueError):
        pass

    os.makedirs(a.out, exist_ok=True)
    logf = open(os.path.join(a.out, "tune.log"), "a")

    def log(msg):
        line = f"[{_real_time.strftime('%H:%M:%S')}] {msg}"
        print(line, flush=True)
        logf.write(line + "\n")
        logf.flush()

    if a.quick:
        a.tracks, a.hold_tracks = "triangle", "hexagon_1_0"
    T = 14.0 if a.quick else a.sim_seconds
    log(f"Camera: x={CAM['x']} z={CAM['z']} pitch={CAM['pitch']} deg; sdf-dir={SDF_DIR}")
    try:
        train, tsign = build_scenarios(parse_tracks(a.tracks, a.mirror), "train",
                                       a.corners_per_track, a.quick)
        hold, hsign = build_scenarios(parse_tracks(a.hold_tracks, a.mirror), "hold",
                                      a.corners_per_track, a.quick)
    except (ValueError, FileNotFoundError, ET.ParseError) as e:
        log(f"Track error: {e}\nPut the SDF files in {SDF_DIR} or pass --sdf-dir.")
        return
    signs = set(tsign + hsign)

    if a.turn_sign == "left":
        fixed = {"corner_turn_sign": 1.0, "preferred_branch_side": -1.0}
    elif a.turn_sign == "right":
        fixed = {"corner_turn_sign": -1.0, "preferred_branch_side": 1.0}
    elif a.turn_sign == "guess":
        fixed = {"corner_turn_sign": 0.0}
    else:
        fixed = {}      # auto: each track gets its own sign (see track_turn_fixed)
    per_track = sorted({(sc["track"], sc["fixed"].get("corner_turn_sign"))
                        for sc in train + hold})
    log("Per-track corner_turn_sign (auto): "
        + ", ".join(f"{t}={v:+.0f}" for t, v in per_track))
    log(f"Train runs={len(train)} hold runs={len(hold)} workers={a.workers} "
        f"turn signs seen={sorted(signs)} fixed={fixed}")

    space_all = build_space(a)
    defaults = read_defaults(a.controller, a.detector, [s[0] for s in space_all])
    space = [s for s in space_all if s[0] in defaults]
    missing = [s[0] for s in space_all if s[0] not in defaults]
    if missing:
        log(f"Not found in your files, not tuned: {missing}")
    names = [s[0] for s in space]
    n = len(space)

    base = dict(defaults)
    best_params = dict(base)
    if a.resume and os.path.exists(os.path.join(a.out, "best.json")):
        saved = json.load(open(os.path.join(a.out, "best.json")))["params"]
        best_params.update({k: v for k, v in saved.items() if k in names})
        log("Resumed from best.json")
    best_x = to_unit(space, best_params)
    best_params = from_unit(space, best_x)

    rng = np.random.default_rng(a.seed)
    pool = mp.Pool(a.workers, initializer=_init,
                   initargs=(a.controller, a.detector, T, fixed))
    deadline = _real_time.time() + a.hours * 3600.0
    csv_path = os.path.join(a.out, "results.csv")
    new_csv = not os.path.exists(csv_path)
    csvf = open(csv_path, "a", newline="")
    cw = csv.writer(csvf)
    if new_csv:
        cw.writerow(["eval", "score", "n_ok", "aborted"] + names)

    t0 = _real_time.time()
    best_score, best_ok, _ = pool.apply(_work, ((best_params, None, train),))
    log(f"Baseline: score={best_score:.1f} ok={best_ok}/{len(train)} "
        f"({_real_time.time() - t0:.0f}s for {len(train)} runs)")
    elites = [(best_score, dict(best_params))]
    n_eval, sigma, stall, gen = 0, 0.15, 0, 0
    full = {**best_params, **fixed}
    write_outputs(a.out, a.controller, full,
                  dict(params=full, train_score=best_score, runs=len(train)))
    # help pls
    try:
        while _real_time.time() < deadline:
            gen += 1
            cands = []
            for _ in range(max(2, a.workers * 2)):
                if rng.random() < 0.15:
                    x = rng.random(n)
                else:
                    mask = rng.random(n) < 0.4
                    if not mask.any():
                        mask[rng.integers(n)] = True
                    x = np.clip(best_x + rng.normal(0, sigma, n) * mask, 0, 1)
                cands.append(from_unit(space, x))
            res = pool.map(_work, [(c, best_score, train) for c in cands], chunksize=1)
            improved = False
            for c, (sc, nok, ab) in zip(cands, res):
                n_eval += 1
                cw.writerow([n_eval, f"{sc:.2f}", nok, int(ab)] + [c[k] for k in names])
                if not ab and sc < best_score:
                    best_score, best_params, best_x = sc, c, to_unit(space, c)
                    elites.append((sc, dict(c)))
                    improved = True
            csvf.flush()
            if improved:
                stall, sigma = 0, min(0.30, sigma * 1.15)
                full = {**best_params, **fixed}
                write_outputs(a.out, a.controller, full,
                              dict(params=full, train_score=best_score,
                                   runs=len(train), evals=n_eval))
            else:
                stall += 1
                sigma = max(0.03, sigma * 0.93)
                if stall >= 12:
                    sigma, stall = 0.20, 0
            left = (deadline - _real_time.time()) / 3600.0
            log(f"gen {gen}: evals={n_eval} best={best_score:.1f} sigma={sigma:.3f} "
                f"{'IMPROVED' if improved else ''} ({left:.1f} h left)")
    except KeyboardInterrupt:
        log("Interrupted; validating best candidates.")

    log("Validating on held-out tracks/variants...")
    cand_list = [("baseline", base)] + [(f"elite{i}", p) for i, (_, p) in
                                        enumerate(elites[-5:])]
    scored = []
    for name, p in cand_list:
        h, hok, _ = pool.apply(_work, ((p, None, hold),))
        tr_, trok, _ = pool.apply(_work, ((p, None, train),))
        combined = tr_ / len(train) + h / len(hold)
        scored.append((combined, name, p, tr_, trok, h, hok))
        log(f"  {name}: train={tr_:.1f} ({trok}/{len(train)} ok) "
            f"holdout={h:.1f} ({hok}/{len(hold)} ok) combined={combined:.2f}")
    scored.sort(key=lambda s: s[0])
    _, name, p, tr_, trok, h, hok = scored[0]
    full = {**p, **fixed}
    write_outputs(a.out, a.controller, full,
                  dict(params=full, winner=name, train_score=tr_, train_ok=trok,
                       train_runs=len(train), holdout_score=h, holdout_ok=hok,
                       holdout_runs=len(hold), evals=n_eval))
    log(f"WINNER: {name} (holdout {hok}/{len(hold)} ok)")
    log(f"Best parameters: {full}")

    allsc = train + hold
    for label, params in (("baseline", base), (f"winner ({name})", p)):
        rows = pool.map(_one, [(params, sc) for sc in allsc], chunksize=1)
        agg = {}
        for tname, mir, ok, t in rows:
            key = tname + ("(mirror)" if mir else "")
            a_ = agg.setdefault(key, [0, 0, 0.0])
            a_[0] += ok
            a_[1] += 1
            a_[2] += t if ok else 0.0
        log(f"Per-track results, {label}:")
        for key, (ok, tot, tsum) in agg.items():
            avg = f"{tsum / ok:.1f}s" if ok else "-"
            log(f"    {key:16s} {ok}/{tot} ok   avg finish {avg}")
    log("Files: best_params.yaml, best_cli.txt, tuned_pd_controller_node.py")
    pool.terminate()


if __name__ == "__main__":
    main()