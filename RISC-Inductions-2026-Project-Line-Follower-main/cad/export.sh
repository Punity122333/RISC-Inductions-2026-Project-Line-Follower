#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "$0")"

SCAD="${OPENSCAD:-openscad}"
SRC="robot.scad"
OUT="out"

mkdir -p "$OUT"

stl() {
    echo "  part_id=$1 -> $OUT/$2"
    "$SCAD" -o "$OUT/$2" -D "part_id=$1" -D "show_pose=0" "$SRC"
}

stl 1 chassis.stl
stl 2 wheel.stl
stl 3 chassis_collision.stl
stl 4 wheel_collision.stl

echo "  assembly -> $OUT/preview.png"
"$SCAD" -o "$OUT/preview.png" \
    --imgsize=1400,1050 \
    --projection=p \
    --autocenter --viewall \
    --camera=0,0,0,58,0,26,560 \
    -D "part_id=0" -D "show_pose=0" "$SRC"

echo "  poses -> $OUT/pose.echo"
"$SCAD" -o "$OUT/pose.echo" -D "part_id=5" -D "show_pose=1" "$SRC"

echo "  checking"
python3 check_stl.py "$OUT/chassis.stl" "$OUT/wheel.stl" \
    "$OUT/chassis_collision.stl" "$OUT/wheel_collision.stl"

echo "done"
