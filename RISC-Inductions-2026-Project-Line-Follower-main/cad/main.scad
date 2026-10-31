include <robot.scad>
include <monza1.scad>

part_id        = -1;
extrude_height = 0.4;
arena_size     = 1500;
$fn            = 48;

start_pts = scale_to_bounds(
                interpolate_spline(monza_waypoints, samples_per_seg),
                arena_size);
start_dir = start_pts[1] - start_pts[0];
start_ang = atan2(start_dir[1], start_dir[0]);

translate([start_pts[0][0], start_pts[0][1], extrude_height])
    rotate([0, 0, start_ang])
        assembly();