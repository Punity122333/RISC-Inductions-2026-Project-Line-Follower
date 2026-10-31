//silver stone
track_width        = 10.0;  // Line width in mm
arena_size         = 600.0; // Bounding dimension across the circuit (mm)
samples_per_seg    = 10;    // Curve resolution (higher = smoother)
show_start_finish  = true;  // Add perpendicular timing/start gate marker
extrude_height     = 0;     // 0 for 2D (SVG/DXF export), >0 for 3D print (mm)

$fn = 28;

silverstone_waypoints = [
    [0.08, 0.28], // Hamilton Straight (Start/Finish)
    [0.20, 0.20], // Abbey (Turn 1)
    [0.30, 0.24], // Farm Curve (Turn 2)
    [0.36, 0.32], // Village (Turn 3)
    [0.31, 0.40], // The Loop Hairpin (Turn 4)
    [0.24, 0.38], // Aintree (Turn 5)
    [0.15, 0.55], // Wellington Straight
    [0.02, 0.78], // Brooklands (Turn 6)
    [0.12, 0.85], // Luffield (Turn 7)
    [0.20, 0.78], 
    [0.18, 0.88], // Woodcote (Turn 8)
    [0.42, 0.98], // National Pits Straight
    [0.62, 0.99], // Copse (Turn 9)
    [0.69, 0.90],
    [0.65, 0.80], // Maggotts (Turn 10/11)
    [0.74, 0.70], // Becketts (Turn 12/13)
    [0.68, 0.60], // Chapel (Turn 14)
    [0.72, 0.40], // Hangar Straight
    [0.75, 0.15], // Stowe Corner (Turn 15)
    [0.62, 0.08],
    [0.48, 0.04], // Vale (Turn 16)
    [0.42, 0.00], // Club Corner (Turn 17/18)
    [0.24, 0.05]
];

function catmull_pt(p0, p1, p2, p3, t) =
    0.5 * (
        (2 * p1) +
        (-p0 + p2) * t +
        (2 * p0 - 5 * p1 + 4 * p2 - p3) * (t * t) +
        (-p0 + 3 * p1 - 3 * p2 + p3) * (t * t * t)
    );

function interpolate_spline(pts, steps) =
    let(n = len(pts))
    [
        for (i = [0 : n - 1])
            let(
                p0 = pts[(i - 1 + n) % n],
                p1 = pts[i],
                p2 = pts[(i + 1) % n],
                p3 = pts[(i + 2) % n]
            )
            for (s = [0 : steps - 1])
                catmull_pt(p0, p1, p2, p3, s / steps)
    ];

function scale_to_bounds(pts, size) =
    let(
        xs = [for (p = pts) p[0]],
        ys = [for (p = pts) p[1]],
        min_x = min(xs), max_x = max(xs),
        min_y = min(ys), max_y = max(ys),
        span = max(max_x - min_x, max_y - min_y),
        mid_x = (min_x + max_x) / 2,
        mid_y = (min_y + max_y) / 2
    )
    [ for (p = pts) [(p[0] - mid_x) * (size / span), (p[1] - mid_y) * (size / span)] ];

module render_track() {
    spline_pts = interpolate_spline(silverstone_waypoints, samples_per_seg);
    pts = scale_to_bounds(spline_pts, arena_size);
    n = len(pts);

    union() {
        for (i = [0 : n - 1]) {
            hull() {
                translate(pts[i]) circle(d = track_width);
                translate(pts[(i + 1) % n]) circle(d = track_width);
            }
        }
        if (show_start_finish) {
            dir = pts[1] - pts[0];
            angle = atan2(dir[1], dir[0]);
            translate(pts[0])
                rotate([0, 0, angle + 90])
                square([track_width * 3.0, track_width * 0.4], center = true);
        }
    }
}

if (extrude_height > 0) {
    linear_extrude(extrude_height) render_track();
} else {
    render_track();
}