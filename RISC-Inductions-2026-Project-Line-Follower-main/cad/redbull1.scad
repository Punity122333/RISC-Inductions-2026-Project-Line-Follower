//redbull
track_width        = 10.0;  // Line width in mm
arena_size         = 600.0; // Bounding dimension across the circuit (mm)
samples_per_seg    = 10;    // Curve resolution (higher = smoother)
show_start_finish  = true;  // Add perpendicular timing/start gate marker
extrude_height     = 0;     // 0 for 2D (SVG/DXF export), >0 for 3D print (mm)

$fn = 28;

redbull_waypoints = [
    [0.35, 0.05], // Start/Finish Straight
    [0.12, 0.07], // Braking for Turn 1
    [0.02, 0.16], // Turn 1 (Niki Lauda Kurve - 90 deg right)
    [0.08, 0.48], // Steep uphill straight
    [0.17, 0.78], // Turn 2 kink
    [0.24, 0.98], // Turn 3 (Remus Hairpin apex)
    [0.32, 0.95],
    [0.60, 0.80], // Downhill straight toward Turn 4
    [0.94, 0.64], // Turn 4 (Rauch Hairpin)
    [0.95, 0.54],
    [0.85, 0.50],
    [0.55, 0.52], // Infield approach
    [0.40, 0.49], // Turn 6 (Left sweep)
    [0.36, 0.35], // Turn 7 (Left downhill)
    [0.46, 0.25], 
    [0.65, 0.24], // Turn 8 (Fast right kink)
    [0.88, 0.22], // Turn 9 (Jochen Rindt Kurve)
    [0.90, 0.10],
    [0.80, 0.02], // Turn 10 (Red Bull Mobile final turn)
    [0.60, 0.04]  // Exit to Start/Finish
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
    spline_pts = interpolate_spline(redbull_waypoints, samples_per_seg);
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