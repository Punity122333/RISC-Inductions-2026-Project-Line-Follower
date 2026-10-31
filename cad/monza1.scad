//monza
track_width        = 10.0;  // Line width in mm
arena_size         = 600.0; // Bounding dimension across the circuit (mm)
samples_per_seg    = 10;    // Curve resolution (higher = smoother)
show_start_finish  = true;  // Add perpendicular timing/start gate marker
extrude_height     = 0;     // 0 for 2D (SVG/DXF export), >0 for 3D print (mm)

$fn = 28;

monza_waypoints = [
    [0.10, 0.40], // Main Straight (Start/Finish)
    [0.10, 0.72], // Braking for Rettifilo
    [0.13, 0.78], // Variante del Rettifilo (T1/T2 Chicane)
    [0.09, 0.84],
    [0.16, 0.94], // Curva Grande entry (Curva Biassono)
    [0.32, 0.98], // Curva Grande apex
    [0.45, 0.92], // Approach to Roggia
    [0.43, 0.88], // Variante della Roggia (T4/T5 Chicane)
    [0.48, 0.84],
    [0.58, 0.83], // Curva di Lesmo 1 (T6)
    [0.62, 0.78],
    [0.59, 0.70], // Curva di Lesmo 2 (T7)
    [0.50, 0.65],
    [0.38, 0.52], // Curva del Serraglio
    [0.32, 0.38], // Approach to Ascari
    [0.27, 0.33], // Variante Ascari Entry (T8)
    [0.34, 0.28], // Ascari Middle (T9)
    [0.30, 0.22], // Ascari Exit (T10)
    [0.30, 0.12], // Rettifilo del Serraglio (Back Straight)
    [0.30, 0.04], // Entry to Parabolica
    [0.22, 0.00], // Curva Parabolica / Alboreto apex (T11)
    [0.12, 0.07], // Parabolica exit
    [0.10, 0.20]  // Exit onto Main Straight
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
    spline_pts = interpolate_spline(monza_waypoints, samples_per_seg);
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