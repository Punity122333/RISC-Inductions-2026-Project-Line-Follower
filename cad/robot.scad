$fn = 48;
part_id = 0;
show_parts = 1;
show_screws = 1;

ch_x0 = -65;  ch_len = 95;
ch_wid = 110; ch_z0 = 10;  ch_h = 24;
ch_top = ch_z0 + ch_h;
pi_z = ch_top + pi_stand;
wall = 2; floor_t = 2; deck_t = 2; corner_r = 4;
deck_z0 = ch_top - deck_t;

wheel_r = 20; wheel_w = 16; axle_z = 20;
wheel_x = 45; wheel_y = 65;

post_x = 25; post_y = 30; post_w = 10; post_h = 30; post_zc = 49;
arm_xc = 51.5; arm_l = 63; arm_w = 30; arm_t = 6; arm_zc = 67;
cam_x = 74; cam_z = 59; cam_l = 18; cam_w = 30; cam_h = 10;

mot_w = 12; mot_h = 10; mot_gear = 15; mot_can = 10; mot_enc = 8;
mot_len = mot_gear + mot_can + mot_enc;
pod_x1 = 56;  pod_y0 = 20;
clamp_dx = 9; clamp_y = 45;

pi_l = 85; pi_w = 56; pi_x0 = -65; pi_y0 = -28; pi_stand = 15;
pi_pat = [[3.5,-24.5],[3.5,24.5],[61.5,-24.5],[61.5,24.5]];
esp_l = 51; esp_w = 28; esp_x = -36.5;
esp_pads = [[-22,-11],[-22,11],[22,-11],[22,11]];
buck_l = 43; buck_w = 21; buck_x = -2.5; buck_ys = [38.2, -38.2];
buck_pads = [[-16,-7],[-16,7],[16,-7],[16,7]];
drv_s = 20; drv_x = 13; drv_y = 40;
drv_pads = [[-8,-8],[-8,8],[8,-8],[8,8]];
bat_l = 68; bat_w = 37; bat_h = 19; bat_x = -28;
posts = [[-60,51.5],[-60,-51.5],[-17.5,51.5],[-17.5,-51.5],[27,51.5],[27,-51.5]];
rear_pillars = [[-45-clamp_dx, clamp_y], [-45+clamp_dx, clamp_y]];


module rbox(x0, y0, z0, l, w, h, r) {
    hull() for (px=[x0+r, x0+l-r]) for (py=[y0+r, y0+w-r]) translate([px, py, z0]) cylinder(r=r, h=h);
}
module screw(d=2.5, l=6, hd=4.5, hh=2) {
    color([0.62,0.62,0.66]) difference() {
        union() { translate([0,0,-l]) cylinder(d=d, h=l, $fn=16); cylinder(d=hd, h=hh, $fn=24); }
        translate([0,0,hh-1]) cylinder(d=d*0.6, h=2, $fn=6);
    }
}
module outline(l, w, t=1, h=0.6) {
    difference() { cube([l, w, h]); translate([t, t, -0.1]) cube([l-2*t, w-2*t, h+0.2]); }
}
module standoff(h, od=6, id=2.2) { difference() { cylinder(d=od, h=h); translate([0,0,-0.1]) cylinder(d=id, h=h+0.2); } }

module mot_slot(sy) {
    mirror([0, sy < 0 ? 1 : 0, 0]) {
        translate([-(mot_w+0.6)/2, 53 - mot_len - 0.5, -(mot_h+0.6)/2]) cube([mot_w+0.6, mot_len+0.5, 60]);
        translate([0, 52, 0]) rotate([-90,0,0]) cylinder(r=2.2, h=5);
    }
}
module motor_ghost() {
    translate([0, 53, 0]) {
        color([0.85,0.85,0.88]) rotate([-90,0,0]) difference() {
            cylinder(r=1.5, h=13);
            translate([0.9,-2,5]) cube([2,4,9]);
        }
        color([0.8,0.7,0.3]) translate([-mot_w/2, -mot_gear, -mot_h/2]) cube([mot_w, mot_gear, mot_h]);
        color([0.75,0.75,0.78]) translate([0, -mot_gear, 0]) rotate([90,0,0]) intersection() {
            cylinder(r=mot_w/2-0.2, h=mot_can);
            cube([mot_w, mot_h, 2*mot_can], center=true);
        }
        color([0.1,0.45,0.2]) translate([-mot_w/2, -mot_gear-mot_can-mot_enc, -mot_h/2]) cube([mot_w, mot_enc-3, mot_h]);
        color([0.1,0.1,0.1]) translate([-mot_w/2+1, -mot_gear-mot_can-3, -mot_h/2]) cube([mot_w-2, 3, mot_h]);
        color([0.8,0.1,0.1]) translate([-2, -mot_len, -1]) cube([4, 6, 2]);
    }
}
module motor_clamp() {
    color([0.3,0.3,0.3]) difference() {
        translate([-clamp_dx-3.5, clamp_y-4.5, mot_h/2+0.4]) cube([2*clamp_dx+7, 9, 2]);
        for (s=[-1,1]) translate([s*clamp_dx, clamp_y, 0]) cylinder(r=1.2, h=30);
        translate([-mot_w/2+0.5, clamp_y-5, mot_h/2+0.3]) cube([mot_w-1, 10, 0.6]);
    }
}

module chassis() {
    color([1,0,0]) difference() {
        union() {
            rbox(ch_x0, -ch_wid/2, ch_z0, ch_len, ch_wid, ch_h - deck_t, corner_r);
            for (s=[-1,1]) translate([ch_x0+ch_len-4, s>0 ? pod_y0 : -ch_wid/2, ch_z0])
                cube([pod_x1 - (ch_x0+ch_len-4), ch_wid/2 - pod_y0, 20]);
        }
        translate([0,0,0]) rbox(ch_x0+wall, -ch_wid/2+wall, ch_z0+floor_t, ch_len-2*wall, ch_wid-2*wall, 40, corner_r-wall);
        for (s=[-1,1]) for (x=[-wheel_x, wheel_x]) translate([x, 0, axle_z]) mot_slot(s);
        for (s=[-1,1]) for (dx=[-clamp_dx, clamp_dx]) translate([wheel_x+dx, s*clamp_y, axle_z+mot_h/2-8]) cylinder(r=1.0, h=10);
        translate([bat_x-bat_l/2-0.3, -bat_w/2-0.3, ch_z0+floor_t-1]) cube([bat_l+0.6, bat_w+0.6, 1.2]);
        for (y=[-12,12]) for (dx=[-20,20]) translate([bat_x+dx-1.5, y-5, ch_z0]) cube([3,10,floor_t]);
        for (s=[-1,1]) translate([drv_x-drv_s/2-0.3, s*drv_y-drv_s/2-0.3, ch_z0+floor_t-1]) cube([drv_s+0.6, drv_s+0.6, 1.2]);
        translate([22, 17, ch_z0+floor_t]) cube([4, 6, 40]);
        translate([22, -23, ch_z0+floor_t]) cube([4, 6, 40]);
        translate([ch_x0-0.1, -10, 18]) cube([wall+0.2, 20, 8]);
        translate([ch_x0-0.1, -28, 14]) cube([wall+0.2, 12, 6]);
        translate([-17.5-10, -15, ch_z0-0.1]) cube([20, 30, 0.8]);
    }
    color([1,0,0]) {
        for (p=posts) translate([p[0], p[1], ch_z0+floor_t]) difference() {
            cylinder(r=2.6, h=deck_z0-ch_z0-floor_t);
            translate([0,0,2]) cylinder(r=1.05, h=40);
        }
        for (s=[-1,1]) for (d=drv_pads) translate([drv_x+d[0], s*drv_y+d[1], ch_z0+floor_t]) standoff(3, 5, 1.8);
        for (s=[-1,1]) for (p=rear_pillars) translate([p[0], s*p[1], ch_z0+floor_t]) difference() {
            cylinder(r=2.2, h=axle_z+mot_h/2-ch_z0-floor_t);
            translate([0,0,2]) cylinder(r=0.9, h=30);
        }
    }
}

module deck() {
    color([1,0.3,0.3]) difference() {
        union() {
            rbox(ch_x0, -ch_wid/2, deck_z0, ch_len, ch_wid, deck_t, corner_r);
            for (p=pi_pat) translate([pi_x0+p[0], p[1], ch_top]) cylinder(d=6, h=pi_stand);
            for (p=esp_pads) translate([esp_x+p[0], p[1], ch_top]) cylinder(d=5, h=3);
            translate([esp_x-esp_l/2, -esp_w/2, ch_top]) outline(esp_l, esp_w);
            translate([pi_x0, pi_y0, ch_top]) outline(pi_l, pi_w);
            for (y=buck_ys) for (p=buck_pads) translate([buck_x+p[0], y+p[1]*0.0 + (y>0 ? p[1]*1 : p[1]*1), ch_top]) cylinder(d=4.5, h=3);
        }
        for (p=pi_pat) translate([pi_x0+p[0], p[1], deck_z0-1]) cylinder(d=2.2, h=pi_stand+deck_t+2);
        for (p=esp_pads) translate([esp_x+p[0], p[1], deck_z0-1]) cylinder(d=1.8, h=deck_t+5);
        for (y=buck_ys) for (p=buck_pads) translate([buck_x+p[0], y+p[1], deck_z0-1]) cylinder(d=1.8, h=deck_t+5);
        for (p=posts) translate([p[0], p[1], deck_z0-1]) {
            cylinder(d=2.8, h=deck_t+2);
            translate([0,0,deck_t+0.1-1]) cylinder(d1=2.8, d2=5, h=1.01);
        }
        translate([22, 17, deck_z0-1]) cube([4, 6, deck_t+2]);
        translate([22, -23, deck_z0-1]) cube([4, 6, deck_t+2]);
        translate([-35, -4, deck_z0-1]) cube([8, 8, deck_t+2]);
        translate([post_x-post_w/2+1, -post_y/2+3, deck_z0-1]) cube([post_w-2, 6, deck_t+2]);
        for (dy=[-10,10]) translate([post_x, dy, deck_z0-1]) cylinder(d=2.2, h=deck_t+2);
    }
}

module camera_mast() {
    color([0.3,0.3,0.3]) difference() {
        union() {
            translate([post_x-post_w/2, -post_y/2, post_zc-post_h/2]) cube([post_w, post_y, post_h]);
            translate([arm_xc-arm_l/2, -arm_w/2, arm_zc-arm_t/2]) cube([arm_l, arm_w, arm_t]);
            hull() {
                translate([post_x+post_w/2, -post_y/2, arm_zc-arm_t/2-0.01]) cube([0.1, post_y, 0.01]);
                translate([post_x+post_w/2, -post_y/2, arm_zc-arm_t/2-10]) cube([0.1, post_y, 0.01]);
                translate([post_x+post_w/2+10, -post_y/2, arm_zc-arm_t/2-0.01]) cube([0.1, post_y, 0.01]);
            }
        }
        translate([post_x-2, -post_y/2+3, ch_top-1]) cube([4, 6, 32]);
        translate([cam_x-1, 0, arm_zc-4]) cube([14, 20, 3]);
        for (dy=[-10,10]) translate([post_x, dy, ch_top-1]) cylinder(d=2.2, h=12);
        for (dx=[-1,1]) for (dy=[-1,1]) translate([cam_x+dx*6.5, dy*10.5, arm_zc-5]) cylinder(d=1.8, h=10);
    }
}

module wheel() {
    color([0.12,0.12,0.12]) rotate([90,0,0]) {
        difference() {
            union() {
                difference() {
                    cylinder(r=wheel_r, h=wheel_w, center=true);
                    cylinder(r=wheel_r-3.5, h=wheel_w+1, center=true);
                }
                cylinder(r=6, h=wheel_w-4, center=true);
                for (i=[0:5]) rotate([0,0,i*60]) translate([0,-1.5,-3]) cube([wheel_r-3, 3, 6]);
            }
            cylinder(d=3.1, h=wheel_w+1, center=true);
            for (i=[0:19]) rotate([0,0,i*18]) translate([wheel_r-0.5, -1, -wheel_w/2-1]) cube([2, 2, wheel_w+2]);
            translate([0,0,wheel_w/2-2]) cylinder(r=5, h=3);
        }
    }
    color([0.7,0.7,0.72]) translate([0, 0, 0]) rotate([0,0,0]) cylinder(r=0.01, h=0.01);
}
module wheel_setscrew() {
    color([0.62,0.62,0.66]) translate([0,0,wheel_r-0]) cylinder(d=0.01,h=0.01);
}

module ghosts() {
    for (s=[-1,1]) for (x=[-wheel_x, wheel_x]) {
        translate([x,0,axle_z]) mirror([0, s<0?1:0, 0]) { motor_ghost(); motor_clamp(); }
    }
    color([0.9,0.9,0.25,0.9]) translate([bat_x-bat_l/2, -bat_w/2, ch_z0+floor_t]) cube([bat_l, bat_w, bat_h]);
    color([0.1,0.1,0.1]) for (dx=[-20,20]) translate([bat_x+dx-3, -17, ch_z0+floor_t+bat_h]) cube([6, 34, 0.8]);
    color([0.1,0.5,0.2]) translate([bat_x+bat_l/2-1, -10, ch_z0+floor_t+2]) cube([1, 20, 8]);
    color([0.15,0.15,0.6]) for (s=[-1,1]) translate([drv_x-drv_s/2, s*drv_y-drv_s/2, ch_z0+floor_t+3]) {
        cube([drv_s, drv_s, 1.6]);
        translate([drv_s/2-4, drv_s/2-3, 1.6]) color([0.05,0.05,0.05]) cube([8, 6, 1.4]);
    }
    color([0.8,0.1,0.1]) translate([ch_x0+0.5, -9, 18.5]) cube([7, 18, 7]);
    color([0.9,0.9,0.9]) translate([ch_x0+0.5, -28+2, 14.5]) cube([6, 8, 5]);
    color([0.1,0.5,0.8]) for (y=buck_ys) translate([buck_x-buck_l/2, y-buck_w/2, ch_top+3]) {
        cube([buck_l, buck_w, 1.6]);
        color([0.2,0.2,0.2]) translate([buck_l/2-5, buck_w/2-5, 1.6]) cube([10, 10, 8]);
        color([0.7,0.7,0.1]) translate([4, 4, 1.6]) cylinder(r=3, h=7);
    }
    color([0.9,0.55,0.2]) translate([post_x-1.5, -9, ch_top+0.5]) cube([3, 6, pi_z+1-ch_top]);
    color([0.9,0.55,0.2]) translate([post_x-1.5, -9, arm_zc-4]) cube([cam_x+6-(post_x-1.5), 6, 0.3]);
    color([0.05,0.3,1]) translate([cam_x-cam_l/2, -cam_w/2, cam_z-cam_h/2]) cube([cam_l, cam_w, cam_h]);
    color([0.05,0.05,0.05]) translate([cam_x, 0, cam_z-cam_h/2-1]) cylinder(r=4, h=1.5);
}

module fasteners() {
    for (p=posts) translate([p[0], p[1], ch_top]) screw(2.5, 8, 4.8, 1.5);
    for (y=buck_ys) for (p=buck_pads) translate([buck_x+p[0], y+p[1], ch_top+3+1.6]) screw(2, 5, 3.8, 1.6);
    for (s=[-1,1]) for (d=drv_pads) translate([drv_x+d[0], s*drv_y+d[1], ch_z0+floor_t+3+1.6]) screw(2, 5, 3.8, 1.6);
    for (s=[-1,1]) for (x=[-wheel_x, wheel_x]) for (dx=[-clamp_dx, clamp_dx])
        translate([x+dx, s*clamp_y, axle_z+mot_h/2+0.4+2]) screw(2, 6, 3.8, 1.6);
    for (dy=[-10,10]) translate([post_x, dy, deck_z0]) rotate([180,0,0]) screw(2.2, 10, 4.5, 2);
    for (dx=[-1,1]) for (dy=[-1,1]) translate([cam_x+dx*6.5, dy*10.5, arm_zc+arm_t/2]) screw(1.8, 6, 3.6, 1.5);
    color([0.62,0.62,0.66]) for (x=[-wheel_x, wheel_x]) for (y=[-wheel_y, wheel_y])
        translate([x, y + (y>0 ? 1 : -1)*(wheel_w/2-2.5), axle_z + 6]) cylinder(d=2, h=1.5, $fn=12);
}

module assembly() {
    chassis(); deck(); camera_mast();
    for (x=[-wheel_x, wheel_x]) for (y=[-wheel_y, wheel_y]) translate([x,y,axle_z]) wheel();
    if (show_parts) ghosts();
    if (show_screws) fasteners();
}

if (part_id==1) chassis();
if (part_id==2) deck();
if (part_id==3) camera_mast();
if (part_id==4) wheel();
if (part_id==5) motor_clamp();
if (part_id==0) assembly();