// Snap-on clock/run jack shield for Arduino UNO R4 WiFi (link_clock)
//
// Sits on the digital header edge. Three single-position male Dupont jumper
// ends (2.54 mm housings) drop into square tubes in the frame and plug into
// D2, D4 and GND on the Arduino's female headers. Two 3.5 mm threaded-bushing
// jacks mount through the top face, hanging out past the board edge so the LED
// matrix stays visible. The Arduino lives in a stacked-plate case whose top
// plate the header pokes ~2 mm through; the frame sits flat on that plate with
// a recess that fits around the header housing, which locates it in X and Y.
// The layer under the top plate is set back ~0.7 mm, leaving an undercut. Two
// fingers hook that undercut: a rigid one at the D0 end and a flexible one on
// the header side at the notch between D13 and D12. Fit by hooking the end
// first, then tilting down until the side finger clicks in.
//
// Wiring:  D2 -> CLK tip,  D4 -> RUN tip,  GND -> both sleeves
//
// Coordinates: origin at the D0 corner of the board edge, X runs along the
// header toward the USB port, Y positive goes inward over the board,
// Z = 0 is the top surface of the case plate the frame rests on.
// Print upside down (top face on the build plate).

$fn = 64;

// ---- Arduino geometry ----
// Cross-checked against the UNO R4 WiFi datasheet (ABX00087, rev 8) mechanical
// drawing: board 68.58 x 53.34, header row 2.54 from the edge, D0 2.54 from the
// corner, one continuous 19-position housing (18 pins + blank between D7/D8),
// female headers 8.5 mm tall. Pin centres from the KiCad Arduino_UNO_R3 footprint.
pitch         = 2.54;
row_y         = 2.54;         // header row centre, from board edge (Y = 0)
d0_x          = 2.54;
d8_x          = 24.38;        // 0.16" gap after D7
hdr_w         = 2.54;         // female header body width
hdr_above     = 2.0;          // header protrusion above the case plate
hdr_allow     = 0.20;         // per side, recess around the header housing

pin_x = [ d0_x + 2*pitch,     // D2
          d0_x + 4*pitch,     // D4
          d8_x + 6*pitch ];   // GND on the 10-pin strip (D8..D13, GND)

// ---- Dupont male jumper end ----
dup_body      = 2.54;         // housing cross-section
dup_len       = 14.0;         // housing length
dup_allow     = 0.10;         // per side; tune so the housing is a snug push fit
dup_wall      = 1.2;          // tube wall
wire_slot_w   = 1.8;
wire_bend     = 3.0;          // room above the housing for the wire to turn

// ---- Jack (measured: body h10 x l9 x d8, M6 bushing) ----
jack_bushing_d   = 6.0;
jack_hole_allow  = 0.2;
jack_body_len    = 9.0;       // along bushing axis (vertical here)
jack_pin_len     = 4.0;
jack_spacing     = 15.0;
jack_x0          = 8.0;
jack_y           = -8.0;      // negative = off the board edge

// ---- Case hooks ----
fingers        = true;
plate_t        = 6.0;         // two stacked 3 mm plates above the undercut
undercut       = 0.7;         // set-back of the layer under the top plate
end_setback    = 2.5;         // header housing end (D0 side) -> plate edge
side_setback   = 2.0;         // header housing outer face -> plate edge
hook_h         = 1.2;         // hook thickness under the plate
hook_reach     = undercut - 0.1;
end_finger_t   = 2.0;         // rigid
end_finger_w   = 6.0;
side_finger_t  = 1.2;         // flexes to snap in
side_finger_w  = 3.2;         // notch is ~4 mm wide; leave ~0.4 mm a side
side_finger_x  = d8_x + 4.5 * pitch;           // midway between D12 (d8+4) and D13 (d8+5)
hdr_end_x      = d0_x - hdr_w/2;               // D0 end of the header housing
hdr_face_y     = row_y - hdr_w/2;              // outer face of the header housing
hook_chamfer   = 0.6;

// ---- Frame ----
wall     = 2.0;
floor_t  = 3.0;
top_t    = 2.5;
x_min    = hdr_end_x - end_setback - end_finger_t - 0.5;
x_max    = 46.0;              // stops short of the case screw at the SCL end
                              // (hole centre ~50.6, head edge ~47.8); header
                              // runs out through an open-ended slot
y_min    = -16.0;             // overhang past board edge
y_max    = 5.5;
dup_top  = hdr_above + dup_len;                 // housing sits on the header top
inner_h  = max(dup_top + wire_bend - floor_t, jack_body_len + jack_pin_len + 1);
H        = floor_t + inner_h + top_t;

label_depth = 0.4;
labels      = ["CLK", "RUN"];

tube_o  = dup_body + 2*dup_allow + 2*dup_wall;   // tube outer
tube_i  = dup_body + 2*dup_allow;                // tube inner

module dupont_tube_solid(x) {
    translate([x - tube_o/2, row_y - tube_o/2, 0])
        cube([tube_o, tube_o, floor_t + inner_h]);
}
module dupont_tube_cut(x) {
    // housing bore: floor bottom up to the stop
    translate([x - tube_i/2, row_y - tube_i/2, -0.01])
        cube([tube_i, tube_i, dup_top + 0.01]);
    // wire exit: slot from the top of the housing out the -Y face (toward jacks)
    translate([x - wire_slot_w/2, row_y - tube_o/2 - 0.01, dup_top - 0.01])
        cube([wire_slot_w, tube_o/2, wire_bend + 0.02]);
    translate([x - wire_slot_w/2, row_y - wire_slot_w/2, dup_top - 0.01])
        cube([wire_slot_w, wire_slot_w, wire_bend + 0.02]);
}

// Recess in the underside that fits around the digital header. On the UNO R4
// WiFi the plastic is one continuous strip from D0 to SCL (the 0.16" gap between
// D7 and D8 is a blank position, not a break), so this is a single slot. It is
// open at the SCL end because the frame stops short of the case screw there.
module header_recess() {
    x_first = d0_x;
    x_last  = x_max + 1;                         // open-ended past the frame
    len = (x_last - x_first) + hdr_w + 2 * hdr_allow;
    w   = hdr_w + 2 * hdr_allow;
    translate([x_first - hdr_w/2 - hdr_allow, row_y - w/2, -0.01])
        cube([len, w, hdr_above + 0.2 + 0.01]);
}

// Rigid hook over the D0 end of the case top plate
module end_finger() {
    x_face = hdr_end_x - end_setback;            // plate end face
    drop   = plate_t + hook_h;
    translate([x_face - end_finger_t, row_y - end_finger_w/2, -drop])
        cube([end_finger_t, end_finger_w, drop + floor_t]);
    translate([x_face - 0.01, row_y - end_finger_w/2, -drop])
        cube([hook_reach + 0.01, end_finger_w, hook_h]);
}

// Flexible finger down the header-side face, hooking the notch under D13/D12
module side_finger() {
    y_face = hdr_face_y - side_setback;          // plate side face
    drop   = plate_t + hook_h;
    // finger, joined to the floor strip above
    translate([side_finger_x - side_finger_w/2, y_face - side_finger_t, -drop])
        cube([side_finger_w, side_finger_t, drop + floor_t]);
    // bridge from finger back to the floor (cavity is open-bottomed here)
    translate([side_finger_x - side_finger_w/2, y_face - side_finger_t, 0])
        cube([side_finger_w, -y_face + side_finger_t + 0.5, floor_t]);
    // hook with a lead-in chamfer so it slides over the plate edge
    hull() {
        translate([side_finger_x - side_finger_w/2, y_face - 0.01, -plate_t - hook_h])
            cube([side_finger_w, 0.01, hook_h]);
        translate([side_finger_x - side_finger_w/2, y_face - 0.01, -plate_t - hook_h])
            cube([side_finger_w, hook_reach + 0.01, hook_h - hook_chamfer]);
    }
}

module frame() {
    difference() {
        union() {
            difference() {
                translate([x_min, y_min, 0])
                    cube([x_max - x_min, y_max - y_min, H]);
                // interior cavity
                translate([x_min + wall, y_min + wall, floor_t])
                    cube([x_max - x_min - 2*wall, y_max - y_min - 2*wall, inner_h]);
                // open bottom over the overhang (jacks and wires go in from below)
                translate([x_min + wall, y_min + wall, -0.01])
                    cube([x_max - x_min - 2*wall, -y_min - wall - 0.5, floor_t + 0.02]);
            }
            for (x = pin_x) dupont_tube_solid(x);
            if (fingers) { end_finger(); side_finger(); }
        }
        for (x = pin_x) dupont_tube_cut(x);
        header_recess();

        for (j = [0 : len(labels) - 1]) {
            jx = jack_x0 + j * jack_spacing;
            translate([jx, jack_y, floor_t + inner_h - 0.01])
                cylinder(d = jack_bushing_d + jack_hole_allow, h = top_t + 0.02);
            translate([jx, jack_y + 6.0, H - label_depth])
                linear_extrude(label_depth + 0.01)
                    text(labels[j], size = 2.2, halign = "center",
                         valign = "center", font = "Liberation Sans:style=Bold");
        }
    }
}

frame();

echo(str("Frame X=", x_max - x_min, " Y=", y_max - y_min, " Z=", H,
         "  tube bore=", tube_i, " sq"));
