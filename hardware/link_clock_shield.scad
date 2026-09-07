// Snap-on clock/run jack shield for Arduino UNO R4 WiFi (link_clock)
//
// Sits on the digital header edge. Three single-position male Dupont jumper
// ends (2.54 mm housings) drop into square tubes in the frame and plug into
// D2, D4 and GND on the Arduino's female headers. Two 3.5 mm threaded-bushing
// jacks mount through the top face, hanging out past the board edge so the LED
// matrix stays visible. The Arduino lives in a case whose top plate the header
// pokes ~2 mm through; the frame sits flat on that plate with a recess that
// fits around both header strips, which locates it in X and Y.
//
// Wiring:  D2 -> CLK tip,  D4 -> RUN tip,  GND -> both sleeves
//
// Coordinates: origin at the D0 corner of the board edge, X runs along the
// header toward the USB port, Y positive goes inward over the board,
// Z = 0 is the top surface of the case plate the frame rests on.
// Print upside down (top face on the build plate).

$fn = 64;

// ---- Arduino geometry (from KiCad Arduino_UNO_R3 footprint) ----
pitch         = 2.54;
row_y         = 2.54;         // header row centre, from board edge (Y = 0)
d0_x          = 2.54;
d8_x          = 24.38;        // 0.16" gap after D7
hdr_w         = 2.54;         // female header body width
hdr_above     = 2.0;          // header protrusion above the case plate
hdr_allow     = 0.15;         // per side, recess around the header strips

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

// ---- Frame ----
wall     = 2.0;
floor_t  = 3.0;
top_t    = 2.5;
x_min    = -1.5;
x_max    = 50.5;              // past the end of the 10-pin strip (SCL 47.24)
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
// D7 and D8 is a blank position, not a break), so this is a single slot.
module header_recess() {
    x_first = d0_x;
    x_last  = d8_x + 9 * pitch;                  // SCL
    len = (x_last - x_first) + hdr_w + 2 * hdr_allow;
    w   = hdr_w + 2 * hdr_allow;
    translate([x_first - hdr_w/2 - hdr_allow, row_y - w/2, -0.01])
        cube([len, w, hdr_above + 0.2 + 0.01]);
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
