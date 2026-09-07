// Snap-on clock/run jack shield for Arduino UNO R4 WiFi (link_clock)
//
// Sits on the two digital header rows (D0-D7 and D8-SCL). Two full-length
// 2.54 mm male pin header strips press into pockets in the underside and plug
// into the Arduino's female headers; they locate and retain the part.
// Two 3.5 mm threaded-bushing jacks mount through the top face, hanging out
// past the board edge so nothing on the board is covered.
//
// Wiring:  D2 -> CLK tip,  D4 -> RUN tip,  GND (10-pin strip) -> both sleeves
//
// Coordinates: origin at the pin-1 corner of the D0 pin, X runs along the
// header toward the USB port, Y positive goes inward over the board,
// Z = 0 is the top surface of the Arduino's female headers.
// Print upside down (top face on the build plate).

$fn = 64;

// ---- Header strips (from KiCad Arduino_UNO_R3 footprint) ----
pitch        = 2.54;
row_y        = 2.54;          // header row distance from board edge
d0_x         = 2.54;          // D0 centre
d8_x         = 24.38;         // D8 centre (0.16" gap after D7)
n8           = 8;
n10          = 10;

// ---- Header pocket tuning (resin: add allowance) ----
hdr_body     = 2.54;          // header plastic cross-section
hdr_body_h   = 2.5;           // header plastic height
pocket_allow = 0.12;          // per side
pin_hole     = 0.9;           // square, through the floor
pin_above    = 6.0;           // pin length above header body

// ---- Jack (measured: body h10 x l9 x d8, M6 bushing) ----
jack_bushing_d   = 6.0;
jack_hole_allow  = 0.2;
jack_body        = [10, 8];   // plan-view body size under the panel [X, Y]
jack_body_len    = 9.0;       // along bushing axis (vertical here)
jack_pin_len     = 4.0;
jack_spacing     = 15.0;
jack_x0          = 8.0;       // first jack centre X
jack_y           = -8.0;      // jack centre Y (negative = off the board edge)

// ---- Frame ----
wall        = 2.0;
floor_t     = 3.0;            // base plate over the headers
top_t       = 2.5;            // panel the jacks mount through
x_min       = -1.5;
x_max       = 50.5;           // just past SCL (47.24)
y_min       = -16.0;          // overhang past board edge
y_max       = 5.5;            // covers header rows (edge to 5.08)
inner_h     = max(pin_above + hdr_body_h + 1,
                  jack_body_len + jack_pin_len + 1);
H           = floor_t + inner_h + top_t;

label_depth = 0.4;
labels      = ["CLK", "RUN"];

module header_pocket(x0, n) {
    len = (n - 1) * pitch + hdr_body + 2 * pocket_allow;
    w   = hdr_body + 2 * pocket_allow;
    translate([x0 - hdr_body/2 - pocket_allow, row_y - w/2, -0.01])
        cube([len, w, hdr_body_h + 0.1 + 0.01]);
    // pin holes through the floor
    for (i = [0 : n - 1])
        translate([x0 + i*pitch - pin_hole/2, row_y - pin_hole/2, -0.01])
            cube([pin_hole, pin_hole, floor_t + 0.02]);
}

module frame() {
    difference() {
        // outer block
        translate([x_min, y_min, 0]) cube([x_max - x_min, y_max - y_min, H]);

        // interior cavity above the floor (over header region and overhang)
        translate([x_min + wall, y_min + wall, floor_t])
            cube([x_max - x_min - 2*wall, y_max - y_min - 2*wall, inner_h]);

        // open bottom over the overhang so jacks/wires go in from below
        translate([x_min + wall, y_min + wall, -0.01])
            cube([x_max - x_min - 2*wall, -y_min - wall - 0.5, floor_t + 0.02]);

        header_pocket(d0_x, n8);
        header_pocket(d8_x, n10);

        for (j = [0 : len(labels) - 1]) {
            jx = jack_x0 + j * jack_spacing;
            // bushing hole
            translate([jx, jack_y, floor_t + inner_h - 0.01])
                cylinder(d = jack_bushing_d + jack_hole_allow, h = top_t + 0.02);
            // engraved label on the top face, inboard of the jack
            translate([jx, jack_y + 6.0, H - label_depth])
                linear_extrude(label_depth + 0.01)
                    text(labels[j], size = 2.2, halign = "center",
                         valign = "center", font = "Liberation Sans:style=Bold");
        }
    }
}

frame();

echo(str("Frame size X=", x_max - x_min, " Y=", y_max - y_min, " Z=", H));
