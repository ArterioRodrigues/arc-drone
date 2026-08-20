#include "mixer.h"
#include <algorithm>
#include <cmath>

// Motor layout, viewed from above:
//   m1 front-left    m2 front-right
//   m3 back-left     m4 back-right
// Roll splits left (m1/m3) from right (m2/m4), pitch splits front (m1/m2) from
// back (m3/m4), and yaw splits the two diagonals.
Motors Mixer::compute(double base, double roll, double pitch, double yaw) {
    double correction[4] = {
        -roll + pitch + yaw,  // m1
        +roll + pitch - yaw,  // m2
        -roll - pitch - yaw,  // m3
        +roll - pitch + yaw,  // m4
    };

    double lowest = correction[0];
    double highest = correction[0];
    for (int i = 1; i < 4; i++) {
        lowest = std::min(lowest, correction[i]);
        highest = std::max(highest, correction[i]);
    }

    // Attitude is controlled by the differences between motors, not their
    // absolute values. Clamping each motor independently would flatten those
    // differences the moment one hit a limit - both sides pinned at 48 look
    // identical, so the quad stops responding exactly when it needs to most.
    // Shrink the whole spread instead, which preserves the ratios between
    // motors and so keeps the commanded attitude, just with less strength.
    //
    // Note this scales rather than shifting the base throttle to make room: a
    // shift would spool every motor up when the pilot did not ask for it, which
    // is alarming on the bench and unsafe in the hand.
    double scale = 1.0;
    if (highest > 0.0) {
        scale = std::min(scale, (MOTOR_MAX - base) / highest);
    }
    if (lowest < 0.0) {
        scale = std::min(scale, (base - MOTOR_MIN) / -lowest);
    }
    scale = std::max(scale, 0.0);

    Motors motors;
    motors.m1 = (int)lround(base + correction[0] * scale);
    motors.m2 = (int)lround(base + correction[1] * scale);
    motors.m3 = (int)lround(base + correction[2] * scale);
    motors.m4 = (int)lround(base + correction[3] * scale);

    return motors;
}
