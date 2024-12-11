#include "math_utils.h"

using Angles = std::array<double, 6>;
using Limits = std::array<std::pair<double, double>, 6>;

bool InvKin(Pose3 & pose, Limits limits, Angles & angles);
Pose3 FwdKin(Angles t);

namespace armdim {
    const double CLAW_LENGTH = 43.5;
    const double BASE_HEIGHT = 43.2;

    const double A1 = 23;       // turret side offset
    const double A2 = 100;      // first arm length
    const double D4 = 105.05;   // second arm length
    const double D2 = 48.25;    // turret front offset

    // const double REACH_MULTIPLIER = 1;     // -1 for balance
    const double ELBOW_UP_MULTIPLIER = 1;  // -1 for elbow down
}