#include "math_utils.h"

using Angles = std::array<double, 6>;
using Limits = std::array<std::pair<double, double>, 6>;

Angles InvKin(Pose3 & pose, Limits limits);
Pose3 FwdKin(Angles t);