#include "kinematics.h"
#include <math.h>
#include <chrono>

const Limits limits = {
    {{0.0,       3*M_PI/2},
     {0,         M_PI},
     {-M_PI_4,   5*M_PI/4},
     {-3*M_PI_4, 3*M_PI_4},
     {-M_PI_2,   M_PI_2},
     {-M_PI_2,   M_PI_2}}
};

int main() {
    Pose3 pose = Pose3(Vec3(200, 60, 60), RotationMatrix(Vec3(-M_PI_2, -M_PI, 0)));

    std::cout << pose << std::endl << std::endl;

    // time the below function
    // auto start = std::chrono::high_resolution_clock::now();

    // this should take at a maximum 2ms
    Angles angles = InvKin(pose, limits);

    Pose3 outpose = FwdKin(angles);

    // auto end = std::chrono::high_resolution_clock::now();
    // std::cout << "Time taken: " << std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count() << "ns" << std::endl;
    std::cout << angles[0] << " " << angles[1] << " " << angles[2] << " " << angles[3] << " " << angles[4] << " " << angles[5] << std::endl << std::endl;

    std::cout << outpose << std::endl;

    return 0;
}