#include "invkin.h"
#include <math.h>

Vec3::Vec3(double x, double y, double z) : x(x), y(y), z(z) {};
Vec3 Vec3::operator+(Vec3 other) {
    return Vec3(x + other.x, y + other.y, z + other.z);
}
Vec3 Vec3::operator-(Vec3 other) {
    return Vec3(x - other.x, y - other.y, z - other.z);
}
Vec3 Vec3::operator*(double scalar) {
    return Vec3(x * scalar, y * scalar, z * scalar);
}
Vec3 Vec3::operator-() {
    return Vec3(-x, -y, -z);
}

Mat3::Mat3(std::array<std::array<double, 3>, 3> data) : data(data) {};
Mat3::Mat3() : data({{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}}) {};
Vec3 Mat3::getColumn(int i) {
    return Vec3(data[0][i], data[1][i], data[2][i]);
}

Mat3 RotationMatrix(Vec3 angles) {
    return Mat3({
        std::array<double, 3>{
            + cos(angles.x)*cos(angles.y),
            + cos(angles.x)*sin(angles.y)*sin(angles.z) - sin(angles.x)*cos(angles.z),
            + cos(angles.x)*sin(angles.y)*cos(angles.z) + sin(angles.x)*sin(angles.z)
        },
        std::array<double, 3>{
            + sin(angles.x)*cos(angles.y),
            + sin(angles.x)*sin(angles.y)*sin(angles.z) + cos(angles.x)*cos(angles.z),
            + sin(angles.x)*sin(angles.y)*cos(angles.z) - cos(angles.x)*sin(angles.z)
        },
        std::array<double, 3>{
            - sin(angles.y),
            + cos(angles.y)*sin(angles.z),
            + cos(angles.y)*cos(angles.z)
        }
    });
}



std::array<double, 6> CalculateAngles(Vec3 pos, Vec3 dir) {
    std::array<double, 6> angles;

    // offset final position to offset base height and arm length
    Mat3 rot = RotationMatrix(dir);
    Vec3 z_offset = -rot.getColumn(3) * CLAW_LENGTH;
    Vec3 f0_pos = pos + z_offset - Vec3(0, 0, BASE_HEIGHT);
    
    // todo: check bounds
    angles[0] = 2 * atan((y + sqrt(x * x + y * y - OFFSET_X * OFFSET_X)) / (x + OFFSET_X));

    double d0 = y * cos(angles[0]) - x * sin(angles[0]) - OFFSET_Y;
    double d1 = z - OFFSET_Z;

    angles[2] = +acos((d0 * d0 + d1 * d1 - L1 * L1 - L2 * L2) / (2 * L1 * L2));

    angles[1] = atan2(d1, d0) - asin((L2 * sin(angles[2])) / sqrt(d0 * d0 + d1 * d1));
}