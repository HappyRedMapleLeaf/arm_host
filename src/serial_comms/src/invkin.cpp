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
double Vec3::dot(Vec3 other) {
    return x*other.x + y*other.y + z*other.z;
}


Mat3::Mat3(std::array<std::array<double, 3>, 3> data) : data(data) {};
Mat3::Mat3() : data({{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}}) {};
Mat3::Mat3(double fill) : data({{{fill, fill, fill}, {fill, fill, fill}, {fill, fill, fill}}}) {};
Vec3 Mat3::getColumn(int i) {
    return Vec3(data[0][i], data[1][i], data[2][i]);
}
Vec3 Mat3::getRow(int i) {
    return Vec3(data[i][0], data[i][1], data[i][2]);
}
std::array<double, 3> Mat3::operator[](int i) {
    return data[i];
}
Mat3 Mat3::mul(Mat3 other) {
    std::array<std::array<double, 3>, 3> result{};
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            result[i][j] = this->getRow(i).dot(other.getColumn(j));
        }
    }
    return Mat3(result);
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

double sq(double x) {
    return x * x;
}

std::array<double, 6> CalculateAngles(Vec3 pos, Vec3 dir) {
    std::array<double, 6> angles;
    
    // pre-compute sines and cosines
    std::array<double, 6> s = { sin(angles[0]), sin(angles[1]), sin(angles[2]),
                                sin(dir.x),     sin(dir.y),     sin(dir.z) };
    std::array<double, 6> c = { cos(angles[0]), cos(angles[1]), cos(angles[2]),
                                cos(dir.x),     cos(dir.y),     cos(dir.z) };

    // offset final position to offset base height and arm length
    Vec3 neg_z_axis = { -c[3]*s[4]*c[5] - s[3]*s[5],
                        -s[3]*s[4]*c[5] + c[3]*s[5],
                        -c[4]*c[5]
    };
    Vec3 z_offset = neg_z_axis * armdim::CLAW_LENGTH;

    // desired position relative to frame of link 0
    Vec3 pos0 = pos + z_offset - Vec3(0, 0, armdim::BASE_HEIGHT);
    
    // todo: check bounds
    angles[0] = M_PI_2 + 2*atan(   (pos0.y + sqrt(sq(pos0.x) + sq(pos0.y) - sq(armdim::D2)))
                                 /                   (pos0.x + armdim::D2)                   );

    double d = pos0.y*s[0] - pos0.x*c[0] - armdim::A1;

    angles[2] = M_PI_2 - acos(   (sq(d) + sq(pos0.z) - sq(armdim::A2) - sq(armdim::D4))
                               /                (2*armdim::A2*armdim::D4)               );

    angles[1] = atan2(d, pos0.z) + asin(      (armdim::D4*c[2])
                                         / sqrt(sq(d) + sq(pos0.z)) );

    Mat3 Rinv_4rel0_t3is0{0};
    Rinv_4rel0_t3is0[0][0] = c[0]*c[1]*c[2] - c[0]*s[1]*s[2];
    Rinv_4rel0_t3is0[0][1] = s[0];
    Rinv_4rel0_t3is0[0][2] = c[0]*c[1]*c[2] + c[0]*s[1]*c[2];
    Rinv_4rel0_t3is0[1][0] = s[0]*c[1]*c[2] - s[0]*s[1]*s[2];
    Rinv_4rel0_t3is0[1][1] = -c[0];
    Rinv_4rel0_t3is0[1][2] = s[0]*c[1]*c[2] + s[0]*s[1]*c[2];
    Rinv_4rel0_t3is0[2][0] = s[1]*c[2] + c[1]*s[2];
    Rinv_4rel0_t3is0[2][2] = s[1]*s[2] - c[1]*c[2];

    Mat3 R_6rel0{0};
    R_6rel0[0][0] = c[3]*c[4];
    R_6rel0[2][0] = -s[3]*c[4];
    R_6rel0[0][2] = c[3]*s[4]*c[5] + s[3]*s[4];
    R_6rel0[1][2] = -c[4]*s[5];
    R_6rel0[2][2] = -s[3]*s[4]*c[5] + c[3]*c[5];

    Mat3 R_6rel4_t3is0{ Rinv_4rel0_t3is0.mul(R_6rel0) };

    angles[4] = acos(R_6rel4_t3is0[2][2]);
    angles[5] = acos(R_6rel4_t3is0[2][0] / sin(angles[4]));
    angles[3] = -acos(R_6rel4_t3is0[0][2] / sin(angles[4]));

    return angles;
}