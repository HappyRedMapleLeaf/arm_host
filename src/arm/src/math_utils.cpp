#include "math_utils.h"


Vec3::Vec3(double x, double y, double z) : x(x), y(y), z(z) {};
Vec3::Vec3() : x(0), y(0), z(0) {};
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
Vec3 Vec3::cross(Vec3 other) {
    return Vec3(
        y*other.z - z*other.y,
        z*other.x - x*other.z,
        x*other.y - y*other.x
    );
}
std::ostream& operator<<(std::ostream & os, const Vec3 & vec) {
    os << vec.x << " " << vec.y << " " << vec.z;
    return os;
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
void Mat3::setColumn(int i, Vec3 vec) {
    this->data[0][i] = vec.x;
    this->data[1][i] = vec.y;
    this->data[2][i] = vec.z;
}
void Mat3::setRow(int i, Vec3 vec) {
    this->data[i][0] = vec.x;
    this->data[i][1] = vec.y;
    this->data[i][2] = vec.z;
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
Mat3 Mat3::transpose() {
    std::array<std::array<double, 3>, 3> result{};
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            result[i][j] = this->data[j][i];
        }
    }
    return Mat3(result);
}
std::array<double, 3> & Mat3::operator[](int i) {
    return data[i];
}
const std::array<double, 3> & Mat3::operator[](int i) const {
    return data[i];
}
std::ostream& operator<<(std::ostream & os, const Mat3 & mat) {
    os << mat[0][0] << " " << mat[0][1] << " " << mat[0][2] << std::endl;
    os << mat[1][0] << " " << mat[1][1] << " " << mat[1][2] << std::endl;
    os << mat[2][0] << " " << mat[2][1] << " " << mat[2][2];
    return os;
}


Pose3::Pose3(Vec3 pos, Mat3 dir) : pos(pos), dir(dir) {};
Pose3::Pose3() : pos(Vec3()), dir(Mat3()) {};
std::ostream& operator<<(std::ostream & os, const Pose3 & pose) {
    os << ">>> Position: " << pose.pos << std::endl;
    os << ">>> Direction:\n" << pose.dir;
    return os;
}


double sq(double x) {
    return x * x;
}


// xyz fixed or zyx euler
Mat3 RotationMatrix(Vec3 angles) {
    return Mat3({
        std::array<double, 3>{
            + cos(angles.z)*cos(angles.y),
            + cos(angles.z)*sin(angles.y)*sin(angles.x) - sin(angles.z)*cos(angles.x),
            + cos(angles.z)*sin(angles.y)*cos(angles.x) + sin(angles.z)*sin(angles.x)
        },
        std::array<double, 3>{
            + sin(angles.z)*cos(angles.y),
            + sin(angles.z)*sin(angles.y)*sin(angles.x) + cos(angles.z)*cos(angles.x),
            + sin(angles.z)*sin(angles.y)*cos(angles.x) - cos(angles.z)*sin(angles.x)
        },
        std::array<double, 3>{
            - sin(angles.y),
            + cos(angles.y)*sin(angles.x),
            + cos(angles.y)*cos(angles.x)
        }
    });
}