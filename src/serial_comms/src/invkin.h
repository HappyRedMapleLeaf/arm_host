#include <array>

namespace armdim {
    const double CLAW_LENGTH = 43.5;
    const double BASE_HEIGHT = 43.2;

    const double A1 = 23;
    const double A2 = 100;
    const double D4 = 105.05;
    const double D2 = 48.25;
}

class Vec3 {
    public:
        double x{};
        double y{};
        double z{};

        Vec3(double x, double y, double z);

        Vec3 operator+(Vec3 other);
        Vec3 operator-(Vec3 other);
        Vec3 operator*(double scalar);
        Vec3 operator-();
        double dot(Vec3 other);
};

class Mat3 {
    public:
        std::array<std::array<double, 3>, 3> data{};

        Mat3(std::array<std::array<double, 3>, 3> data);
        Mat3(double fill);
        Mat3();
        
        std::array<double, 3> operator[](int i);

        Vec3 getColumn(int i);
        Vec3 getRow(int i);
        Mat3 mul(Mat3 other);
};

Mat3 RotationMatrix(Vec3 angles);

std::array<double, 6> CalculateAngles(Vec3 pos, Vec3 dir);