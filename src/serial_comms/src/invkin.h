#include <array>

const double CLAW_LENGTH = 43.5;
const double BASE_HEIGHT = 43.2;

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
};

class Mat3 {
    public:
        std::array<std::array<double, 3>, 3> data{};

        Mat3(std::array<std::array<double, 3>, 3> data);
        Mat3();
        
        Vec3 getColumn(int i);
};

Mat3 RotationMatrix(Vec3 angles);

std::array<double, 6> CalculateAngles(Vec3 pos, Vec3 dir);