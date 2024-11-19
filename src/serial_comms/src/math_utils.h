#include <array>
#include <math.h>
#include <iostream>

namespace armdim {
    const double CLAW_LENGTH = 43.5;
    const double BASE_HEIGHT = 43.2;

    const double A1 = 23;       // turret side offset
    const double A2 = 100;      // first arm length
    const double D4 = 105.05;   // second arm length
    const double D2 = 48.25;    // turret front offset

    const double REACH_MULTIPLIER = 1;     // -1 for balance
    const double ELBOW_UP_MULTIPLIER = 1;  // -1 for elbow down
}

class Vec3 {
    public:
        double x{};
        double y{};
        double z{};

        Vec3(double x, double y, double z);
        Vec3();

        Vec3 operator+(Vec3 other);
        Vec3 operator-(Vec3 other);
        Vec3 operator*(double scalar);
        Vec3 operator-();
        double dot(Vec3 other);

        // https://learn.microsoft.com/en-us/cpp/standard-library/overloading-the-output-operator-for-your-own-classes?view=msvc-170
        friend std::ostream& operator<<(std::ostream & os, const Vec3 & vec);
};

class Mat3 {
    public:
        std::array<std::array<double, 3>, 3> data{};

        Mat3(std::array<std::array<double, 3>, 3> data);
        Mat3(double fill);
        Mat3();
        
        std::array<double, 3> & operator[](int i);
        const std::array<double, 3> & operator[](int i) const;

        Vec3 getColumn(int i);
        Vec3 getRow(int i);
        Mat3 mul(Mat3 other);

        friend std::ostream& operator<<(std::ostream & os, const Mat3 & mat);
};

class Pose3 {
    public:
        Vec3 pos;
        Mat3 dir;

        Pose3(Vec3 pos, Mat3 dir);
        Pose3();
        
        friend std::ostream& operator<<(std::ostream & os, const Pose3 & pose);
};

double sq(double x);
Mat3 RotationMatrix(Vec3 angles);