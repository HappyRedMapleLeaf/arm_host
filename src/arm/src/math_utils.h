#include <array>
#include <math.h>
#include <iostream>

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
        Vec3 cross(Vec3 other);

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
        void setColumn(int i, Vec3 vec);
        void setRow(int i, Vec3 vec);
        Mat3 mul(Mat3 other);
        Mat3 transpose();

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