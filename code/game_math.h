#pragma once
#include "types.h"

struct V3
{
    f32 x;
    f32 y;
    f32 z;

    V3 operator+(V3 b);
    V3 operator-(V3 b);
    V3 operator-();
    V3 operator*(f32 t);
    void operator+=(V3 b);
    void operator-=(V3 b);
};

inline V3 v3(f32 x, f32 y, f32 z)
{
    return {x, y, z};
}

inline f32 SquareLength(V3 a)
{
    return a.x * a.x + a.y * a.y + a.z * a.z;
}

inline f32 Length(V3 a)
{
    return sqrt(SquareLength(a));
}

inline V3 Normalize(V3 a)
{
    f32 length = sqrt(a.x * a.x + a.y * a.y + a.z * a.z);
    return v3(a.x / length, a.y / length, a.z / length);
}

inline V3 Cross(V3 a, V3 b)
{
    V3 result = {};
    result.x = a.y * b.z - a.z * b.y;
    result.y = a.z * b.x - a.x * b.z;
    result.z = a.x * b.y - a.y * b.x;
    return result;
}

f32 Dot(V3 a, V3 b)
{
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

struct Quat
{
    f32 w;
    f32 x;
    f32 y;
    f32 z;
};

Quat RotationAroundAxis(V3 axis, f32 radians);

// IMPORTANT: Column major!
// 0 3 6
// 1 4 7
// 2 5 8

// NOTE: If a matrix has a determinant of 1 (meaning "it has no scale component to it"), then its inverse is equal to its transpose
struct Mat3
{
    f32 v[9];

    V3 operator*(V3 v);
    Mat3 operator *(f32 t);
    Mat3 operator *(Mat3 m);
};

Mat3 FromRotation(Quat q);
Mat3 Scale(f32 scale);
Mat3 Transpose(Mat3 m);
Mat3 Inverse(Mat3 m);
