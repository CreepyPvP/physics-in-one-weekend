#include "game_math.h"

V3 V3::operator+(V3 b)
{
    return v3(x + b.x, y + b.y, z + b.z);
}

V3 V3::operator-(V3 b)
{
    return v3(x - b.x, y - b.y, z - b.z);
}

V3 V3::operator-()
{
    return v3(-x, -y, -z);
}

void V3::operator+=(V3 b)
{
    x += b.x;
    y += b.y;
    z += b.z;
}

void V3::operator-=(V3 b)
{
    x -= b.x;
    y -= b.y;
    z -= b.z;
}

V3 V3::operator*(f32 t)
{
    return v3(x * t, y * t, z * t);
}

Mat3 Mat3::operator*(f32 t)
{
    Mat3 result;
    for (u32 i = 0; i < 9; ++i)
    {
        result.v[i] = v[i] * t;
    }
    return result;
}

Mat3 Mat3::operator*(Mat3 m)
{
    return {
        v[0] * m.v[0] + v[3] + m.v[1] + v[6] * m.v[2], v[1] * m.v[0] + v[4] * m.v[1] + v[7] * m.v[2], v[2] * m.v[0] + v[5] * m.v[1] + m.v[8] * m.v[2],
        v[0] * m.v[3] + v[3] + m.v[4] + v[6] * m.v[5], v[1] * m.v[3] + v[4] * m.v[4] + v[7] * m.v[5], v[2] * m.v[3] + v[5] * m.v[4] + m.v[8] * m.v[5],
        v[0] * m.v[6] + v[3] + m.v[7] + v[6] * m.v[8], v[1] * m.v[6] + v[4] * m.v[7] + v[7] * m.v[8], v[2] * m.v[6] + v[5] * m.v[7] + m.v[8] * m.v[8],
    };
}

V3 Mat3::operator*(V3 b)
{
    return v3(v[0] * b.x + v[3] * b.y + v[6] * b.z,
              v[1] * b.x + v[4] * b.y + v[7] * b.z,
              v[2] * b.x + v[5] * b.y + v[8] * b.z);
}

Mat3 Transpose(Mat3 m)
{
    return {
        m.v[0], m.v[3], m.v[6],
        m.v[1], m.v[4], m.v[7],
        m.v[2], m.v[5], m.v[8],
    };
}

Mat3 Scale(f32 scale)
{
    return {
        scale, 0, 0,
        0, scale, 0,
        0, 0, scale,
    };
}

Mat3 Inverse(Mat3 m)
{
    Mat3 result = {
        m.v[4] * m.v[8] - m.v[7] * m.v[5], -(m.v[1] * m.v[8] - m.v[7] * m.v[2]), m.v[1] * m.v[5] - m.v[4] * m.v[2],
        -(m.v[3] * m.v[8] - m.v[6] * m.v[5]), m.v[0] * m.v[8] - m.v[6] * m.v[2], -(m.v[0] * m.v[5] - m.v[3] * m.v[2]),
        m.v[3] * m.v[7] - m.v[6] * m.v[4], -(m.v[0] * m.v[7] - m.v[6] * m.v[1]), m.v[0] * m.v[4] - m.v[3] * m.v[1],
    };

    f32 det = m.v[0] * result.v[0] + m.v[3] * result.v[1] + m.v[6] * result.v[2];

    for (u32 i = 0; i < 9; ++i)
    {
        result.v[i] *= 1 / det;
    }

    return result;
}

Mat3 FromRotation(Quat q)
{
    return {
        1 - 2 * (q.y * q.y + q.z * q.z), 2 * (q.x * q.y + q.z * q.w), 2 * (q.x * q.z - q.y * q.w),
        2 * (q.x * q.y - q.z * q.w), 1 - 2 * (q.x * q.x + q.z * q.z), 2 * (q.y * q.z + q.x * q.w),
        2 * (q.x * q.z + q.y * q.w), 2 * (q.y * q.z - q.x * q.w), 1 - 2 * (q.x * q.x + q.y * q.y),
    };
}

Quat RotationAroundAxis(V3 axis, f32 radians)
{
    Quat result;

    f32 half_angle = 0.5 * radians;
    result.w = cos(half_angle);
    
    f32 half_sine = sin(half_angle);
    axis = Normalize(axis);
    result.x = axis.x * half_sine;
    result.y = axis.y * half_sine;
    result.z = axis.z * half_sine;

    return result;
}
