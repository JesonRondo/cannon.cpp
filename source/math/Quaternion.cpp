#include <cmath>

#include "math/Quaternion.h"
#include "math/Vec3.h"

using namespace Cannon::Math;

Quaternion* Quaternion::setFromEuler(float x, float y, float z) {
    return setFromEuler(x, y, z, EulerOrder::XYZ);
};

Quaternion* Quaternion::setFromEuler(float x, float y, float z, EulerOrder order) {
    float c1 = std::cos(x / 2);
    float c2 = std::cos(y / 2);
    float c3 = std::cos(z / 2);
    float s1 = std::sin(x / 2);
    float s2 = std::sin(y / 2);
    float s3 = std::sin(z / 2);

    if (order == EulerOrder::XYZ) {
        this->x = s1 * c2 * c3 + c1 * s2 * s3;
        this->y = c1 * s2 * c3 - s1 * c2 * s3;
        this->z = c1 * c2 * s3 + s1 * s2 * c3;
        this->w = c1 * c2 * c3 - s1 * s2 * s3;
    } else if (order == EulerOrder::YXZ) {
        this->x = s1 * c2 * c3 + c1 * s2 * s3;
        this->y = c1 * s2 * c3 - s1 * c2 * s3;
        this->z = c1 * c2 * s3 - s1 * s2 * c3;
        this->w = c1 * c2 * c3 + s1 * s2 * s3;
    } else if ( order == EulerOrder::ZXY) {
        this->x = s1 * c2 * c3 - c1 * s2 * s3;
        this->y = c1 * s2 * c3 + s1 * c2 * s3;
        this->z = c1 * c2 * s3 + s1 * s2 * c3;
        this->w = c1 * c2 * c3 - s1 * s2 * s3;
    } else if ( order == EulerOrder::ZYX) {
        this->x = s1 * c2 * c3 - c1 * s2 * s3;
        this->y = c1 * s2 * c3 + s1 * c2 * s3;
        this->z = c1 * c2 * s3 - s1 * s2 * c3;
        this->w = c1 * c2 * c3 + s1 * s2 * s3;
    } else if ( order == EulerOrder::YZX) {
        this->x = s1 * c2 * c3 + c1 * s2 * s3;
        this->y = c1 * s2 * c3 + s1 * c2 * s3;
        this->z = c1 * c2 * s3 - s1 * s2 * c3;
        this->w = c1 * c2 * c3 - s1 * s2 * s3;
    } else if ( order == EulerOrder::XZY) {
        this->x = s1 * c2 * c3 - c1 * s2 * s3;
        this->y = c1 * s2 * c3 - s1 * c2 * s3;
        this->z = c1 * c2 * s3 + s1 * s2 * c3;
        this->w = c1 * c2 * c3 + s1 * s2 * s3;
    }

    return this;
};

Vec3* Quaternion::vmult(Vec3* v, Vec3* target) {
    float x = v->x;
    float y = v->y;
    float z = v->z;

    float qx = this->x;
    float qy = this->y;
    float qz = this->z;
    float qw = this->w;

    // q*v
    float ix =  qw * x + qy * z - qz * y;
    float iy =  qw * y + qz * x - qx * z;
    float iz =  qw * z + qx * y - qy * x;
    float iw = -qx * x - qy * y - qz * z;

    target->x = ix * qw + iw * -qx + iy * -qz - iz * -qy;
    target->y = iy * qw + iw * -qy + iz * -qx - ix * -qz;
    target->z = iz * qw + iw * -qz + ix * -qy - iy * -qx;

    return target;
}

std::string Quaternion::toString() {
    return std::to_string(x) + "," + std::to_string(y) + "," + std::to_string(z) + "," + std::to_string(w);
}
