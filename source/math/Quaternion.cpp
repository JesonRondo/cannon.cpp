#include <cmath>
#include <stdexcept>

#include "math/Quaternion.h"
#include "math/Vec3.h"

using namespace Cannon::Math;

Quaternion* Quaternion::set(float x, float y, float z, float w) {
    this->x = x;
    this->y = y;
    this->z = z;
    this->w = w;

    return this;
}

std::string Quaternion::toString() {
    return std::to_string(x) + "," + std::to_string(y) + "," + std::to_string(z) + "," + std::to_string(w);
}

std::array<float, 4> Quaternion::toArray() {
    return {x, y, z, w};
}

Quaternion* Quaternion::setFromAxisAngle(Vec3* axis, float angle) {
    float halfAngle = angle / 2;
    float s = std::sin(halfAngle);

    this->x = axis->x * s;
    this->y = axis->y * s;
    this->z = axis->z * s;
    this->w = std::cos(halfAngle);

    return this;
}

float Quaternion::toAxisAngle(Vec3* targetAxis) {
    this->normalize();
    float angle = 2 * std::acos(this->w);
    float s = std::sqrt(1 - this->w * this->w);
    if (s < 0.001) {
        targetAxis->x = this->x;
        targetAxis->y = this->y;
        targetAxis->z = this->z;
    } else {
        targetAxis->x = this->x / s;
        targetAxis->y = this->y / s;
        targetAxis->z = this->z / s;
    }
    return angle;
}

Vec3 innerSFVSearchVec3_1;
Vec3 innerSFVSearchVec3_2;
Quaternion* Quaternion::setFromVectors(Vec3* u, Vec3* v) {
    if (u->isAntiparallelTo(v, 0.00001f)) {
        Vec3* t1 = &innerSFVSearchVec3_1;
        Vec3* t2 = &innerSFVSearchVec3_2;
        
        u->tangents(t1, t2);
        this->setFromAxisAngle(t1, M_PI);
    } else {
        Vec3* a = &innerSFVSearchVec3_1;
        
        u->cross(v, a);
        this->x = a->x;
        this->y = a->y;
        this->z = a->z;
        this->w = std::sqrt(u->lengthSquared() * v->lengthSquared()) + u->dot(v);
        this->normalize();
    }
    return this;
}

Quaternion* Quaternion::mult(Quaternion* q, Quaternion* target) {
    float ax = this->x;
    float ay = this->y;
    float az = this->z;
    float aw = this->w;

    float bx = q->x;
    float by = q->y;
    float bz = q->z;
    float bw = q->w;

    target->x = ax * bw + aw * bx + ay * bz - az * by;
    target->y = ay * bw + aw * by + az * bx - ax * bz;
    target->z = az * bw + aw * bz + ax * by - ay * bx;
    target->w = aw * bw - ax * bx - ay * by - az * bz;

    return target;
}

Quaternion* Quaternion::inverse(Quaternion* target) {
    float x = this->x;
    float y = this->y;
    float z = this->z;
    float w = this->w;

    this->conjugate(target);

    float inorm2 = 1 / (x * x + y * y + z * z + w * w);
    target->x *= inorm2;
    target->y *= inorm2;
    target->z *= inorm2;
    target->w *= inorm2;

    return target;
}

Quaternion* Quaternion::conjugate(Quaternion* target) {
    target->x = -this->x;
    target->y = -this->y;
    target->z = -this->z;
    target->w = this->w;

    return target;
}

Quaternion* Quaternion::normalize() {
    float l = std::sqrt(this->x * this->x + this->y * this->y + this->z * this->z + this->w * this->w);
    if (l == 0) {
        this->x = 0;
        this->y = 0;
        this->z = 0;
        this->w = 0;
    } else {
        l = 1 / l;
        this->x *= l;
        this->y *= l;
        this->z *= l;
        this->w *= l;
    }
    return this;
}

Quaternion* Quaternion::normalizeFast() {
    float f = (3.0f - (this->x * this->x + this->y * this->y + this->z * this->z + this->w * this->w)) / 2.0f;
    if (f == 0) {
        this->x = 0;
        this->y = 0;
        this->z = 0;
        this->w = 0;
    } else {
        this->x *= f;
        this->y *= f;
        this->z *= f;
        this->w *= f;
    }
    return this;
}

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

Quaternion* Quaternion::copy(Quaternion* source) {
    this->x = source->x;
    this->y = source->y;
    this->z = source->z;
    this->w = source->w;

    return this;
}

void Quaternion::toEuler(Vec3* target) {
    this->toEuler(target, EulerOrder::YZX);
}

void Quaternion::toEuler(Vec3* target, EulerOrder order) {
    float x = this->x;
    float y = this->y;
    float z = this->z;
    float w = this->w;

    float heading, attitude, bank;
    float test;

    switch (order) {
        case EulerOrder::YZX:
            test = x * y + z * w;
            if (test > 0.499) {
                heading = 2 * std::atan2(x, w);
                attitude = M_PI / 2;
                bank = 0;
            } else if (test < -0.499) {
                heading = -2 * std::atan2(x, w);
                attitude = -M_PI / 2;
                bank = 0;
            } else {
                float sqx = x * x;
                float sqy = y * y;
                float sqz = z * z;
                heading = std::atan2(2 * y * w - 2 * x * z, 1 - 2 * sqy - 2 * sqz);
                attitude = std::asin(2 * test);
                bank = std::atan2(2 * x * w - 2 * y * z, 1 - 2 * sqx - 2 * sqz);
            }
            break;
        
        default:
            throw std::runtime_error("Euler order not supported yet");
            break;
    }

    target->x = bank;
    target->y = heading;
    target->z = attitude;
}

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

Quaternion Quaternion::clone() {
    return Quaternion(this->x, this->y, this->z, this->w);
}

Quaternion* Quaternion::slerp(Quaternion* toQuat, float t, Quaternion* target) {
    float ax = this->x;
    float ay = this->y;
    float az = this->z;
    float aw = this->w;
    float bx = toQuat->x;
    float by = toQuat->y;
    float bz = toQuat->z;
    float bw = toQuat->w;

    float omega, cosom, sinom, scale0, scale1;

    // calc cosine
    cosom = ax * bx + ay * by + az * bz + aw * bw;

    // adjust signs (if necessary)
    if (cosom < 0.0) {
        cosom = -cosom;
        bx = -bx;
        by = -by;
        bz = -bz;
        bw = -bw;
    }

    // calculate coefficients
    if ((1.0 - cosom) > 0.000001) {
        // standard case (slerp)
        omega = std::acos(cosom);
        sinom = std::sin(omega);
        scale0 = std::sin((1.0 - t) * omega) / sinom;
        scale1 = std::sin(t * omega) / sinom;
    } else {
        // "from" and "to" quaternions are very close
        //  ... so we can do a linear interpolation
        scale0 = 1.0 - t;
        scale1 = t;
    }

    // calculate final values
    target->x = scale0 * ax + scale1 * bx;
    target->y = scale0 * ay + scale1 * by;
    target->z = scale0 * az + scale1 * bz;
    target->w = scale0 * aw + scale1 * bw;

    return target;
}

Quaternion* Quaternion::integrate(Vec3* angularVelocity, float dt, Vec3* angularFactor, Quaternion* target) {
    float ax = angularVelocity->x * angularFactor->x;
    float ay = angularVelocity->y * angularFactor->y;
    float az = angularVelocity->z * angularFactor->z;
    float bx = this->x;
    float by = this->y;
    float bz = this->z;
    float bw = this->w;

    float half_dt = dt * 0.5;

    target->x += half_dt * (ax * bw + ay * bz - az * by);
    target->y += half_dt * (ay * bw + az * bx - ax * bz);
    target->z += half_dt * (az * bw + ax * by - ay * bx);
    target->w += half_dt * (-ax * bx - ay * by - az * bz);

    return target;
}
