#include <cmath>

#include "math/Vec3.h"
#include "math/Mat3.h"

using namespace Cannon::Math;

const Vec3 Vec3::ZERO{0.0f, 0.0f, 0.0f};

const Vec3 Vec3::UNIT_X{1.0f, 0.0f, 0.0f};

const Vec3 Vec3::UNIT_Y{0.0f, 1.0f, 0.0f};

const Vec3 Vec3::UNIT_Z{0.0f, 0.0f, 1.0f};

Vec3* Vec3::cross(Vec3* v, Vec3* target) {
    float vx = v->x;
    float vy = v->y;
    float vz = v->z;

    x = this->x,
    y = this->y,
    z = this->z;

    target->x = (y * vz) - (z * vy);
    target->y = (z * vx) - (x * vz);
    target->z = (x * vy) - (y * vx);

    return target;
}

Vec3* Vec3::set(float x, float y, float z) {
    this->x = x;
    this->y = y;
    this->z = z;
    return this;
}

Vec3* Vec3::setZero() {
    this->x = 0;
    this->y = 0;
    this->z = 0;
    return this;
}

Vec3* Vec3::vadd(Vec3* v, Vec3* target) {
    target->x = this->x + v->x;
    target->y = this->y + v->y;
    target->z = this->z + v->z;
    return target;
}

Vec3* Vec3::vsub(Vec3* v, Vec3* target) {
    target->x = this->x - v->x;
    target->y = this->y - v->y;
    target->z = this->z - v->z;
    return target;
}

Mat3 Vec3::crossmat() {
    return Mat3({
        0, -this->z, this->y,
        this->z, 0, -this->x,
        -this->y, this->x, 0
    });
}

float Vec3::normalize() {
    float x = this->x;
    float y = this->y;
    float z = this->z;

    float n = std::sqrt(x * x + y * y + z * z);

    if(n > 0.0){
        float invN = 1 / n;
        this->x *= invN;
        this->y *= invN;
        this->z *= invN;
    } else {
        // Make something up
        this->x = 0;
        this->y = 0;
        this->z = 0;
    }
    return n;
}

Vec3* Vec3::unit(Vec3* target) {
    float x = this->x;
    float y = this->y;
    float z = this->z;

    float ninv = std::sqrt(x * x + y * y + z * z);

    if (ninv > 0.0) {
        ninv = 1.0 / ninv;
        target->x = x * ninv;
        target->y = y * ninv;
        target->z = z * ninv;
    } else {
        target->x = 1;
        target->y = 0;
        target->z = 0;
    }
    return target;
}

float Vec3::length() {
    float x = this->x;
    float y = this->y;
    float z = this->z;

    return std::sqrt(x * x + y * y + z * z);
}

float Vec3::lengthSquared() {
    return this->dot(this);
}

float Vec3::distanceTo(Vec3* p) {
    float x = this->x;
    float y = this->y;
    float z = this->z;

    float px = p->x;
    float py = p->y;
    float pz = p->z;

    return std::sqrt(
        (px - x) * (px - x) +
        (py - y) * (py - y) +
        (pz - z) * (pz - z));
}

float Vec3::distanceSquared(Vec3* p) {
    float x = this->x;
    float y = this->y;
    float z = this->z;

    float px = p->x;
    float py = p->y;
    float pz = p->z;

    return (px - x) * (px - x) +
           (py - y) * (py - y) +
           (pz - z) * (pz - z);
}

Vec3* Vec3::vmul(Vec3* vector, Vec3* target) {
    target->x = this->x * vector->x;
    target->y = this->y * vector->y;
    target->z = this->z * vector->z;
    return target;
}

Vec3* Vec3::scale(float scalar, Vec3* target) {
    target->x = this->x * scalar;
    target->y = this->y * scalar;
    target->z = this->z * scalar;
    return target;
}

Vec3* Vec3::addScaledVector(float scalar, Vec3* vector, Vec3* target) {
    target->x = this->x + vector->x * scalar;
    target->y = this->y + vector->y * scalar;
    target->z = this->z + vector->z * scalar;
    return target;
}

float Vec3::dot(Vec3* v) {
    return this->x * v->x + this->y * v->y + this->z * v->z;
}

bool Vec3::isZero() {
    return this->x == 0 && this->y == 0 && this->z == 0;
}

Vec3* Vec3::negate(Vec3* target) {
    target->x = -this->x;
    target->y = -this->y;
    target->z = -this->z;
    return target;
}

Vec3 innerTangentsSearchVec3;
Vec3 innerTangentsSearchVec3_2;

void Vec3::tangents(Vec3* t1, Vec3* t2) {
    float norm = this->length();

    if (norm > 0.0) {
        Vec3* n = &innerTangentsSearchVec3;
        float inorm = 1 / norm;
        n->set(
            this->x * inorm,
            this->y * inorm,
            this-> z * inorm
        );
        Vec3* randVec = &innerTangentsSearchVec3_2;
        if (std::abs(n->x) < 0.9){
            randVec->set(1, 0, 0);
            n->cross(randVec, t1);
        } else {
            randVec->set(0, 1, 0);
            n->cross(randVec, t1);
        }
        n->cross(t1, t2);
    } else {
        // The normal length is zero, make something up
        t1->set(1.0, 0.0, 0.0);
        t2->set(0.0, 1.0, 0.0);
    }
}

std::string Vec3::toString() {
    return std::to_string(x) + "," + std::to_string(y) + "," + std::to_string(z);
}

std::array<float, 3> Vec3::toArray() {
    return {x, y, z};
}

Vec3* Vec3::copy(Vec3* source) {
    this->x = source->x;
    this->y = source->y;
    this->z = source->z;
    return this;
}

Vec3* Vec3::lerp(Vec3* v, float t, Vec3* target) {
    float x = this->x;
    float y = this->y;
    float z = this->z;

    target->x = x + (v->x - x) * t;
    target->y = y + (v->y - y) * t;
    target->z = z + (v->z - z) * t;

    return target;
}

bool Vec3::almostEquals(Vec3* v, float precision) {
    if (std::abs(this->x - v->x) > precision ||
        std::abs(this->y - v->y) > precision ||
        std::abs(this->z - v->z) > precision) {
        return false;
    };
    return true;
}

bool Vec3::almostZero(float precision) {
    if (std::abs(this->x) > precision ||
        std::abs(this->y) > precision ||
        std::abs(this->z) > precision) {
        return false;
    };
    return true;
}

Vec3 innerAntipNegSearchVec3;
bool Vec3::isAntiparallelTo(Vec3* v, float precision) {
    Vec3* an = &innerAntipNegSearchVec3;
    this->negate(an);
    return an->almostEquals(v, precision);
}

Vec3 Vec3::clone() {
    return Vec3(this->x, this->y, this->z);
}
