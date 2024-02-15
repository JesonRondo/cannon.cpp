#include <cmath>

#include "math/Vec3.h"

using namespace Cannon::Math;

/**
 * @static
 * @property {Vec3} ZERO
 */
const Vec3 Vec3::ZERO{0.0f, 0.0f, 0.0f};

/**
 * @static
 * @property {Vec3} UNIT_X
 */
const Vec3 Vec3::UNIT_X{1.0f, 0.0f, 0.0f};

/**
 * @static
 * @property {Vec3} UNIT_Y
 */
const Vec3 Vec3::UNIT_Y{0.0f, 1.0f, 0.0f};

/**
 * @static
 * @property {Vec3} UNIT_Z
 */
const Vec3 Vec3::UNIT_Z{0.0f, 0.0f, 1.0f};

std::string Vec3::toString() {
    return std::to_string(x) + "," + std::to_string(y) + "," + std::to_string(z);
}

bool Vec3::almostEquals(Vec3* v, float precision) {
    if (std::abs(this->x - v->x) > precision ||
        std::abs(this->y - v->y) > precision ||
        std::abs(this->z - v->z) > precision) {
        return false;
    };
    return true;
}
