#include "utils/Vec3Pool.h"

using namespace Cannon::Utils;

Vec3Pool::Vec3Pool() : Pool<Math::Vec3>() {}

Vec3Pool::~Vec3Pool() {}

Cannon::Math::Vec3 Vec3Pool::constructObject() {
    return Math::Vec3();
}
