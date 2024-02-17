#include "math/Transform.h"

using namespace Cannon::Math;

Quaternion tmpQuat;
Vec3* Transform::pointToLocalFrame(Vec3* position, Quaternion* quaternion, Vec3* worldPoint, Vec3* result) {
    worldPoint->vsub(position, result);
    quaternion->conjugate(&tmpQuat);
    tmpQuat.vmult(result, result);
    return result;
}

Vec3* Transform::pointToWorldFrame(Vec3* position, Quaternion* quaternion, Vec3* localPoint, Vec3* result) {
    quaternion->vmult(localPoint, result);
    result->vadd(position, result);
    return result;
}

Vec3* Transform::vectorToWorldFrame(Quaternion* quaternion, Vec3* localVector, Vec3* result) {
    quaternion->vmult(localVector, result);
    return result;
}

Vec3* Transform::vectorToLocalFrame(Vec3* position, Quaternion* quaternion, Vec3* worldVector, Vec3* result) {
    quaternion->w *= -1;
    quaternion->vmult(worldVector, result);
    quaternion->w *= -1;
    return result;
}

Vec3* Transform::pointToLocal(Vec3* worldPoint, Vec3* result) {
    return Transform::pointToLocalFrame(this->position_, this->quaternion_, worldPoint, result);
}

Vec3* Transform::pointToWorld(Vec3* localPoint, Vec3* result) {
    return Transform::pointToWorldFrame(this->position_, this->quaternion_, localPoint, result);
}

Vec3* Transform::vectorToWorldFrame(Vec3* localVector, Vec3* result) {
    return Transform::vectorToWorldFrame(this->quaternion_, localVector, result);
}
