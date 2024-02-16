#include "shapes/Plane.h"

using namespace Cannon::Shapes;

Plane::Plane() : Shape(ShapeTypes::PLANE) {}

void Plane::computeWorldNormal(Math::Quaternion* quat) {
    this->worldNormal.set(0, 0, 1);
    quat->vmult(&this->worldNormal, &this->worldNormal);
    this->worldNormalNeedsUpdate = false;
}

void Plane::calculateLocalInertia(float mass, Math::Vec3* target) {
    // do nothing
}

double Plane::volume() {
    return Number.MAX_VALUE; // The plane is infinite...
}

Cannon::Math::Vec3 tempNormal;
void Plane::calculateWorldAABB(
    Math::Vec3* pos,
    Math::Quaternion* quat,
    Math::Vec3* min,
    Math::Vec3* max) {
    tempNormal.set(0, 0, 1);
    quat->vmult(&tempNormal, &tempNormal);
    auto maxVal = Number.MAX_VALUE;
    min->set(-maxVal, -maxVal, -maxVal);
    max->set(maxVal, maxVal, maxVal);

    if (tempNormal.x == 1) {
        max->x = pos->x;
    } else if (tempNormal.x == -1) {
        min->x = pos->x;
    }

    if (tempNormal.y == 1) {
        max->y = pos->y;
    } else if (tempNormal.y == -1) {
        min->y = pos->y;
    }

    if (tempNormal.z == 1) {
        max->z = pos->z;
    } else if (tempNormal.z == -1) {
        min->z = pos->z;
    }
}
