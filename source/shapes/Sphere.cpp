#include "shapes/Sphere.h"

#include <stdexcept>
#include <cmath>

using namespace Cannon::Shapes;

Sphere::Sphere() : Shape(ShapeTypes::SPHERE) {
    Sphere(1);
}

Sphere::Sphere(float radius) : Shape(ShapeTypes::SPHERE) {
    this->radius = radius;

    if (radius < 0) {
        throw std::runtime_error("The sphere radius cannot be negative.");
    }

    this->updateBoundingSphereRadius();
}

void Sphere::calculateLocalInertia(float mass, Math::Vec3* target) {
    float I = 2.0 / 5.0 * mass * this->radius * this->radius;
    target->set(I, I, I);
}

double Sphere::volume() {
    return 4.0 / 3.0 * M_PI * this->radius;
}

void Sphere::updateBoundingSphereRadius() {
    this->boundingSphereRadius = this->radius;
}

void Sphere::calculateWorldAABB(
    Math::Vec3* pos,
    Math::Quaternion* quat,
    Math::Vec3* min,
    Math::Vec3* max) {
    float r = this->radius;

    min->set(pos->x - r, pos->y - r, pos->z - r);
    max->set(pos->x + r, pos->y + r, pos->z + r);     
}
