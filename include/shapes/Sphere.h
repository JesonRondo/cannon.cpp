#ifndef Sphere_h
#define Sphere_h

#include "shapes/Shape.h"
#include "math/Quaternion.h"

namespace Cannon::Shapes {

class Sphere : public Shape {
public:
    /**
     * @property {Number} radius
     */
    float radius;

    /**
     * Spherical shape
     * @class Sphere
     * @constructor
     * @extends Shape
     * @param {Number} radius The radius of the sphere, a non-negative number.
     * @author schteppe / http://github.com/schteppe
     */
    Sphere();
    Sphere(float radius);

    void calculateLocalInertia(float mass, Math::Vec3* target);

    double volume();

    void updateBoundingSphereRadius();

    void calculateWorldAABB(
        Math::Vec3* pos,
        Math::Quaternion* quat,
        Math::Vec3* min,
        Math::Vec3* max);
};

}

#endif
