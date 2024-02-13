#ifndef Particle_h
#define Particle_h

#include "shapes/Shape.h"
#include "math/Quaternion.h"

namespace Cannon::Shapes {

class Particle : public Shape {
public:
    /**
     * Particle shape.
     * @class Particle
     * @constructor
     * @author schteppe
     * @extends Shape
     */
    Particle();

    /**
     * @method calculateLocalInertia
     * @param  {Number} mass
     * @param  {Vec3} target
     * @return {Vec3}
     */
    Math::Vec3* calculateLocalInertia(float mass, Math::Vec3* target);

    double volume();

    void calculateWorldAABB(
        Math::Vec3 pos,
        Math::Quaternion quat,
        Math::Vec3* min,
        Math::Vec3* max);

    void updateBoundingSphereRadius();
};

}

#endif
