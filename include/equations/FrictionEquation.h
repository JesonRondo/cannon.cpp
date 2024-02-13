#ifndef FrictionEquation_h
#define FrictionEquation_h

#include "Equation.h"

namespace Cannon::Objects {
    class Body;
}

namespace Cannon::Equations {

class FrictionEquation : public Equations::Equation {
private:
    Math::Vec3 ri_;
    Math::Vec3 rj_;
    Math::Vec3 t_; // tangent

public:
    /**
     * Constrains the slipping in a contact along a tangent
     * @class FrictionEquation
     * @constructor
     * @author schteppe
     * @param {Body} bodyA
     * @param {Body} bodyB
     * @param {Number} slipForce should be +-F_friction = +-mu * F_normal = +-mu * m * g
     * @extends Equation
     */
    FrictionEquation(Objects::Body* bodyA, Objects::Body* bodyB, double slipForce)
        : Equations::Equation(bodyA, bodyB, -slipForce, slipForce) {};

    double computeB(double h);
}

#endif
