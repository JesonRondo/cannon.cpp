#ifndef ContactEquation_h
#define ContactEquation_h

#include "Equation.h"

namespace Cannon::Objects {
    class Body;
}

namespace Cannon::Shapes {
    class Shape;
}

namespace Cannon::Equations {

class ContactEquation : public Equations::Equation {
private:

public:
    /**
     * @property si
     * @type {Shape}
     */
    Shapes::Shape* si;

    /**
     * @property sj
     * @type {Shape}
     */
    Shapes::Shape* sj;
    
    /**
     * @property restitution
     * @type {Number}
     */
    double restitution = 0.0; // "bounciness": u1 = -e*u0

    /**
     * World-oriented vector that goes from the center of bi to the contact point.
     * @property {Vec3} ri
     */
    Math::Vec3 ri;

    /**
     * World-oriented vector that starts in body j position and goes to the contact point.
     * @property {Vec3} rj
     */
    Math::Vec3 rj;

    /**
     * Contact normal, pointing out of body i.
     * @property {Vec3} ni
     */
    Math::Vec3 ni;

    /**
     * Contact/non-penetration constraint equation
     * @class ContactEquation
     * @constructor
     * @author schteppe
     * @param {Body} bodyA
     * @param {Body} bodyB
     * @extends Equation
     */
    ContactEquation(Objects::Body* bodyA, Objects::Body* bodyB, double maxForce)
        : Equations::Equation(bodyA, bodyB, 0, maxForce) {};

    double computeB(double h);

    double getImpactVelocityAlongNormal();
};

}

#endif
