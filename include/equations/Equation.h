#ifndef Equation_h
#define Equation_h

#include "math/JacobianElement.h"

namespace Cannon::Objects {
    class Body;
}

namespace Cannon::Equations {
    
class Equation {
public:
    static int idCounter;

    /**
     * ContactMaterial id.
     * @property id
     * @type {number}
     */
    int id;

    /**
     * @property {number} minForce
     */
    double minForce = -1e6;

    /**
     * @property {number} maxForce
     */
    double maxForce = 1e6;

    /**
     * @property bi
     * @type {Body}
     */
    Objects::Body* bi;

    /**
     * @property bj
     * @type {Body}
     */
    Objects::Body* bj;

    /**
     * SPOOK parameter
     * @property {number} a
     */
    double a = 0.0;

    /**
     * SPOOK parameter
     * @property {number} b
     */
    double b = 0.0;

    /**
     * SPOOK parameter
     * @property {number} eps
     */
    double eps = 0.0;

    /**
     * @property {JacobianElement} jacobianElementA
     */
    Math::JacobianElement jacobianElementA;

    /**
     * @property {JacobianElement} jacobianElementB
     */
    Math::JacobianElement jacobianElementB;

    /**
     * @property {boolean} enabled
     * @default true
     */
    bool enabled = true;

    /**
     * A number, proportional to the force added to the bodies.
     * @property {number} multiplier
     * @readonly
     */
    int multiplier = 0;

    /**
     * Equation base class
     * @class Equation
     * @constructor
     * @author schteppe
     * @param {Body} bi
     * @param {Body} bj
     */
    Equation(Objects::Body* bi, Objects::Body* bj);

    /**
     * Equation base class
     * @class Equation
     * @constructor
     * @author schteppe
     * @param {Body} bi
     * @param {Body} bj
     * @param {Number} minForce Minimum (read: negative max) force to be applied by the constraint.
     * @param {Number} maxForce Maximum (read: positive max) force to be applied by the constraint.
     */
    Equation(Objects::Body* bi, Objects::Body* bj, double minForce, double maxForce);

    /**
     * Recalculates a,b,eps.
     * @method setSpookParams
     */
    void setSpookParams(double stiffness, double relaxation, float timeStep);

    /**
     * Computes the RHS of the SPOOK equation
     * @method computeB
     * @return {Number}
     */
    double computeB(double a, double b, double h);

    /**
     * Computes G*q, where q are the generalized body coordinates
     * @method computeGq
     * @return {Number}
     */
    double computeGq();

    /**
     * Computes G*W, where W are the body velocities
     * @method computeGW
     * @return {Number}
     */
    double computeGW();

    /**
     * Computes G*Wlambda, where W are the body velocities
     * @method computeGWlambda
     * @return {Number}
     */
    double computeGWlambda();

    /**
     * Computes G*inv(M)*f, where M is the mass matrix with diagonal blocks for each body, and f are the forces on the bodies.
     * @method computeGiMf
     * @return {Number}
     */
    double computeGiMf();

    /**
     * Computes G*inv(M)*G'
     * @method computeGiMGt
     * @return {Number}
     */
    double computeGiMGt();

    /**
     * Add constraint velocity to the bodies.
     * @method addToWlambda
     * @param {Number} deltalambda
     */
    void addToWlambda(double deltalambda);

    /**
     * Compute the denominator part of the SPOOK equation: C = G*inv(M)*G' + eps
     * @method computeInvC
     * @param  {Number} eps
     * @return {Number}
     */
   double computeC();
};

}

#endif
