#ifndef Constraint_h
#define Constraint_h

#include <vector>

namespace Cannon::Equations {
    class Equation;
}

namespace Cannon::Objects {
    class Body;
}

namespace Cannon::Constraints {

class Constraint {

public:
    static int idCounter;

    /**
     * ContactMaterial id.
     * @property id
     * @type {number}
     */
    int id;

    /**
     * Equations to be solved in this constraint
     * @property equations
     * @type {Array}
     */
    std::vector<Equations::Equation*> equations;;

    /**
     * @property {Body} bodyA
     */
    Objects::Body* bodyA;

    /**
     * @property {Body} bodyB
     */
    Objects::Body* bodyB;

    /**
     * Set to true if you want the bodies to collide when they are connected.
     * @property collideConnected
     * @type {boolean}
     */
    bool collideConnected = true;

    /**
     * Constraint base class
     * @class Constraint
     * @constructor
     * @param {Body} bodyA
     * @param {Body} bodyB
     */
    Constraint(Objects::Body* bodyA, Objects::Body* bodyB);

    void wakeUpBodies();

    /**
     * Update all the equations with data.
     * @method update
     */
    virtual void update() = 0;

    /**
     * Enables all equations in the constraint.
     * @method enable
     */
    void enable();

    /**
     * Disables all equations in the constraint.
     * @method disable
     */
    void disable();
};

}

#endif
