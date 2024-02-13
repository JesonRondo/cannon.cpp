#ifndef Solver_h
#define Solver_h

#include <vector>
#include "equations/Equation.h"

namespace Cannon::World {
    class World;
}

namespace Cannon::Solver {

class Solver
{
private:
    std::vector<Equations::Equation> equations_;

public:
    /**
     * Constraint equation solver base class.
     * @class Solver
     * @constructor
     * @author schteppe / https://github.com/schteppe
     */
    Solver();

    /**
     * Should be implemented in subclasses!
     * @method solve
     * @param  {Number} dt
     * @param  {World} world
     */
    virtual int solve(int dt, World::World* world) = 0;

    /**
     * Add an equation
     * @method addEquation
     * @param {Equation} eq
     */
    void addEquation(Equations::Equation* eq);

    /**
     * Remove an equation
     * @method removeEquation
     * @param {Equation} eq
     */
    void removeEquation(Equations::Equation* eq);

    /**
     * Add all equations
     * @method removeAllEquations
     */
    void removeAllEquations();
};


}

#endif
