#ifndef GSSolver_h
#define GSSolver_h

#include "solver/Solver.h"

namespace Cannon::Solver {

class GSSolver : public Solver::Solver {
private:

public:
    /**
     * The number of solver iterations determines quality of the constraints in the world. The more iterations, the more correct simulation. More iterations need more computations though. If you have a large gravity force in your world, you will need more iterations.
     * @property iterations
     * @type {Number}
     * @todo write more about solver and iterations in the wiki
     */
    int iterations = 10;

    /**
     * When tolerance is reached, the system is assumed to be converged.
     * @property tolerance
     * @type {Number}
     */
    double tolerance = 1e-7;

    /**
     * Constraint equation Gauss-Seidel solver.
     * @class GSSolver
     * @constructor
     * @todo The spook parameters should be specified for each constraint, not globally.
     * @author schteppe / https://github.com/schteppe
     * @see https://www8.cs.umu.se/kurser/5DV058/VT09/lectures/spooknotes.pdf
     * @extends Solver
     */
    GSSolver();

    /**
     * @method solve
     * @param  {Number} dt
     * @param  {World} world
     */
    int solve(int dt, World::World* world);
};


}

#endif
