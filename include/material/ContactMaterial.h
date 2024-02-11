#ifndef ContactMaterial_h
#define ContactMaterial_h

#include <array>
#include "material/Material.h"

namespace Cannon::Material {

class ContactMaterial {
public:
    static int idCounter;

    /**
     * ContactMaterial id.
     * @property id
     * @type {number}
     */
    int id;

    /**
     * Participating materials
     * @property {Array} materials
     * @todo  Should be .materialA and .materialB instead
     */
    std::array<Material*, 2> materials;

    /**
     * Friction coefficient
     * @property {Number} friction
     */
    float friction = 0.3;

    /**
     * Restitution coefficient
     * @property {Number} restitution
     */
    float restitution = 0.3;

    /**
     * Stiffness of the produced contact equations
     * @property {Number} contactEquationStiffness
     */
    double contactEquationStiffness = 1e7;

    /**
     * Relaxation time of the produced contact equations
     * @property {Number} contactEquationRelaxation
     */
    float contactEquationRelaxation = 3;

    /**
     * Stiffness of the produced friction equations
     * @property {Number} frictionEquationStiffness
     */
    double frictionEquationStiffness = 1e7;

    /**
     * Relaxation time of the produced friction equations
     * @property {Number} frictionEquationRelaxation
     */
    float frictionEquationRelaxation = 3;

    /**
     * Defines what happens when two materials meet.
     * @class ContactMaterial
     * @constructor
     * @param {Material} m1
     * @param {Material} m2
     */
    ContactMaterial(Material* m1, Material* m2): materials({m1, m2}){ id = ContactMaterial::idCounter++; };
};

int ContactMaterial::idCounter = 0;

}

#endif
