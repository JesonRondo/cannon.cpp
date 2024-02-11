#ifndef Material_h
#define Material_h

#include <string>

namespace Cannon::Material {

class Material {
private:

public:
    static int idCounter;

    /**
     * material id.
     * @property id
     * @type {number}
     */
    int id;

    /**
     * @property name
     * @type {String}
     */
    std::string name;

    /**
     * Friction for this material. If non-negative, it will be used instead of the friction given by ContactMaterials. If there's no matching ContactMaterial, the value from .defaultContactMaterial in the World will be used.
     * @property {number} friction
     */
    float friction;

    /**
     * Restitution for this material. If non-negative, it will be used instead of the restitution given by ContactMaterials. If there's no matching ContactMaterial, the value from .defaultContactMaterial in the World will be used.
     * @property {number} restitution
     */
    float restitution;

    /**
     * Defines a physics material.
     * @class Material
     * @constructor
     */
    Material();
    
    /**
     * Defines a physics material.
     * @class Material
     * @constructor
     * @param {string} name
     */
    Material(std::string name);

};

}

#endif
