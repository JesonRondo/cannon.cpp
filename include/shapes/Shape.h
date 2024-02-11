#ifndef Shape_h
#define Shape_h

#include "math/Vec3.h"
#include "material/Material.h"
#include "utils/EventTarget.h"

namespace Cannon::Objects {
    class Body;
}

namespace Cannon::Shapes {

/**
 * The available shape types.
 * @static
 * @property types
 * @type {Object}
 */
enum ShapeTypes {
    SPHERE = 1,
    PLANE = 2,
    BOX = 4,
    COMPOUND = 8,
    CONVEXPOLYHEDRON = 16,
    HEIGHTFIELD = 32,
    PARTICLE = 64,
    CYLINDER = 128,
    TRIMESH = 256
};

class Shape : public Utils::EventTarget {
private:

public:
    static int idCounter;

    int id;

    /**
     * The type of this shape. Must be set to an int > 0 by subclasses.
     * @property type
     * @type {Number}
     * @see Shape.types
     */
    ShapeTypes type;

    /**
     * The local bounding sphere radius of this shape.
     * @property {Number} boundingSphereRadius
     */
    float boundingSphereRadius;

    /**
     * Whether to produce contact forces when in contact with other bodies. Note that contacts will be generated, but they will be disabled.
     * @property {boolean} collisionResponse
     */
    bool collisionResponse;

    /**
     * @property {Number} collisionFilterGroup
     */
    int collisionFilterGroup;

    /**
     * @property {Number} collisionFilterMask
     */
    int collisionFilterMask;

    /**
     * @property {Material} material
     */
    Material::Material* material;

    /**
     * @property {Body} body
     */
    Objects::Body* body;

    /**
     * Base class for shapes
     * @class Shape
     * @constructor
     */
    Shape();

    /**
     * Computes the bounding sphere radius. The result is stored in the property .boundingSphereRadius
     * @method updateBoundingSphereRadius
     */
    virtual void updateBoundingSphereRadius() = 0;

    /**
     * Get the volume of this shape
     * @method volume
     * @return {Number}
     */
    virtual float volume() = 0;

    /**
     * Calculates the inertia in the local frame for this shape.
     * @method calculateLocalInertia
     * @param {Number} mass
     * @param {Vec3} target
     * @see http://en.wikipedia.org/wiki/List_of_moments_of_inertia
     */
    virtual void calculateLocalInertia(float mass, Math::Vec3 target) = 0;
};

}

#endif