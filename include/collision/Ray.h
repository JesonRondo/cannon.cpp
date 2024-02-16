#ifndef Ray_h
#define Ray_h

#include <vector>
#include <functional>
#include "math/Vec3.h"
#include "math/Quaternion.h"
#include "objects/Body.h"
#include "shapes/Shape.h"
#include "collision/RaycastResult.h"

namespace Cannon::World {
    class World;
}

namespace Cannon::Collision {

enum RayMode {
    CLOSEST = 1,
    ANY = 2,
    ALL = 4,
};

class AABB;

struct RaycastOptions {
    int collisionFilterMask = -1;
    int collisionFilterGroup = -1;
    bool skipBackfaces = false;
    bool checkCollisionResponse = true;
};

typedef std::function<void(RaycastResult* result)> RaycastResultCallback;

class Ray {
private:
    /**
     * @private
     * @property {Vec3} _direction
     */
    Math::Vec3 _direction;

    /**
     * Updates the _direction vector.
     * @private
     * @method _updateDirection
     */
    void _updateDirection();

public:
    /**
     * @property {Vec3} from
     */
    Math::Vec3 from;

    /**
     * @property {Vec3} to
     */
    Math::Vec3 to;
    
    /**
     * The precision of the ray. Used when checking parallelity etc.
     * @property {Number} precision
     */
    float precision = 0.0001;

    /**
     * Set to true if you want the Ray to take .collisionResponse flags into account on bodies and shapes.
     * @property {Boolean} checkCollisionResponse
     */
    bool checkCollisionResponse = true;

    /**
     * If set to true, the ray skips any hits with normal.dot(rayDirection) < 0.
     * @property {Boolean} skipBackfaces
     */
    bool skipBackfaces = false;

    /**
     * @property {number} collisionFilterMask
     * @default -1
     */
    int collisionFilterMask = -1;

    /**
     * @property {number} collisionFilterGroup
     * @default -1
     */
    int collisionFilterGroup = -1;

    /**
     * The intersection mode. Should be Ray.ANY, Ray.ALL or Ray.CLOSEST.
     * @property {number} mode
     */
    RayMode mode = RayMode::ANY;

    /**
     * Current result object.
     * @property {RaycastResult} result
     */
    RaycastResult result;

    /**
     * Will be set to true during intersectWorld() if the ray hit anything.
     * @property {Boolean} hasHit
     */
    bool hasHit = false;

    /**
     * Current, user-provided result callback. Will be used if mode is Ray.ALL.
     * @property {Function} callback
     */
    RaycastResultCallback callback;

    /*
    * As per "Barycentric Technique" as named here http://www.blackpawn.com/texts/pointinpoly/default.html But without the division
    */
    static bool pointInTriangle(Math::Vec3 p, Math::Vec3 a, Math::Vec3 b, Math::Vec3 c);

    /**
     * A line in 3D space that intersects bodies and return points.
     * @class Ray
     * @constructor
     */
    Ray();

    /**
     * A line in 3D space that intersects bodies and return points.
     * @class Ray
     * @constructor
     * @param {Vec3} from
     * @param {Vec3} to
     */
    Ray(Math::Vec3 from, Math::Vec3 to): from(from), to(to){};

    /**
     * Do itersection against all bodies in the given World.
     * @method intersectWorld
     * @param  {World} world
     * @return {Boolean} True if the ray hit anything, otherwise false.
     */
    bool intersectWorld(World::World* world);

    /**
     * Shoot a ray at a body, get back information about the hit.
     * @method intersectBody
     * @private
     * @param {Body} body
     * @param {RaycastResult} [result] Deprecated - set the result property of the Ray instead.
     */
    void intersectBody(Objects::Body* body, RaycastResult result);

    /**
     * @method intersectBodies
     * @param {Array} bodies An array of Body objects.
     * @param {RaycastResult} [result] Deprecated
     */
    void intersectBodies(std::vector<Objects::Body*> bodies, RaycastResult result);

    /**
     * @method intersectShape
     * @private
     * @param {Shape} shape
     * @param {Quaternion} quat
     * @param {Vec3} position
     * @param {Body} body
     */
    void intersectShape(Shapes::Shape* shape, Math::Quaternion quat, Math::Vec3 position, Objects::Body* body);

    /**
     * @method intersectBox
     * @private
     * @param  {Shape} shape
     * @param  {Quaternion} quat
     * @param  {Vec3} position
     * @param  {Body} body
     */
    void intersectBox(Shapes::Shape* shape, Math::Quaternion quat, Math::Vec3 position, Objects::Body* body, int reportedShape);
    // Ray.prototype[Shape.types.BOX] = Ray.prototype.intersectBox;

    /**
     * @method intersectPlane
     * @private
     * @param  {Shape} shape
     * @param  {Quaternion} quat
     * @param  {Vec3} position
     * @param  {Body} body
     */
    void intersectPlane(Shapes::Shape* shape, Math::Quaternion quat, Math::Vec3 position, Objects::Body* body, int reportedShape);
    // Ray.prototype[Shape.types.PLANE] = Ray.prototype.intersectPlane;

    /**
     * Get the world AABB of the ray.
     * @method getAABB
     * @param  {AABB} aabb
     */
    void getAABB(AABB result);

    /**
     * @method intersectHeightfield
     * @private
     * @param  {Shape} shape
     * @param  {Quaternion} quat
     * @param  {Vec3} position
     * @param  {Body} body
     */
    void intersectHeightfield(Shapes::Shape* shape, Math::Quaternion quat, Math::Vec3 position, Objects::Body* body, int reportedShape);
    // Ray.prototype[Shape.types.HEIGHTFIELD] = Ray.prototype.intersectHeightfield;

    /**
     * @method intersectSphere
     * @private
     * @param  {Shape} shape
     * @param  {Quaternion} quat
     * @param  {Vec3} position
     * @param  {Body} body
     */
    void intersectSphere(Shapes::Shape* shape, Math::Quaternion quat, Math::Vec3 position, Objects::Body* body, int reportedShape);
    // Ray.prototype[Shape.types.SPHERE] = Ray.prototype.intersectSphere;

    /**
     * @method intersectConvex
     * @private
     * @param  {Shape} shape
     * @param  {Quaternion} quat
     * @param  {Vec3} position
     * @param  {Body} body
     * @param {object} [options]
     * @param {array} [options.faceList]
     */
    void intersectConvex(
        Shapes::Shape* shape,
        Math::Quaternion quat,
        Math::Vec3 position,
        Objects::Body* body,
        int reportedShape,
        std::vector<int> faceList
    );
    // Ray.prototype[Shape.types.CONVEXPOLYHEDRON] = Ray.prototype.intersectConvex;

    /**
     * @method intersectTrimesh
     * @private
     * @param  {Shape} shape
     * @param  {Quaternion} quat
     * @param  {Vec3} position
     * @param  {Body} body
     * @param {object} [options]
     * @todo Optimize by transforming the world to local space first.
     * @todo Use Octree lookup
     */
    void intersectTrimesh(
        Shapes::Shape* mesh,
        Math::Quaternion quat,
        Math::Vec3 position,
        Objects::Body* body,
        int reportedShape,
        std::vector<int> faceList
    );
    // Ray.prototype[Shape.types.TRIMESH] = Ray.prototype.intersectTrimesh;


    /**
     * @method reportIntersection
     * @private
     * @param  {Vec3} normal
     * @param  {Vec3} hitPointWorld
     * @param  {Shape} shape
     * @param  {Body} body
     * @return {boolean} True if the intersections should continue
     */
    bool reportIntersection(Math::Vec3 normal, Math::Vec3 hitPointWorld, Shapes::Shape* shape, Objects::Body* body, int hitFaceIndex);
};

} // end namespace Collision

#endif
