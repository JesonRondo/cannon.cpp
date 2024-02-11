#ifndef AABB_h
#define AABB_h

#include "math/Vec3.h"

namespace Cannon::Math {
    class Quaternion;
    class Transform;
}

namespace Cannon::Collision {

class Ray;

class AABB {
private:
    /**
     * The lower bound of the bounding box.
     * @property lowerBound
     * @type {Vec3}
     */
    Math::Vec3 lowerBound_;

    /**
     * The upper bound of the bounding box.
     * @property upperBound
     * @type {Vec3}
     */
    Math::Vec3 upperBound_;

public:
    /**
     * Axis aligned bounding box class.
     * @class AABB
     * @constructor
     */
    AABB();
    
    /**
     * Axis aligned bounding box class.
     * @class AABB
     * @constructor
     * @param {Vec3} upperBound
     * @param {Vec3} lowerBound
     */
    AABB(Math::Vec3 lowerBound, Math::Vec3 upperBound);

    /**
     * Set the AABB bounds from a set of points.
     * @method setFromPoints
     * @param {Array} points An array of Vec3's.
     * @param {Vec3} position
     * @param {Quaternion} quaternion
     * @param {number} skinSize
     * @return {AABB} The self object
     */
    AABB setFromPoints(Math::Vec3* points, Math::Vec3 position, Math::Quaternion quaternion, float skinSize);
    
    /**
     * Copy bounds from an AABB to this AABB
     * @method copy
     * @param  {AABB} aabb Source to copy from
     * @return {AABB} The this object, for chainability
     */
    AABB copy(AABB aabb);

    /**
     * Clone an AABB
     * @method clone
     */
    AABB clone();

    /**
     * Extend this AABB so that it covers the given AABB too.
     * @method extend
     * @param  {AABB} aabb
     */
    void extend(AABB aabb);

    /**
     * Returns true if the given AABB overlaps this AABB.
     * @method overlaps
     * @param  {AABB} aabb
     * @return {Boolean}
     */
    bool overlaps(AABB aabb);

    // Mostly for debugging
    float volume();

    /**
     * Returns true if the given AABB is fully contained in this AABB.
     * @method contains
     * @param {AABB} aabb
     * @return {Boolean}
     */
    bool contains(AABB aabb);

    /**
     * @method getCorners
     * @param {Vec3} a
     * @param {Vec3} b
     * @param {Vec3} c
     * @param {Vec3} d
     * @param {Vec3} e
     * @param {Vec3} f
     * @param {Vec3} g
     * @param {Vec3} h
     */
    void getCorners(
        Math::Vec3 a, Math::Vec3 b, Math::Vec3 c, Math::Vec3 d,
        Math::Vec3 e, Math::Vec3 f, Math::Vec3 g, Math::Vec3 h);

    /**
     * Get the representation of an AABB in another frame.
     * @method toLocalFrame
     * @param  {Transform} frame
     * @param  {AABB} target
     * @return {AABB} The "target" AABB object.
     */
    AABB toLocalFrame(Math::Transform frame, AABB target);

    /**
     * Get the representation of an AABB in the global frame.
     * @method toWorldFrame
     * @param  {Transform} frame
     * @param  {AABB} target
     * @return {AABB} The "target" AABB object.
     */
    AABB toWorldFrame(Math::Transform frame, AABB target);

    /**
     * Check if the AABB is hit by a ray.
     * @param  {Ray} ray
     * @return {number}
     */
    bool overlapsRay(Ray ray);
};

} // end namespace Collision

#endif
