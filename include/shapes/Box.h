#ifndef Box_h
#define Box_h

#include "shapes/Shape.h"
#include "math/Quaternion.h"

namespace Cannon::Shapes {

typedef void (*CornerCallback)(float x, float y, float z);

class Box : public Shape {
public:
    /**
     * @property halfExtents
     * @type {Vec3}
     */
    Math::Vec3* halfExtents;

    /**
     * Used by the contact generator to make contacts with other convex polyhedra for example
     * @property convexPolyhedronRepresentation
     * @type {ConvexPolyhedron}
     */
    ConvexPolyhedron* convexPolyhedronRepresentation = nullptr;

    /**
     * A 3d box shape.
     * @class Box
     * @constructor
     * @param {Vec3} halfExtents
     * @author schteppe
     * @extends Shape
     */
    Box(Math::Vec3* halfExtents);

    /**
     * Updates the local convex polyhedron representation used for some collisions.
     * @method updateConvexPolyhedronRepresentation
     */
    void updateConvexPolyhedronRepresentation();

    /**
     * @method calculateLocalInertia
     * @param  {Number} mass
     * @param  {Vec3} target
     * @return {Vec3}
     */
    Math::Vec3* calculateLocalInertia(float mass, Math::Vec3* target);

    void calculateInertia(Math::Vec3* halfExtents, float mass, Math::Vec3* target);

    /**
     * Get the box 6 side normals
     * @method getSideNormals
     * @param {array}      sixTargetVectors An array of 6 vectors, to store the resulting side normals in.
     * @param {Quaternion} quat             Orientation to apply to the normal vectors. If not provided, the vectors will be in respect to the local frame.
     * @return {array}
     */
    std::array<Math::Vec3*, 6>* getSideNormals(
        std::array<Math::Vec3*, 6>* sixTargetVectors,
        Math::Quaternion* quat);

    double volume();

    void updateBoundingSphereRadius();

    void forEachWorldCorner(Math::Vec3* pos, Math::Quaternion* quat, CornerCallback* callback);

    void calculateWorldAABB(Math::Vec3* pos, Math::Quaternion* quat, Math::Vec3* min, Math::Vec3* max);

};

}

#endif