#ifndef ConvexPolyhedron_h
#define ConvexPolyhedron_h

#include <vector>
#include "shapes/Shape.h"
#include "math/Quaternion.h"

namespace Cannon::Shapes {

struct PointObject {
    Math::Vec3 point;
    Math::Vec3 normal;
    float depth;
};

class ConvexPolyhedron : public Shape {

public:
    /**
     * Array of Vec3
     * @property vertices
     * @type {Array}
     */
    std::vector<Math::Vec3> vertices;

    std::vector<Math::Vec3> worldVertices; // World transformed version of .vertices
    bool worldVerticesNeedsUpdate = true;

    /**
     * Array of integer arrays, indicating which vertices each face consists of
     * @property faces
     * @type {Array}
     */
    std::vector<std::vector<int>> faces;

    /**
     * Array of Vec3
     * @property faceNormals
     * @type {Array}
     */
    std::vector<Math::Vec3> faceNormals;

    bool worldFaceNormalsNeedsUpdate = true;
    std::vector<Math::Vec3> worldFaceNormals; // World transformed version of .faceNormals

    /**
     * Array of Vec3
     * @property uniqueEdges
     * @type {Array}
     */
    std::vector<Math::Vec3> uniqueEdges;

    /**
     * If given, these locally defined, normalized axes are the only ones being checked when doing separating axis check.
     * @property {Array} uniqueAxes
     */
    std::vector<Math::Vec3> uniqueAxes;

    /**
     * A set of polygons describing a convex shape.
     * @class ConvexPolyhedron
     * @constructor
     * @extends Shape
     * @description The shape MUST be convex for the code to work properly. No polygons may be coplanar (contained
     * in the same 3D plane), instead these should be merged into one polygon.
     *
     * @param {array} points An array of Vec3's
     * @param {array} faces Array of integer arrays, describing which vertices that is included in each face.
     *
     * @author qiao / https://github.com/qiao (original author, see https://github.com/qiao/three.js/commit/85026f0c769e4000148a67d45a9e9b9c5108836f)
     * @author schteppe / https://github.com/schteppe
     * @see http://www.altdevblogaday.com/2011/05/13/contact-generation-between-3d-convex-meshes/
     * @see http://bullet.googlecode.com/svn/trunk/src/BulletCollision/NarrowPhaseCollision/btPolyhedralContactClipping.cpp
     *
     * @todo Move the clipping functions to ContactGenerator?
     * @todo Automatically merge coplanar polygons in constructor.
     */
    ConvexPolyhedron(
        std::vector<Math::Vec3> points,
        std::vector<std::vector<int>> faces,
        std::vector<Math::Vec3> uniqueAxes);

        /**
         * Computes uniqueEdges
         * @method computeEdges
         */
        void computeEdges();

        /**
         * Compute the normals of the faces. Will reuse existing Vec3 objects in the .faceNormals array if they exist.
         * @method computeNormals
         */
        void computeNormals();

        /**
         * Get face normal given 3 vertices
         * @static
         * @method getFaceNormal
         * @param {Vec3} va
         * @param {Vec3} vb
         * @param {Vec3} vc
         * @param {Vec3} target
         */
        void computeNormal(Math::Vec3 va, Math::Vec3 vb, Math::Vec3 vc, Math::Vec3* target);

        /**
         * Compute the normal of a face from its vertices
         * @method getFaceNormal
         * @param  {Number} i
         * @param  {Vec3} target
         */
        void getFaceNormal(int i, Math::Vec3* target);

        /**
         * @method clipAgainstHull
         * @param {Vec3} posA
         * @param {Quaternion} quatA
         * @param {ConvexPolyhedron} hullB
         * @param {Vec3} posB
         * @param {Quaternion} quatB
         * @param {Vec3} separatingNormal
         * @param {Number} minDist Clamp distance
         * @param {Number} maxDist
         * @param {array} result The an array of contact point objects, see clipFaceAgainstHull
         * @see http://bullet.googlecode.com/svn/trunk/src/BulletCollision/NarrowPhaseCollision/btPolyhedralContactClipping.cpp
         */
        void clipAgainstHull(
            Math::Vec3 posA,
            Math::Quaternion quatA,
            ConvexPolyhedron* hullB,
            Math::Vec3 posB,
            Math::Quaternion quatB,
            Math::Vec3 separatingNormal,
            float minDist,
            float maxDist,
            std::vector<PointObject>* result);

        /**
         * Find the separating axis between this hull and another
         * @method findSeparatingAxis
         * @param {ConvexPolyhedron} hullB
         * @param {Vec3} posA
         * @param {Quaternion} quatA
         * @param {Vec3} posB
         * @param {Quaternion} quatB
         * @param {Vec3} target The target vector to save the axis in
         * @return {bool} Returns false if a separation is found, else true
         */
        bool findSeparatingAxis(
            ConvexPolyhedron* hullB,
            Math::Vec3 posA,
            Math::Quaternion quatA,
            Math::Vec3 posB,
            Math::Quaternion quatB,
            Math::Vec3* target,
            std::vector<int>* faceListA,
            std::vector<int>* faceListB);

        /**
         * Test separating axis against two hulls. Both hulls are projected onto the axis and the overlap size is returned if there is one.
         * @method testSepAxis
         * @param {Vec3} axis
         * @param {ConvexPolyhedron} hullB
         * @param {Vec3} posA
         * @param {Quaternion} quatA
         * @param {Vec3} posB
         * @param {Quaternion} quatB
         * @return {number} The overlap depth, or -1 if no penetration.
         */
        float testSepAxis(
            Math::Vec3 axis,
            ConvexPolyhedron* hullB,
            Math::Vec3 posA,
            Math::Quaternion quatA,
            Math::Vec3 posB,
            Math::Quaternion quatB);

        /**
         * @method calculateLocalInertia
         * @param  {Number} mass
         * @param  {Vec3} target
         */
        void calculateLocalInertia(float mass, Math::Vec3* target);

        /**
         * @method getPlaneConstantOfFace
         * @param  {Number} face_i Index of the face
         * @return {Number}
         */
        float getPlaneConstantOfFace(int face_i);

        /**
         * Clip a face against a hull.
         * @method clipFaceAgainstHull
         * @param {Vec3} separatingNormal
         * @param {Vec3} posA
         * @param {Quaternion} quatA
         * @param {Array} worldVertsB1 An array of Vec3 with vertices in the world frame.
         * @param {Number} minDist Distance clamping
         * @param {Number} maxDist
         * @param Array result Array to store resulting contact points in. Will be objects with properties: point, depth, normal. These are represented in world coordinates.
         */
        void clipFaceAgainstHull(
            Math::Vec3 separatingNormal,
            Math::Vec3 posA,
            Math::Quaternion quatA,
            std::vector<Math::Vec3> worldVertsB1,
            float minDist,
            float maxDist,
            std::vector<PointObject>* result);

        /**
         * Clip a face in a hull against the back of a plane.
         * @method clipFaceAgainstPlane
         * @param {Array} inVertices
         * @param {Array} outVertices
         * @param {Vec3} planeNormal
         * @param {Number} planeConstant The constant in the mathematical plane equation
         */
        void clipFaceAgainstPlane(
            std::vector<Math::Vec3> inVertices,
            std::vector<Math::Vec3>* outVertices,
            Math::Vec3 planeNormal,
            float planeConstant);

        // Updates .worldVertices and sets .worldVerticesNeedsUpdate to false.
        void computeWorldVertices(Math::Vec3 position, Math::Quaternion quat);

        void computeLocalAABB(Math::Vec3* aabbmin, Math::Vec3* aabbmax);

        /**
        * Updates .worldVertices and sets .worldVerticesNeedsUpdate to false.
        * @method computeWorldFaceNormals
        * @param  {Quaternion} quat
        */
        void computeWorldFaceNormals(Math::Quaternion quat);

        /**
        * @method updateBoundingSphereRadius
        */
        void updateBoundingSphereRadius();

        /**
        * @method calculateWorldAABB
        * @param {Vec3}        pos
        * @param {Quaternion}  quat
        * @param {Vec3}        min
        * @param {Vec3}        max
        */
        void calculateWorldAABB(
            Math::Vec3 pos,
            Math::Quaternion quat,
            Math::Vec3* min,
            Math::Vec3* max);

        /**
        * Get approximate convex volume
        * @method volume
        * @return {Number}
        */
        double volume();

        /**
        * Get an average of all the vertices positions
        * @method getAveragePointLocal
        * @param  {Vec3} target
        * @return {Vec3}
        */
        Math::Vec3* getAveragePointLocal(Math::Vec3* target);

        /**
        * Transform all local points. Will change the .vertices
        * @method transformAllPoints
        * @param  {Vec3} offset
        * @param  {Quaternion} quat
        */
        void transformAllPoints(Math::Vec3 offset, Math::Quaternion quat);

        /**
        * Checks whether p is inside the polyhedra. Must be in local coords. The point lies outside of the convex hull of the other points if and only if the direction of all the vectors from it to those other points are on less than one half of a sphere around it.
        * @method pointIsInside
        * @param  {Vec3} p      A point given in local coordinates
        * @return {Boolean}
        */
        bool pointIsInside(Math::Vec3 p);

        /**
        * Get max and min dot product of a convex hull at position (pos,quat) projected onto an axis. Results are saved in the array maxmin.
        * @static
        * @method project
        * @param {ConvexPolyhedron} hull
        * @param {Vec3} axis
        * @param {Vec3} pos
        * @param {Quaternion} quat
        * @param {array} result result[0] and result[1] will be set to maximum and minimum, respectively.
        */
        void project(
            ConvexPolyhedron* hull,
            Math::Vec3 axis,
            Math::Vec3 pos,
            Math::Quaternion quat,
            std::array<float, 2> result);
};

}

#endif
