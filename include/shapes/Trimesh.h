#ifndef Trimesh_h
#define Trimesh_h

#include "shapes/Shape.h"
#include "math/Quaternion.h"
#include "utils/Octree.h"

namespace Cannon::Collision {
    class AABB;
}

namespace Cannon::Shapes {

class Trimesh : public Shape {
private:
    /**
     * Get raw vertex i
     * @private
     * @method _getUnscaledVertex
     * @param  {number} i
     * @param  {Vec3} out
     * @return {Vec3} The "out" vector object
     */
    Math::Vec3* getUnscaledVertex_(int i, Math::Vec3* out);

public:
    /**
     * Create a Trimesh instance, shaped as a torus.
     * @static
     * @method createTorus
     * @param  {number} [radius=1]
     * @param  {number} [tube=0.5]
     * @param  {number} [radialSegments=8]
     * @param  {number} [tubularSegments=6]
     * @param  {number} [arc=6.283185307179586]
     * @return {Trimesh} A torus
     */
    static Trimesh* createTorus(float radius, float tube, int radialSegments, int tubularSegments, double arc);

    /**
     * @property vertices
     * @type {Array}
     */
    float* vertices;

    /**
     * Array of integers, indicating which vertices each triangle consists of. The length of this array is thus 3 times the number of triangles.
     * @property indices
     * @type {Array}
     */
    int* indices;

    /**
     * The normals data.
     * @property normals
     * @type {Array}
     */
    float* normals;

    /**
     * The local AABB of the mesh.
     * @property aabb
     * @type {Array}
     */
    Collision::AABB* aabb;

    /**
     * References to vertex pairs, making up all unique edges in the trimesh.
     * @property {array} edges
     */
    int* edges;

    /**
     * Local scaling of the mesh. Use .setScale() to set it.
     * @property {Vec3} scale
     */
    Math::Vec3 scale = Vec3(1, 1, 1);

    /**
     * The indexed triangles. Use .updateTree() to update it.
     * @property {Octree} tree
     */
    Utils::Octree<std::vector<float>>* tree;

    /**
     * @class Trimesh
     * @constructor
     * @param {array} vertices
     * @param {array} indices
     * @extends Shape
     * @example
     *     // How to make a mesh with a single triangle
     *     var vertices = [
     *         0, 0, 0, // vertex 0
     *         1, 0, 0, // vertex 1
     *         0, 1, 0  // vertex 2
     *     ];
     *     var indices = [
     *         0, 1, 2  // triangle 0
     *     ];
     *     var trimeshShape = new Trimesh(vertices, indices);
     */
    Trimesh(float* vertices, int* indices);

    /**
     * @method updateTree
     */
    void updateTree();

    /**
     * Get triangles in a local AABB from the trimesh.
     * @method getTrianglesInAABB
     * @param  {AABB} aabb
     * @param  {array} result An array of integers, referencing the queried triangles.
     */
    std::vector<std::vector<float>>* getTrianglesInAABB(
        Collision::AABB* aabb,
        std::vector<std::vector<float>>* result);

    /**
     * @method setScale
     * @param {Vec3} scale
     */
    void setScale(Math::Vec3 scale);

    /**
     * Compute the normals of the faces. Will save in the .normals array.
     * @method updateNormals
     */
    void updateNormals();

    /**
     * Update the .edges property
     * @method updateEdges
     */
    void updateEdges();

    /**
     * Get an edge vertex
     * @method getEdgeVertex
     * @param  {number} edgeIndex
     * @param  {number} firstOrSecond 0 or 1, depending on which one of the vertices you need.
     * @param  {Vec3} vertexStore Where to store the result
     */
    void getEdgeVertex(int edgeIndex, int firstOrSecond, Math::Vec3* vertexStore);

    /**
     * Get a vector along an edge.
     * @method getEdgeVector
     * @param  {number} edgeIndex
     * @param  {Vec3} vectorStore
     */
    void getEdgeVector(int edgeIndex, Math::Vec3* vectorStore);

    /**
     * Get face normal given 3 vertices
     * @static
     * @method computeNormal
     * @param {Vec3} va
     * @param {Vec3} vb
     * @param {Vec3} vc
     * @param {Vec3} target
     */
    void computeNormal(Math::Vec3 va, Math::Vec3 vb, Math::Vec3 vc, Math::Vec3* target);

    /**
     * Get vertex i.
     * @method getVertex
     * @param  {number} i
     * @param  {Vec3} out
     * @return {Vec3} The "out" vector object
     */
    Math::Vec3* getVertex(int i, Math::Vec3* out);

    /**
     * Get a vertex from the trimesh,transformed by the given position and quaternion.
     * @method getWorldVertex
     * @param  {number} i
     * @param  {Vec3} pos
     * @param  {Quaternion} quat
     * @param  {Vec3} out
     * @return {Vec3} The "out" vector object
     */
    Math::Vec3* getWorldVertex(int i, Math::Vec3 pos, Math::Quaternion quat, Math::Vec3* out);

    /**
     * Get the three vertices for triangle i.
     * @method getTriangleVertices
     * @param  {number} i
     * @param  {Vec3} a
     * @param  {Vec3} b
     * @param  {Vec3} c
     */
    void getTriangleVertices(int i, Math::Vec3* a, Math::Vec3* b, Math::Vec3* c);

    /**
     * Compute the normal of triangle i.
     * @method getNormal
     * @param  {Number} i
     * @param  {Vec3} target
     * @return {Vec3} The "target" vector object
     */
    Math::Vec3* getNormal(int i, Math::Vec3* target);

    /**
     * @method calculateLocalInertia
     * @param  {Number} mass
     * @param  {Vec3} target
     * @return {Vec3} The "target" vector object
     */
    void calculateLocalInertia(float mass, Math::Vec3* target);

    /**
     * Compute the local AABB for the trimesh
     * @method computeLocalAABB
     * @param  {AABB} aabb
     */
    void computeLocalAABB(Collision::AABB* aabb);

    /**
     * Update the .aabb property
     * @method updateAABB
     */
    void updateAABB();

    /**
     * Will update the .boundingSphereRadius property
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
    void calculateWorldAABB(Math::Vec3 pos, Math::Quaternion quat, Math::Vec3* min, Math::Vec3* max);

    /**
     * Get approximate volume
     * @method volume
     * @return {Number}
     */
    double volume();
};

}

#endif
