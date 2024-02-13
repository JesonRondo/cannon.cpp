#ifndef Octree_h
#define Octree_h

#include <vector>
#include "collision/Ray.h"
#include "math/Transform.h"

namespace Cannon::Collision {
    class AABB;
}

namespace Cannon::Utils {

/**
 * @class OctreeNode
 * @param {object} [options]
 * @param {Octree} [options.root]
 * @param {AABB} [options.aabb]
 */
template <typename T>
class OctreeNode {
public:
    /**
     * The root node
     * @property {OctreeNode} root
     */
    OctreeNode* root;

    /**
     * Boundary of this node
     * @property {AABB} aabb
     */
    Collision::AABB aabb;

    /**
     * Contained data at the current node level.
     * @property {Array} data
     */
    std::vector<T> data;

    /**
     * Children to this node
     * @property {Array} children
     */
    std::vector<OctreeNode *> children;

    void reset();

    /**
     * Insert data into this node
     * @method insert
     * @param  {AABB} aabb
     * @param  {object} elementData
     * @return {boolean} True if successful, otherwise false
     */
    bool insert(Collision::AABB aabb, std::vector<T>* elementData, int level);

    /**
     * Create 8 equally sized children nodes and put them in the .children array.
     * @method subdivide
     */
    void subdivide();

    /**
     * Get all data, potentially within an AABB
     * @method aabbQuery
     * @param  {AABB} aabb
     * @param  {array} result
     * @return {array} The "result" object
     */
     std::vector<std::vector<T>>* aabbQuery(Collision::AABB aabb, std::vector<std::vector<T>>* result);

    /**
     * Get all data, potentially intersected by a ray.
     * @method rayQuery
     * @param  {Ray} ray
     * @param  {Transform} treeTransform
     * @param  {array} result
     * @return {array} The "result" object
     */
    std::vector<std::vector<T>>* rayQuery(
        Collision::Ray ray,
        Math::Transform treeTransform,
        std::vector<std::vector<T>>* result);

    /**
     * @method removeEmptyNodes
     */
    void removeEmptyNodes();
};

template <typename T>
class Octree : public OctreeNode<T> {
public:
    /**
     * Maximum subdivision depth
     * @property {number} maxDepth
     */
    int maxDepth = 8;

    /**
     * @class Octree
     * @param {AABB} aabb The total AABB of the tree
     * @param {object} [options]
     * @param {number} [options.maxDepth=8]
     * @extends OctreeNode
     */
    Octree(Collision::AABB aabb);
};

}

#endif
