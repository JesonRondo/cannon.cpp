#ifndef Broadphase_h
#define Broadphase_h

#include <vector>

namespace Cannon::World {
    class World;
}

namespace Cannon::Objects {
    class Body;
}

namespace Cannon::Collision {

class AABB;

class Broadphase {
private:

public:
    /**
    * The world to search for collisions in.
    * @property world
    * @type {World}
    */
    World::World* world;

    /**
     * If set to true, the broadphase uses bounding boxes for intersection test, else it uses bounding spheres.
     * @property useBoundingBoxes
     * @type {Boolean}
     */
    bool useBoundingBoxes = false;

    /**
     * Set to true if the objects in the world moved.
     * @property {Boolean} dirty
     */
    bool dirty = true;

    /**
     * Base class for broadphase implementations
     * @class Broadphase
     * @constructor
     * @author schteppe
     */
    Broadphase() {};

    /**
     * Get the collision pairs from the world
     * @method collisionPairs
     * @param {World} world The world to search in
     * @param {Array} p1 Empty array to be filled with body objects
     * @param {Array} p2 Empty array to be filled with body objects
     */
    virtual void collisionPairs(
        World::World* world,
        std::vector<Objects::Body*>* p1,
        std::vector<Objects::Body*>* p2) = 0;

    /**
     * Check if a body pair needs to be intersection tested at all.
     * @method needBroadphaseCollision
     * @param {Body} bodyA
     * @param {Body} bodyB
     * @return {bool}
     */
    bool needBroadphaseCollision(Objects::Body* bodyA, Objects::Body* bodyB);

    /**
     * Check if the bounding volumes of two bodies intersect.
     * @method intersectionTest
     * @param {Body} bodyA
     * @param {Body} bodyB
     * @param {array} pairs1
     * @param {array} pairs2
     */
    void intersectionTest(
        Objects::Body* bodyA,
        Objects::Body* bodyB,
        std::vector<Objects::Body*>* pairs1,
        std::vector<Objects::Body*>* pairs2);

    /**
     * Check if the bounding spheres of two bodies are intersecting.
     * @method doBoundingSphereBroadphase
     * @param {Body} bodyA
     * @param {Body} bodyB
     * @param {Array} pairs1 bodyA is appended to this array if intersection
     * @param {Array} pairs2 bodyB is appended to this array if intersection
     */
    void doBoundingSphereBroadphase(
        Objects::Body* bodyA,
        Objects::Body* bodyB,
        std::vector<Objects::Body*>* pairs1,
        std::vector<Objects::Body*>* pairs2);

    /**
     * Check if the bounding boxes of two bodies are intersecting.
     * @method doBoundingBoxBroadphase
     * @param {Body} bodyA
     * @param {Body} bodyB
     * @param {Array} pairs1
     * @param {Array} pairs2
     */
    void doBoundingBoxBroadphase(
        Objects::Body* bodyA,
        Objects::Body* bodyB,
        std::vector<Objects::Body*>* pairs1,
        std::vector<Objects::Body*>* pairs2);

    /**
     * Removes duplicate pairs from the pair arrays.
     * @method makePairsUnique
     * @param {Array} pairs1
     * @param {Array} pairs2
     */
    void makePairsUnique(
        std::vector<Objects::Body*>* pairs1,
        std::vector<Objects::Body*>* pairs2);

    /**
     * To be implemented by subcasses
     * @method setWorld
     * @param {World} world
     */
    virtual void setWorld(World::World* world);

    /**
     * Check if the bounding spheres of two bodies overlap.
     * @method boundingSphereCheck
     * @param {Body} bodyA
     * @param {Body} bodyB
     * @return {boolean}
     */
    bool boundingSphereCheck(Objects::Body* bodyA, Objects::Body* bodyB);

    /**
     * Returns all the bodies within the AABB.
     * @method aabbQuery
     * @param  {World} world
     * @param  {AABB} aabb
     * @param  {array} result An array to store resulting bodies in.
     * @return {array}
     */
    virtual std::vector<Objects::Body*>* aabbQuery(
        World::World* world,
        Collision::AABB* aabb,
        std::vector<Objects::Body*>* result);
};

}

#endif
