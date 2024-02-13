#ifndef NaiveBroadphase_h
#define NaiveBroadphase_h

#include "collision/Broadphase.h"

namespace Cannon::World {
    class World;
}

namespace Cannon::Objects {
    class Body;
}

namespace Cannon::Collision {

class NaiveBroadphase : public Collision::Broadphase {
private:

public:
    /**
     * Get all the collision pairs in the physics world
     * @method collisionPairs
     * @param {World} world
     * @param {Array} pairs1
     * @param {Array} pairs2
     */
    void collisionPairs(
        World::World* world,
        std::vector<Objects::Body*>* pairs1,
        std::vector<Objects::Body*>* pairs2);

    /**
     * Returns all the bodies within an AABB.
     * @method aabbQuery
     * @param  {World} world
     * @param  {AABB} aabb
     * @param {array} result An array to store resulting bodies in.
     * @return {array}
     */
    std::vector<Objects::Body*>* aabbQuery(
        World::World* world,
        Collision::AABB* aabb,
        std::vector<Objects::Body*>* result);
};

}

#endif
