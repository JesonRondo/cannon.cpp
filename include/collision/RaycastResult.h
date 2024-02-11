#ifndef RaycastResult_h
#define RaycastResult_h

#include "math/Vec3.h"

namespace Cannon::Collision {

class RaycastResult {
private:
    /**
     * If the ray should stop traversing the bodies.
     * @private
     * @property {Boolean} _shouldStop
     * @default false
     */
    bool _shouldStop = false;

public:
    /**
     * @property {Vec3} rayFromWorld
     */
    Math::Vec3 rayFromWorld;

    /**
     * @property {Vec3} rayToWorld
     */
    Math::Vec3 rayToWorld;

    /**
     * @property {Vec3} hitNormalWorld
     */
    Math::Vec3 hitNormalWorld;

    /**
     * @property {Vec3} hitPointWorld
     */
    Math::Vec3 hitPointWorld;

    /**
     * @property {boolean} hasHit
     */
    bool hasHit = false;

    /**
     * The hit shape, or null.
     * @property {Shape} shape
     */
    Shape shape = nullptr;

    /**
     * The hit body, or null.
     * @property {Body} body
     */
    Body body = nullptr;

    /**
     * The index of the hit triangle, if the hit shape was a trimesh.
     * @property {number} hitFaceIndex
     * @default -1
     */
    int hitFaceIndex = -1;

    /**
     * Distance to the hit. Will be set to -1 if there was no hit.
     * @property {number} distance
     * @default -1
     */
    float distance = -1;

    /**
     * @constructor RaycastResult
     */
    RaycastResult(){};

    /**
     * Reset all result data.
     * @method reset
     */
    void reset();

    /**
     * @method abort
     */
    void abort();

    /**
     * @method set
     * @param {Vec3} rayFromWorld
     * @param {Vec3} rayToWorld
     * @param {Vec3} hitNormalWorld
     * @param {Vec3} hitPointWorld
     * @param {Shape} shape
     * @param {Body} body
     * @param {number} distance
     */
    void set(
        Math::Vec3 rayFromWorld,
        Math::Vec3 rayToWorld,
        Math::Vec3 hitNormalWorld,
        Math::Vec3 hitPointWorld,
        Shape shape,
        Body body,
        float distance
    );
};

} // namespace Collision

#endif
