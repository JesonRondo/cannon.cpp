#ifndef Body_h
#define Body_h

#include <vector>

#include "math/Vec3.h"
#include "math/Mat3.h"
#include "math/Quaternion.h"
#include "collision/AABB.h"
#include "material/Material.h"
#include "utils/EventTarget.h"

namespace Cannon::World {
    class World;
}

namespace Cannon::Shapes {
    class Shape;
}

namespace Cannon::Objects {

struct BodyEvent : public Utils::Event {
    Body* body;
    BodyEvent(std::string type, Body* body) : Utils::Event(type), body(body) {}
};

enum BodyType {
    /**
     * A dynamic body is fully simulated. Can be moved manually by the user, but normally they move according to forces. A dynamic body can collide with all body types. A dynamic body always has finite, non-zero mass.
     * @static
     * @property DYNAMIC
     * @type {Number}
     */
    DYNAMIC = 1,

    /**
     * A static body does not move during simulation and behaves as if it has infinite mass. Static bodies can be moved manually by setting the position of the body. The velocity of a static body is always zero. Static bodies do not collide with other static or kinematic bodies.
     * @static
     * @property STATIC
     * @type {Number}
     */
    STATIC = 2,

    /**
     * A kinematic body moves under simulation according to its velocity. They do not respond to forces. They can be moved manually, but normally a kinematic body is moved by setting its velocity. A kinematic body behaves as if it has infinite mass. Kinematic bodies do not collide with other static or kinematic bodies.
     * @static
     * @property KINEMATIC
     * @type {Number}
     */
    KINEMATIC = 4
};

enum BodyState {
    /**
     * @static
     * @property AWAKE
     * @type {number}
     */
    AWAKE = 0,

    /**
     * @static
     * @property SLEEPY
     * @type {number}
     */
    SLEEPY = 1,

    /**
     * @static
     * @property SLEEPING
     * @type {number}
     */
    SLEEPING = 2
};

class Body : public Utils::EventTarget {
private:
    bool _wakeUpAfterNarrowphase;

public:
    /**
     * Dispatched after two bodies collide. This event is dispatched on each
     * of the two bodies involved in the collision.
     * @event collide
     * @param {Body} body The body that was involved in the collision.
     * @param {ContactEquation} contact The details of the collision.
     */
    static const std::string COLLIDE_EVENT_NAME;

    static int idCounter;

    static const Utils::Event wakeupEvent;

    /**
     * Dispatched after a body has gone in to the sleepy state.
     * @event sleepy
     */
    static const Utils::Event sleepyEvent;

    /**
     * Dispatched after a body has fallen asleep.
     * @event sleep
     */
    static const Utils::Event sleepEvent;

    int id;

    /**
     * Reference to the world the body is living in
     * @property world
     * @type {World}
     */
    World::World* world;

    Math::Vec3 vlambda;

    /**
     * @property {Number} collisionFilterGroup
     */
    int collisionFilterGroup;

    /**
     * @property {Number} collisionFilterMask
     */
    int collisionFilterMask;

    /**
     * Whether to produce contact forces when in contact with other bodies. Note that contacts will be generated, but they will be disabled.
     * @property {Number} collisionResponse
     */
    bool collisionResponse;

    /**
     * World space position of the body.
     * @property position
     * @type {Vec3}
     */
    Math::Vec3 position;

    /**
     * @property {Vec3} previousPosition
     */
    Math::Vec3 previousPosition;

    /**
     * Interpolated position of the body.
     * @property {Vec3} interpolatedPosition
     */
    Math::Vec3 interpolatedPosition;

    /**
     * Initial position of the body
     * @property initPosition
     * @type {Vec3}
     */
    Math::Vec3 initPosition;

    /**
     * World space velocity of the body.
     * @property velocity
     * @type {Vec3}
     */
    Math::Vec3 velocity;

    /**
     * @property initVelocity
     * @type {Vec3}
     */
    Math::Vec3 initVelocity;

    /**
     * Linear force on the body in world space.
     * @property force
     * @type {Vec3}
     */
    Math::Vec3 force;

    /**
     * @property mass
     * @type {Number}
     * @default 0
     */
    float mass;

    /**
     * @property invMass
     * @type {Number}
     */
    float invMass;

    /**
     * @property material
     * @type {Material}
     */
    Material::Material* material;

    /**
     * @property linearDamping
     * @type {Number}
     */
    float linearDamping;

    /**
     * One of: Body.DYNAMIC, Body.STATIC and Body.KINEMATIC.
     * @property type
     * @type {Number}
     */
    BodyType type;

    /**
     * If true, the body will automatically fall to sleep.
     * @property allowSleep
     * @type {Boolean}
     * @default true
     */
    bool allowSleep;

    /**
     * Current sleep state.
     * @property sleepState
     * @type {Number}
     */
    BodyState sleepState;

    /**
     * If the speed (the norm of the velocity) is smaller than this value, the body is considered sleepy.
     * @property sleepSpeedLimit
     * @type {Number}
     * @default 0.1
     */
    float sleepSpeedLimit;

    /**
     * If the body has been sleepy for this sleepTimeLimit seconds, it is considered sleeping.
     * @property sleepTimeLimit
     * @type {Number}
     * @default 1
     */
    float sleepTimeLimit;

    float timeLastSleepy;

    /**
     * World space rotational force on the body, around center of mass.
     * @property {Vec3} torque
     */
    Math::Vec3 torque;

    /**
     * World space orientation of the body.
     * @property quaternion
     * @type {Quaternion}
     */
    Math::Quaternion quaternion;

    /**
     * @property initQuaternion
     * @type {Quaternion}
     */
    Math::Quaternion initQuaternion;

    /**
     * @property {Quaternion} previousQuaternion
     */
    Math::Quaternion previousQuaternion;

    /**
     * Interpolated orientation of the body.
     * @property {Quaternion} interpolatedQuaternion
     */
    Math::Quaternion interpolatedQuaternion;

    /**
     * Angular velocity of the body, in world space. Think of the angular velocity as a vector, which the body rotates around. The length of this vector determines how fast (in radians per second) the body rotates.
     * @property angularVelocity
     * @type {Vec3}
     */
    Math::Vec3 angularVelocity;

    /**
     * @property initAngularVelocity
     * @type {Vec3}
     */
    Math::Vec3 initAngularVelocity;

    /**
     * @property shapes
     * @type {array}
     */
    std::vector<Shapes::Shape*> shapes;

    /**
     * Position of each Shape in the body, given in local Body space.
     * @property shapeOffsets
     * @type {array}
     */
    std::vector<Math::Vec3> shapeOffsets;

    /**
     * Orientation of each Shape, given in local Body space.
     * @property shapeOrientations
     * @type {array}
     */
    std::vector<Math::Quaternion> shapeOrientations;

    /**
     * @property inertia
     * @type {Vec3}
     */
    Math::Vec3 inertia;

    /**
     * @property {Vec3} invInertia
     */
    Math::Vec3 invInertia;

    /**
     * @property {Mat3} invInertiaWorld
     */
    Math::Mat3 invInertiaWorld;

    float invMassSolve;

    /**
     * @property {Vec3} invInertiaSolve
     */
    Math::Vec3 invInertiaSolve;

    /**
     * @property {Mat3} invInertiaWorldSolve
     */
    Math::Mat3 invInertiaWorldSolve;

    /**
     * Set to true if you don't want the body to rotate. Make sure to run .updateMassProperties() after changing this.
     * @property {Boolean} fixedRotation
     * @default false
     */
    bool fixedRotation;

    /**
     * use gravity ?
     * @property {Boolean} useGravity
     * @default true
     */
    bool useGravity = true;

    /**
     * @property {Number} angularDamping
     */
    float angularDamping;

    /**
     * Use this property to limit the motion along any world axis. (1,1,1) will allow motion along all axes while (0,0,0) allows none.
     * @property {Vec3} linearFactor
     */
    Math::Vec3 linearFactor;

    /**
     * Use this property to limit the rotational motion along any world axis. (1,1,1) will allow rotation along all axes while (0,0,0) allows none.
     * @property {Vec3} angularFactor
     */
    Math::Vec3 angularFactor;

    /**
     * World space bounding box of the body and its shapes.
     * @property aabb
     * @type {AABB}
     */
    Collision::AABB aabb;

    /**
     * Indicates if the AABB needs to be updated before use.
     * @property aabbNeedsUpdate
     * @type {Boolean}
     */
    bool aabbNeedsUpdate;

    /**
     * Total bounding radius of the Body including its shapes, relative to body.position.
     * @property boundingRadius
     * @type {Number}
     */
    float boundingRadius;

    Math::Vec3 wlambda;

    /**
     * has trigger?
     */
    bool hasTrigger;

    /**
     * @constructor
     */
    Body();

    /**
     * Wake the body up.
     * @method wakeUp
     */
    void wakeUp();

    /**
     * Force body sleep
     * @method sleep
     */
    void sleep();

    /**
     * Called every timestep to update internal sleep timer and change sleep state if needed.
     * @method sleepTick
     * @param {Number} time The world time in seconds
     */
    void sleepTick(float time);

    /**
     * If the body is sleeping, it should be immovable / have infinite mass during solve. We solve it by having a separate "solve mass".
     * @method updateSolveMassProperties
     */
    void updateSolveMassProperties();

    /**
     * Convert a world point to local body frame.
     * @method pointToLocalFrame
     * @param  {Vec3} worldPoint
     * @param  {Vec3} result
     * @return {Vec3}
     */
    Math::Vec3 pointToLocalFrame(Math::Vec3 worldPoint, Math::Vec3 result);

    /**
     * Convert a world vector to local body frame.
     * @method vectorToLocalFrame
     * @param  {Vec3} worldPoint
     * @param  {Vec3} result
     * @return {Vec3}
     */
    Math::Vec3 vectorToLocalFrame(Math::Vec3 worldVector, Math::Vec3 result);

    /**
     * Convert a local body point to world frame.
     * @method pointToWorldFrame
     * @param  {Vec3} localPoint
     * @param  {Vec3} result
     * @return {Vec3}
     */
    Math::Vec3 pointToWorldFrame(Math::Vec3 localPoint, Math::Vec3 result);

    /**
     * Convert a local body point to world frame.
     * @method vectorToWorldFrame
     * @param  {Vec3} localVector
     * @param  {Vec3} result
     * @return {Vec3}
     */
    Math::Vec3 vectorToWorldFrame(Math::Vec3 localVector, Math::Vec3 result);

    /**
     * Add a shape to the body with a local offset and orientation.
     * @method addShape
     * @param {Shape} shape
     * @param {Vec3} [_offset]
     * @param {Quaternion} [_orientation]
     * @return {Body} The body object, for chainability.
     */
    Body addShape(Shapes::Shape* shape, Math::Vec3 _offset, Math::Quaternion _orientation);

    /**
     * Remove a shape from the body
     */
    void removeShape(Shapes::Shape* shape);

    /**
     * Update the bounding radius of the body. Should be done if any of the shapes are changed.
     * @method updateBoundingRadius
     */
    void updateBoundingRadius();

    /**
     * Updates the .aabb
     * @method computeAABB
     * @todo rename to updateAABB()
     */
    void computeAABB();

    /**
     * Update .inertiaWorld and .invInertiaWorld
     * @method updateInertiaWorld
     */
    void updateInertiaWorld(bool force);

    /**
     * Apply force to a world point. This could for example be a point on the Body surface. Applying force this way will add to Body.force and Body.torque.
     * @method applyForce
     * @param  {Vec3} force The amount of force to add.
     * @param  {Vec3} relativePoint A point relative to the center of mass to apply the force on.
     */
    void applyForce(Math::Vec3 force, Math::Vec3 relativePoint);

    /**
     * Apply force to a local point in the body.
     * @method applyLocalForce
     * @param  {Vec3} force The force vector to apply, defined locally in the body frame.
     * @param  {Vec3} localPoint A local point in the body to apply the force on.
     */
    void applyLocalForce(Math::Vec3 localForce, Math::Vec3 localPoint);

    /**
     * Apply impulse to a world point. This could for example be a point on the Body surface. An impulse is a force added to a body during a short period of time (impulse = force * time). Impulses will be added to Body.velocity and Body.angularVelocity.
     * @method applyImpulse
     * @param  {Vec3} impulse The amount of impulse to add.
     * @param  {Vec3} relativePoint A point relative to the center of mass to apply the force on.
     */
    void applyImpulse(Math::Vec3 impulse, Math::Vec3 relativePoint);

    /**
     * Apply locally-defined impulse to a local point in the body.
     * @method applyLocalImpulse
     * @param  {Vec3} force The force vector to apply, defined locally in the body frame.
     * @param  {Vec3} localPoint A local point in the body to apply the force on.
     */
    void applyLocalImpulse(Math::Vec3 localImpulse, Math::Vec3 localPoint);

    /**
     * Should be called whenever you change the body shape or mass.
     * @method updateMassProperties
     */
    void updateMassProperties();

    /**
     * Get world velocity of a point in the body.
     * @method getVelocityAtWorldPoint
     * @param  {Vec3} worldPoint
     * @param  {Vec3} result
     * @return {Vec3} The result vector.
     */
    Math::Vec3 getVelocityAtWorldPoint(Math::Vec3 worldPoint, Math::Vec3 result);

    /**
     * Move the body forward in time.
     * @param {number} dt Time step
     * @param {boolean} quatNormalize Set to true to normalize the body quaternion
     * @param {boolean} quatNormalizeFast If the quaternion should be normalized using "fast" quaternion normalization
     */
    void integrate(float dt, bool quatNormalize, bool quatNormalizeFast);

    /**
     * Is Sleeping
     */
    bool isSleeping();

    /**
     * Is Sleepy
     */
    bool isSleepy();

    /**
     * Is Awake
     */
    bool isAwake();

    /**
     * Update hasTrigger
     */
    void updateHasTrigger();
};

} // namespace Cannon::Objects

#endif
