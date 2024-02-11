#ifndef World_h
#define World_h

#include <vector>
#include "math/Vec3.h"
#include "material/Material.h"
#include "material/ContactMaterial.h"
#include "constraints/Constraint.h"
#include "objects/Body.h"
#include "collision/Ray.h"
#include "utils/EventTarget.h"

namespace Cannon::World {

struct WorldProfile {
    int solve;
    int makeContactConstraints;
    int broadphase;
    int integrate;
    int narrowphase;
};

class World : public Utils::EventTarget {
private:

public:
    static const std::map<int, Objects::Body*> idToBodyMap;
    static const std::map<int, Shapes::Shape*> idToShapeMap;

    /**
     * Currently / last used timestep. Is set to -1 if not available. This value is updated before each internal step, which means that it is "fresh" inside event callbacks.
     * @property {Number} dt
     */
    float dt;

    /**
     * Makes bodies go to sleep when they've been inactive
     * @property allowSleep
     * @type {Boolean}
     * @default false
     */
    bool allowSleep;

    /**
     * All the current contacts (instances of ContactEquation) in the world.
     * @property contacts
     * @type {Array}
     */
    this.contacts = [];
    this.frictionEquations = [];

    this.triggerDic = new TupleDictionary();
    this.oldTriggerDic = new TupleDictionary();

    this.contactsDic = new TupleDictionary();
    this.oldContactsDic = new TupleDictionary();

    /**
     * How often to normalize quaternions. Set to 0 for every step, 1 for every second etc.. A larger value increases performance. If bodies tend to explode, set to a smaller value (zero to be sure nothing can go wrong).
     * @property quatNormalizeSkip
     * @type {Number}
     * @default 0
     */
    int quatNormalizeSkip;

    /**
     * Set to true to use fast quaternion normalization. It is often enough accurate to use. If bodies tend to explode, set to false.
     * @property quatNormalizeFast
     * @type {Boolean}
     * @see Quaternion.normalizeFast
     * @see Quaternion.normalize
     * @default false
     */
    bool quatNormalizeFast;

    /**
     * The wall-clock time since simulation start
     * @property time
     * @type {Number}
     */
    float time;

    /**
     * Number of timesteps taken since start
     * @property stepnumber
     * @type {Number}
     */
    int stepnumber = 0;

    int substeps = 0;

    /// Default and last timestep sizes
    float default_dt = 1 / 60;

    int nextId = 0;

    /**
     * @property gravity
     * @type {Vec3}
     */
    Math::Vec3 gravity;

    /**
     * The broadphase algorithm to use. Default is NaiveBroadphase
     * @property broadphase
     * @type {Broadphase}
     */
    this.broadphase = options.broadphase !== undefined ? options.broadphase : new NaiveBroadphase();

    /**
     * @property bodies
     * @type {Array}
     */
    std::vector<Objects::Body*> bodies;

    /**
     * The solver algorithm to use. Default is GSSolver
     * @property solver
     * @type {Solver}
     */
    this.solver = options.solver !== undefined ? options.solver : new GSSolver();

    /**
     * @property constraints
     * @type {Array}
     */
    std::vector<Constraints::Constraint*> constraints;

    /**
     * @property narrowphase
     * @type {Narrowphase}
     */
    this.narrowphase = new Narrowphase(this);

    /**
     * @property {ObjectCollisionMatrix} collisionMatrix
	 * @type {ObjectCollisionMatrix}
	 */
    this.collisionMatrix = new ObjectCollisionMatrix();

    this.triggerMatrix = new ObjectCollisionMatrix();

    /**
     * All added materials
     * @property materials
     * @type {Array}
     */
    std::vector<Material::Material*> materials;

    /**
     * @property contactmaterials
     * @type {Array}
     */
    std::vector<Material::ContactMaterial*> contactmaterials;

    /**
     * Used to look up a ContactMaterial given two instances of Material.
     * @property {TupleDictionary} contactMaterialTable
     */
    this.contactMaterialTable = new TupleDictionary();

    Material::Material* defaultMaterial;

    /**
     * This contact material is used if no suitable contactmaterial is found for a contact.
     * @property defaultContactMaterial
     * @type {ContactMaterial}
     */
    Material::ContactMaterial* defaultContactMaterial;

    /**
     * @property profile
     * @type {Object}
     */
    WorldProfile profile {
        solve: 0,
        makeContactConstraints: 0,
        broadphase: 0,
        integrate: 0,
        narrowphase: 0,
    };

    /**
     * Time accumulator for interpolation. See http://gafferongames.com/game-physics/fix-your-timestep/
     * @property {Number} accumulator
     */
    float accumulator = 0;

    /**
     * @property subsystems
     * @type {Array}
     */
    this.subsystems = [];

    /**
     * Dispatched after a body has been added to the world.
     * @event addBody
     * @param {Body} body The body that has been added to the world.
     */
    Objects::BodyEvent addBodyEvent = Objects::BodyEvent("addBody", nullptr);

    /**
     * Dispatched after a body has been removed from the world.
     * @event removeBody
     * @param {Body} body The body that has been removed from the world.
     */
    Objects::BodyEvent removeBodyEvent = Objects::BodyEvent("removeBody", nullptr);

    /**
     * The physics world
     * @class World
     * @constructor
     */
    World();

    /**
     * Get the contact material between materials m1 and m2
     * @method getContactMaterial
     * @param {Material} m1
     * @param {Material} m2
     * @return {ContactMaterial} The contact material if it was found.
     */
    Material::ContactMaterial* getContactMaterial(Material::Material* m1, Material::Material* m2);

    /**
     * Store old collision state info
     * @method collisionMatrixTick
     */
    void collisionMatrixTick();

    /**
     * Add a rigid body to the simulation.
     * @method add
     * @param {Body} body
     * @todo If the simulation has not yet started, why recrete and copy arrays for each body? Accumulate in dynamic arrays in this case.
     * @todo Adding an array of bodies should be possible. This would save some loops too
     * @deprecated Use .addBody instead
     */
    void addBody(Objects::Body* body);

    /**
     * Add a constraint to the simulation.
     * @method addConstraint
     * @param {Constraint} c
     */
    void addConstraint(Constraints::Constraint* c);

    /**
     * Removes a constraint
     * @method removeConstraint
     * @param {Constraint} c
     */
    void removeConstraint(Constraints::Constraint* c);

    /**
     * Ray cast against all bodies. The provided callback will be executed for each hit with a RaycastResult as single argument.
     * @method raycastAll
     * @param  {Vec3} from
     * @param  {Vec3} to
     * @param  {Object} options
     * @param  {number} [options.collisionFilterMask=-1]
     * @param  {number} [options.collisionFilterGroup=-1]
     * @param  {boolean} [options.skipBackfaces=false]
     * @param  {boolean} [options.checkCollisionResponse=true]
     * @param  {Function} callback
     * @return {boolean} True if any body was hit.
     */
    bool raycastAll(Math::Vec3 from, Math::Vec3 to, Collision::RaycastOptions options, Collision::RaycastResultCallback* callback);

    /**
     * Ray cast, and stop at the first result. Note that the order is random - but the method is fast.
     * @method raycastAny
     * @param  {Vec3} from
     * @param  {Vec3} to
     * @param  {Object} options
     * @param  {number} [options.collisionFilterMask=-1]
     * @param  {number} [options.collisionFilterGroup=-1]
     * @param  {boolean} [options.skipBackfaces=false]
     * @param  {boolean} [options.checkCollisionResponse=true]
     * @param  {RaycastResult} result
     * @return {boolean} True if any body was hit.
     */
    bool raycastAny(Math::Vec3 from, Math::Vec3 to, Collision::RaycastOptions options, Collision::RaycastResult* result);

    /**
     * Ray cast, and return information of the closest hit.
     * @method raycastClosest
     * @param  {Vec3} from
     * @param  {Vec3} to
     * @param  {Object} options
     * @param  {number} [options.collisionFilterMask=-1]
     * @param  {number} [options.collisionFilterGroup=-1]
     * @param  {boolean} [options.skipBackfaces=false]
     * @param  {boolean} [options.checkCollisionResponse=true]
     * @param  {RaycastResult} result
     * @return {boolean} True if any body was hit.
     */
    bool raycastClosest(Math::Vec3 from, Math::Vec3 to, Collision::RaycastOptions options, Collision::RaycastResult* result);

    /**
     * Remove a rigid body from the simulation.
     * @method removeBody
     * @param {Body} body
     */
    void removeBody(Objects::Body* body);

    Objects::Body* getBodyById(int id);

    Shapes::Shape* getShapeById(int id);

    /**
     * Adds a material to the World.
     * @method addMaterial
     * @param {Material} m
     * @todo Necessary?
     */
    void addMaterial(Material::Material* m);

    /**
     * Adds a contact material to the World
     * @method addContactMaterial
     * @param {ContactMaterial} cmat
     */
    void addContactMaterial(Material::ContactMaterial* cmat);

    /**
     * Step the physics world forward in time.
     *
     * There are two modes. The simple mode is fixed timestepping without interpolation. In this case you only use the first argument. The second case uses interpolation. In that you also provide the time since the function was last used, as well as the maximum fixed timesteps to take.
     *
     * @method step
     * @param {Number} dt                       The fixed time step size to use.
     * @param {Number} [timeSinceLastCalled]    The time elapsed since the function was last called.
     * @param {Number} [maxSubSteps=10]         Maximum number of fixed steps to take per function call.
     *
     * @example
     *     // fixed timestepping without interpolation
     *     world.step(1/60);
     *
     * @see http://bulletphysics.org/mediawiki-1.5.8/index.php/Stepping_The_World
     */
    void step(float dt, float timeSinceLastCalled, int maxSubSteps);

    void internalStep(float dt);

    void emitTriggeredEvents();

    void emitCollisionEvents();

    /**
     * Sets all body forces in the world to zero.
     * @method clearForces
     */
    void clearForces();
};

}

#endif
