#ifndef Narrowphase_h
#define Narrowphase_h

#include <vector>
#include "objects/Body.h"
#include "shapes/Shape.h"
#include "shapes/Box.h"
#include "shapes/Sphere.h"
#include "shapes/Plane.h"
#include "shapes/Trimesh.h"
#include "shapes/Particle.h"
#include "shapes/Heightfield.h"
#include "shapes/ConvexPolyhedron.h"
#include "utils/Vec3Pool.h"
#include "material/ContactMaterial.h"
#include "equations/ContactEquation.h"
#include "equations/FrictionEquation.h"

namespace Cannon::World {

class World;

class Narrowphase {
private:
    World* world_;

public:
    /**
     * Internal storage of pooled contact points.
     * @property {Array} contactPointPool
     */
    std::vector<Equations::ContactEquation *> contactPointPool;
    std::vector<Equations::FrictionEquation *> frictionEquationPool;

    std::vector<Equations::ContactEquation*> result;
    std::vector<Equations::FrictionEquation*> frictionResult;

    /**
     * Pooled vectors.
     * @property {Vec3Pool} v3pool
     */
    Utils::Vec3Pool v3pool;

    Material::ContactMaterial* currentContactMaterial;

    /**
     * @property {Boolean} enableFrictionReduction
     */
    bool enableFrictionReduction = false;

    /**
     * Helper class for the World. Generates ContactEquations.
     * @class Narrowphase
     * @constructor
     * @todo Sphere-ConvexPolyhedron contacts
     * @todo Contact reduction
     * @todo  should move methods to prototype
     */
    Narrowphase(World* world): world_(world) {};

    /**
     * Make a contact object, by using the internal pool or creating a new one.
     * @method createContactEquation
     * @param {Body} bi
     * @param {Body} bj
     * @param {Shape} si
     * @param {Shape} sj
     * @param {Shape} overrideShapeA
     * @param {Shape} overrideShapeB
     * @return {ContactEquation}
     */
    Equations::ContactEquation* createContactEquation(
        Objects::Body* bi,
        Objects::Body* bj,
        Shapes::Shape* si,
        Shapes::Shape* sj,
        Shapes::Shape* overrideShapeA,
        Shapes::Shape* overrideShapeB);

    bool createFrictionEquationsFromContact(
        Equations::ContactEquation* contactEquation,
        std::vector<Equations::FrictionEquation*>* outArray);

    // Take the average N latest contact point on the plane.
    void createFrictionFromAverage(int numContacts);

    /**
     * Generate all contacts between a list of body pairs
     * @method getContacts
     * @param {array} p1 Array of body indices
     * @param {array} p2 Array of body indices
     * @param {World} world
     * @param {array} result Array to store generated contacts
     * @param {array} oldcontacts Optional. Array of reusable contact objects
     */
    void getContacts(
        std::vector<Objects::Body*>* p1,
        std::vector<Objects::Body*>* p2,
        World* world,
        std::vector<Equations::ContactEquation*>* result,
        std::vector<Equations::ContactEquation*>* oldcontacts,
        std::vector<Equations::FrictionEquation*>* frictionResult,
        std::vector<Equations::FrictionEquation*>* frictionPool);

    // Narrowphase.prototype[Shape.types.BOX | Shape.types.BOX] =
    bool boxBox(
        Shapes::Box* si,
        Shapes::Box* sj,
        Math::Vec3* xi,
        Math::Vec3* xj,
        Math::Quaternion* qi,
        Math::Quaternion* qj,
        Objects::Body* bi,
        Objects::Body* bj,
        Shapes::Shape* rsi,
        Shapes::Shape* rsj,
        bool justTest);

    // Narrowphase.prototype[Shape.types.BOX | Shape.types.CONVEXPOLYHEDRON] =
    bool boxConvex(
        Shapes::Box* si,
        Shapes::Box* sj,
        Math::Vec3* xi,
        Math::Vec3* xj,
        Math::Quaternion* qi,
        Math::Quaternion* qj,
        Objects::Body* bi,
        Objects::Body* bj,
        Shapes::Shape* rsi,
        Shapes::Shape* rsj,
        bool justTest);

    // Narrowphase.prototype[Shape.types.BOX | Shape.types.PARTICLE] =
    bool boxParticle(
        Shapes::Box* si,
        Shapes::Particle* sj,
        Math::Vec3* xi,
        Math::Vec3* xj,
        Math::Quaternion* qi,
        Math::Quaternion* qj,
        Objects::Body* bi,
        Objects::Body* bj,
        Shapes::Shape* rsi,
        Shapes::Shape* rsj,
        bool justTest);

    /**
     * @method sphereSphere
     * @param  {Shape}      si
     * @param  {Shape}      sj
     * @param  {Vec3}       xi
     * @param  {Vec3}       xj
     * @param  {Quaternion} qi
     * @param  {Quaternion} qj
     * @param  {Body}       bi
     * @param  {Body}       bj
     */
    // Narrowphase.prototype[Shape.types.SPHERE] =
    bool sphereSphere(
        Shapes::Sphere* si,
        Shapes::Sphere* sj,
        Math::Vec3* xi,
        Math::Vec3* xj,
        Math::Quaternion* qi,
        Math::Quaternion* qj,
        Objects::Body* bi,
        Objects::Body* bj,
        Shapes::Shape* rsi,
        Shapes::Shape* rsj,
        bool justTest);

    /**
     * @method planeTrimesh
     * @param  {Shape}      si
     * @param  {Shape}      sj
     * @param  {Vec3}       xi
     * @param  {Vec3}       xj
     * @param  {Quaternion} qi
     * @param  {Quaternion} qj
     * @param  {Body}       bi
     * @param  {Body}       bj
     */
    // Narrowphase.prototype[Shape.types.PLANE | Shape.types.TRIMESH] =
    bool planeTrimesh(
        Shapes::Plane* planeShape,
        Shapes::Trimesh* trimeshShape,
        Math::Vec3* planePos,
        Math::Vec3* trimeshPos,
        Math::Quaternion* planeQuat,
        Math::Quaternion* trimeshQuat,
        Objects::Body* planeBody,
        Objects::Body* trimeshBody,
        Shapes::Shape* rsi,
        Shapes::Shape* rsj,
        bool justTest);

    /**
     * @method sphereTrimesh
     * @param  {Shape}      sphereShape
     * @param  {Shape}      trimeshShape
     * @param  {Vec3}       spherePos
     * @param  {Vec3}       trimeshPos
     * @param  {Quaternion} sphereQuat
     * @param  {Quaternion} trimeshQuat
     * @param  {Body}       sphereBody
     * @param  {Body}       trimeshBody
     */
    // Narrowphase.prototype[Shape.types.SPHERE | Shape.types.TRIMESH] =
    bool sphereTrimesh(
        Shapes::Sphere* sphereShape,
        Shapes::Trimesh* trimeshShape,
        Math::Vec3* spherePos,
        Math::Vec3* trimeshPos,
        Math::Quaternion* sphereQuat,
        Math::Quaternion* trimeshQuat,
        Objects::Body* sphereBody,
        Objects::Body* trimeshBody,
        Shapes::Shape* rsi,
        Shapes::Shape* rsj,
        bool justTest);

    /**
     * @method spherePlane
     * @param  {Shape}      si
     * @param  {Shape}      sj
     * @param  {Vec3}       xi
     * @param  {Vec3}       xj
     * @param  {Quaternion} qi
     * @param  {Quaternion} qj
     * @param  {Body}       bi
     * @param  {Body}       bj
     */
    // Narrowphase.prototype[Shape.types.SPHERE | Shape.types.PLANE] =
    bool spherePlane(
        Shapes::Sphere* si,
        Shapes::Plane* sj,
        Math::Vec3* xi,
        Math::Vec3* xj,
        Math::Quaternion* qi,
        Math::Quaternion* qj,
        Objects::Body* bi,
        Objects::Body* bj,
        Shapes::Shape* rsi,
        Shapes::Shape* rsj,
        bool justTest);

    /**
     * @method sphereBox
     * @param  {Shape}      si
     * @param  {Shape}      sj
     * @param  {Vec3}       xi
     * @param  {Vec3}       xj
     * @param  {Quaternion} qi
     * @param  {Quaternion} qj
     * @param  {Body}       bi
     * @param  {Body}       bj
     */
    // Narrowphase.prototype[Shape.types.SPHERE | Shape.types.BOX] =
    bool sphereBox(
        Shapes::Sphere* si,
        Shapes::Box* sj,
        Math::Vec3* xi,
        Math::Vec3* xj,
        Math::Quaternion* qi,
        Math::Quaternion* qj,
        Objects::Body* bi,
        Objects::Body* bj,
        Shapes::Shape* rsi,
        Shapes::Shape* rsj,
        bool justTest);

    /**
     * @method sphereConvex
     * @param  {Shape}      si
     * @param  {Shape}      sj
     * @param  {Vec3}       xi
     * @param  {Vec3}       xj
     * @param  {Quaternion} qi
     * @param  {Quaternion} qj
     * @param  {Body}       bi
     * @param  {Body}       bj
     */
    // Narrowphase.prototype[Shape.types.SPHERE | Shape.types.CONVEXPOLYHEDRON] =
    bool sphereConvex(
        Shapes::Sphere* si,
        Shapes::ConvexPolyhedron* sj,
        Math::Vec3* xi,
        Math::Vec3* xj,
        Math::Quaternion* qi,
        Math::Quaternion* qj,
        Objects::Body* bi,
        Objects::Body* bj,
        Shapes::Shape* rsi,
        Shapes::Shape* rsj,
        bool justTest);

    /**
     * @method planeBox
     * @param  {Array}      result
     * @param  {Shape}      si
     * @param  {Shape}      sj
     * @param  {Vec3}       xi
     * @param  {Vec3}       xj
     * @param  {Quaternion} qi
     * @param  {Quaternion} qj
     * @param  {Body}       bi
     * @param  {Body}       bj
     */
    // Narrowphase.prototype[Shape.types.PLANE | Shape.types.BOX] =
    bool planeBox(
        Shapes::Plane* si,
        Shapes::Box* sj,
        Math::Vec3* xi,
        Math::Vec3* xj,
        Math::Quaternion* qi,
        Math::Quaternion* qj,
        Objects::Body* bi,
        Objects::Body* bj,
        Shapes::Shape* rsi,
        Shapes::Shape* rsj,
        bool justTest);

    /**
     * @method planeConvex
     * @param  {Shape}      si
     * @param  {Shape}      sj
     * @param  {Vec3}       xi
     * @param  {Vec3}       xj
     * @param  {Quaternion} qi
     * @param  {Quaternion} qj
     * @param  {Body}       bi
     * @param  {Body}       bj
     */
    // Narrowphase.prototype[Shape.types.PLANE | Shape.types.CONVEXPOLYHEDRON] =
    bool planeConvex(
        Shapes::Sphere* planeShape,
        Shapes::ConvexPolyhedron* convexShape,
        Math::Vec3* planePosition,
        Math::Vec3* convexPosition,
        Math::Quaternion* planeQuat,
        Math::Quaternion* convexQuat,
        Objects::Body* planeBody,
        Objects::Body* convexBody,
        Shapes::Shape* si,
        Shapes::Shape* sj,
        bool justTest);

    /**
     * @method convexConvex
     * @param  {Shape}      si
     * @param  {Shape}      sj
     * @param  {Vec3}       xi
     * @param  {Vec3}       xj
     * @param  {Quaternion} qi
     * @param  {Quaternion} qj
     * @param  {Body}       bi
     * @param  {Body}       bj
     */
    // Narrowphase.prototype[Shape.types.CONVEXPOLYHEDRON] =
    bool convexConvex(
        Shapes::ConvexPolyhedron* si,
        Shapes::ConvexPolyhedron* sj,
        Math::Vec3* xi,
        Math::Vec3* xj,
        Math::Quaternion* qi,
        Math::Quaternion* qj,
        Objects::Body* bi,
        Objects::Body* bj,
        Shapes::Shape* rsi,
        Shapes::Shape* rsj,
        bool justTest,
        std::vector<int>* faceListA,
        std::vector<int>* faceListB);

    /**
     * @method convexTrimesh
     * @param  {Array}      result
     * @param  {Shape}      si
     * @param  {Shape}      sj
     * @param  {Vec3}       xi
     * @param  {Vec3}       xj
     * @param  {Quaternion} qi
     * @param  {Quaternion} qj
     * @param  {Body}       bi
     * @param  {Body}       bj
     */
    // Narrowphase.prototype[Shape.types.CONVEXPOLYHEDRON | Shape.types.TRIMESH] =
    // Narrowphase.prototype.convexTrimesh = function(si,sj,xi,xj,qi,qj,bi,bj,rsi,rsj,faceListA,faceListB){
    //     var sepAxis = convexConvex_sepAxis;

    //     if(xi.distanceTo(xj) > si.boundingSphereRadius + sj.boundingSphereRadius){
    //         return;
    //     }

    //     // Construct a temp hull for each triangle
    //     var hullB = new ConvexPolyhedron();

    //     hullB.faces = [[0,1,2]];
    //     var va = new Vec3();
    //     var vb = new Vec3();
    //     var vc = new Vec3();
    //     hullB.vertices = [
    //         va,
    //         vb,
    //         vc
    //     ];

    //     for (var i = 0; i < sj.indices.length / 3; i++) {

    //         var triangleNormal = new Vec3();
    //         sj.getNormal(i, triangleNormal);
    //         hullB.faceNormals = [triangleNormal];

    //         sj.getTriangleVertices(i, va, vb, vc);

    //         var d = si.testSepAxis(triangleNormal, hullB, xi, qi, xj, qj);
    //         if(!d){
    //             triangleNormal.scale(-1, triangleNormal);
    //             d = si.testSepAxis(triangleNormal, hullB, xi, qi, xj, qj);

    //             if(!d){
    //                 continue;
    //             }
    //         }

    //         var res = [];
    //         var q = convexConvex_q;
    //         si.clipAgainstHull(xi,qi,hullB,xj,qj,triangleNormal,-100,100,res);
    //         for(var j = 0; j !== res.length; j++){
    //             var r = this.createContactEquation(bi,bj,si,sj,rsi,rsj),
    //                 ri = r.ri,
    //                 rj = r.rj;
    //             r.ni.copy(triangleNormal);
    //             r.ni.negate(r.ni);
    //             res[j].normal.negate(q);
    //             q.mult(res[j].depth, q);
    //             res[j].point.vadd(q, ri);
    //             rj.copy(res[j].point);

    //             // Contact points are in world coordinates. Transform back to relative
    //             ri.vsub(xi,ri);
    //             rj.vsub(xj,rj);

    //             // Make relative to bodies
    //             ri.vadd(xi, ri);
    //             ri.vsub(bi.position, ri);
    //             rj.vadd(xj, rj);
    //             rj.vsub(bj.position, rj);

    //             result.push(r);
    //         }
    //     }
    // };

    /**
     * @method particlePlane
     * @param  {Array}      result
     * @param  {Shape}      si
     * @param  {Shape}      sj
     * @param  {Vec3}       xi
     * @param  {Vec3}       xj
     * @param  {Quaternion} qi
     * @param  {Quaternion} qj
     * @param  {Body}       bi
     * @param  {Body}       bj
     */
    // Narrowphase.prototype[Shape.types.PLANE | Shape.types.PARTICLE] =
    bool planeParticle(
        Shapes::Plane* sj,
        Shapes::Particle* si,
        Math::Vec3* xj,
        Math::Vec3* xi,
        Math::Quaternion* qj,
        Math::Quaternion* qi,
        Objects::Body* bj,
        Objects::Body* bi,
        Shapes::Shape* rsi,
        Shapes::Shape* rsj,
        bool justTest);

    /**
     * @method particleSphere
     * @param  {Array}      result
     * @param  {Shape}      si
     * @param  {Shape}      sj
     * @param  {Vec3}       xi
     * @param  {Vec3}       xj
     * @param  {Quaternion} qi
     * @param  {Quaternion} qj
     * @param  {Body}       bi
     * @param  {Body}       bj
     */
    // Narrowphase.prototype[Shape.types.PARTICLE | Shape.types.SPHERE] =
    bool sphereParticle(
        Shapes::Sphere* sj,
        Shapes::Particle* si,
        Math::Vec3* xj,
        Math::Vec3* xi,
        Math::Quaternion* qj,
        Math::Quaternion* qi,
        Objects::Body* bj,
        Objects::Body* bi,
        Shapes::Shape* rsi,
        Shapes::Shape* rsj,
        bool justTest);

    /**
     * @method convexParticle
     * @param  {Array}      result
     * @param  {Shape}      si
     * @param  {Shape}      sj
     * @param  {Vec3}       xi
     * @param  {Vec3}       xj
     * @param  {Quaternion} qi
     * @param  {Quaternion} qj
     * @param  {Body}       bi
     * @param  {Body}       bj
     */
    // Narrowphase.prototype[Shape.types.PARTICLE | Shape.types.CONVEXPOLYHEDRON] =
    bool convexParticle(
        Shapes::ConvexPolyhedron* sj,
        Shapes::Particle* si,
        Math::Vec3* xj,
        Math::Vec3* xi,
        Math::Quaternion* qj,
        Math::Quaternion* qi,
        Objects::Body* bj,
        Objects::Body* bi,
        Shapes::Shape* rsi,
        Shapes::Shape* rsj,
        bool justTest);

    // Narrowphase.prototype[Shape.types.BOX | Shape.types.HEIGHTFIELD] =
    bool boxHeightfield(
        Shapes::Box* sj,
        Shapes::Heightfield* si,
        Math::Vec3* xj,
        Math::Vec3* xi,
        Math::Quaternion* qj,
        Math::Quaternion* qi,
        Objects::Body* bj,
        Objects::Body* bi,
        Shapes::Shape* rsi,
        Shapes::Shape* rsj,
        bool justTest);

    /**
     * @method convexHeightfield
     */
    // Narrowphase.prototype[Shape.types.CONVEXPOLYHEDRON | Shape.types.HEIGHTFIELD] =
    bool convexHeightfield(
        Shapes::ConvexPolyhedron* convexShape,
        Shapes::Heightfield* hfShape,
        Math::Vec3* convexPos,
        Math::Vec3* hfPos,
        Math::Quaternion* convexQuat,
        Math::Quaternion* hfQuat,
        Objects::Body* convexBody,
        Objects::Body* hfBody,
        Shapes::Shape* rsi,
        Shapes::Shape* rsj,
        bool justTest);

    /**
     * @method sphereHeightfield
     */
    // Narrowphase.prototype[Shape.types.SPHERE | Shape.types.HEIGHTFIELD] =
    bool sphereHeightfield(
        Shapes::Sphere* sphereShape,
        Shapes::Heightfield* hfShape,
        Math::Vec3* spherePos,
        Math::Vec3* hfPos,
        Math::Quaternion* sphereQuat,
        Math::Quaternion* hfQuat,
        Objects::Body* sphereBody,
        Objects::Body* hfBody,
        Shapes::Shape* rsi,
        Shapes::Shape* rsj,
        bool justTest);

};

}

#endif
