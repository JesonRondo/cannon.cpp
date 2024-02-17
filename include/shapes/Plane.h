#ifndef Plane_h
#define Plane_h

#include "shapes/Shape.h"
#include "math/Quaternion.h"

#ifndef MAX_FLOAT
#define MAX_FLOAT 3.40282e+38
#endif

namespace Cannon::Shapes {

class Plane : public Shape {
public:
    // World oriented normal
    Math::Vec3 worldNormal;
    bool worldNormalNeedsUpdate = true;

    /**
     * A plane, facing in the Z direction. The plane has its surface at z=0 and everything below z=0 is assumed to be solid plane. To make the plane face in some other direction than z, you must put it inside a Body and rotate that body. See the demos.
     * @class Plane
     * @constructor
     * @extends Shape
     * @author schteppe
     */
    Plane();

    ~Plane();

    void computeWorldNormal(Math::Quaternion* quat);

    void calculateLocalInertia(float mass, Math::Vec3* target);

    double volume();

    void calculateWorldAABB(
        Math::Vec3* pos,
        Math::Quaternion* quat,
        Math::Vec3* min,
        Math::Vec3* max);

    void updateBoundingSphereRadius();
};

}

#endif
