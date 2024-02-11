#ifndef Transform_h
#define Transform_h

#include "math/Vec3.h"
#include "math/Quaternion.h"

namespace Cannon::Math {

class Transform {
private:
    Vec3 position_;
    Quaternion quaternion_;

public:
    /**
     * @static
     * @method pointToLocaFrame
     * @param {Vec3} position
     * @param {Quaternion} quaternion
     * @param {Vec3} worldPoint
     * @param {Vec3} result
     */
    static Vec3 pointToLocalFrame(Vec3 position, Quaternion quaternion, Vec3 worldPoint, Vec3 result);

    /**
     * @static
     * @method pointToWorldFrame
     * @param {Vec3} position
     * @param {Vec3} quaternion
     * @param {Vec3} localPoint
     * @param {Vec3} result
     */
    static Vec3 pointToWorldFrame(Vec3 position, Vec3 quaternion, Vec3 localPoint, Vec3 result);

    static Vec3 vectorToWorldFrame(Vec3 quaternion, Vec3 localVector, Vec3 result);

    static Vec3 vectorToLocalFrame(Vec3 position, Vec3 quaternion, Vec3 worldVector, Vec3 result);

    /**
     * @class Transform
     * @constructor
     */
    Transform();

    /**
     * @class Transform
     * @constructor
     */
    Transform(Vec3 position, Quaternion quaternion){
        position_.copy(position);
        quaternion_.copy(quaternion); };

    /**
     * Get a global point in local transform coordinates.
     * @method pointToLocal
     * @param  {Vec3} point
     * @param  {Vec3} result
     * @return {Vec3} The "result" vector object
     */
    Vec3 pointToLocal(Vec3 worldPoint, Vec3 result);

    /**
     * Get a local point in global transform coordinates.
     * @method pointToWorld
     * @param  {Vec3} point
     * @param  {Vec3} result
     * @return {Vec3} The "result" vector object
     */
    Vec3 pointToWorld(Vec3 localPoint, Vec3 result);

    Vec3 vectorToWorldFrame(Vec3 localVector, Vec3 result);
};

}

#endif
