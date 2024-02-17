#ifndef Vec3Pool_h
#define Vec3Pool_h

#include "utils/Pool.h"
#include "math/Vec3.h"

namespace Cannon::Utils {

class Vec3Pool : public Pool<Math::Vec3> {
public:
    /**
     * @class Vec3Pool
     * @constructor
     * @extends Pool
     */
    Vec3Pool();

    ~Vec3Pool();

    /**
     * Construct a vector
     * @method constructObject
     * @return {Vec3}
     */
    Math::Vec3 constructObject();
};

}

#endif
