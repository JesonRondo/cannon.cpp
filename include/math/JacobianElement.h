#ifndef JacobianElement_h
#define JacobianElement_h

#include "math/Vec3.h"

namespace Cannon::Math {

class JacobianElement {
public:
    /**
     * @property {Vec3} spatial
     */
    Vec3 spatial;

    /**
     * @property {Vec3} rotational
     */
    Vec3 rotational;

    /**
     * Multiply with other JacobianElement
     * @method multiplyElement
     * @param  {JacobianElement} element
     * @return {Number}
     */
    double multiplyElement(JacobianElement element);

    /**
     * Multiply with two vectors
     * @method multiplyVectors
     * @param  {Vec3} spatial
     * @param  {Vec3} rotational
     * @return {Number}
     */
    double multiplyVectors(Vec3 spatial, Vec3 rotational);
};

}

#endif
