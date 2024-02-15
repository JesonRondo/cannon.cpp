#ifndef Quaternion_h
#define Quaternion_h

namespace Cannon::Math {

class Vec3;

enum EulerOrder {
    XYZ,
    YXZ,
    ZXY,
    ZYX,
    YZX,
    XZY
};

class Quaternion {
private:

public:
    float x;
    float y;
    float z;
    float w;

    /**
     * A Quaternion describes a rotation in 3D space. The Quaternion is mathematically defined as Q = x*i + y*j + z*k + w, where (i,j,k) are imaginary basis vectors. (x,y,z) can be seen as a vector related to the axis of rotation, while the real multiplier, w, is related to the amount of rotation.
     * @class Quaternion
     * @constructor
     * @see http://en.wikipedia.org/wiki/Quaternion
     */
    Quaternion(){ x = 0.0; y = 0.0; z = 0.0; w = 0.0; };

    /**
     * A Quaternion describes a rotation in 3D space. The Quaternion is mathematically defined as Q = x*i + y*j + z*k + w, where (i,j,k) are imaginary basis vectors. (x,y,z) can be seen as a vector related to the axis of rotation, while the real multiplier, w, is related to the amount of rotation.
     * @class Quaternion
     * @constructor
     * @param {Number} x Multiplier of the imaginary basis vector i.
     * @param {Number} y Multiplier of the imaginary basis vector j.
     * @param {Number} z Multiplier of the imaginary basis vector k.
     * @param {Number} w Multiplier of the real part.
     * @see http://en.wikipedia.org/wiki/Quaternion
     */
    Quaternion(float x, float y, float z, float w): x(x), y(y), z(z), w(w) {};

    /**
     * Set the value of the quaternion.
     * @method set
     * @param {Number} x
     * @param {Number} y
     * @param {Number} z
     * @param {Number} w
     */
    Quaternion set(float x, float y, float z, float w);

    /**
     * Convert to a readable format
     * @method toString
     * @return string
     */
    char* toString();

    /**
     * Convert to an Array
     * @method toArray
     * @return Array
     */
    float* toArray();

    /**
     * Set the quaternion components given an axis and an angle.
     * @method setFromAxisAngle
     * @param {Vec3} axis
     * @param {Number} angle in radians
     */
    Quaternion setFromAxisAngle(Vec3 axis, float angle);

    /**
     * Converts the quaternion to axis/angle representation.
     * @method toAxisAngle
     * @param {Vec3} [targetAxis] A vector object to reuse for storing the axis.
     * @return {Array} An array, first elemnt is the axis and the second is the angle in radians.
     */
    float* toAxisAngle(Vec3 targetAxis);

    /**
     * Set the quaternion value given two vectors. The resulting rotation will be the needed rotation to rotate u to v.
     * @method setFromVectors
     * @param {Vec3} u
     * @param {Vec3} v
     */
    Quaternion setFromVectors(Vec3 u, Vec3 v);

    /**
     * Quaternion multiplication
     * @method mult
     * @param {Quaternion} q
     * @param {Quaternion} target Optional.
     * @return {Quaternion}
     */
    Quaternion mult(Quaternion q, Quaternion target);

    /**
     * Get the inverse quaternion rotation.
     * @method inverse
     * @param {Quaternion} target
     * @return {Quaternion}
     */
    Quaternion inverse(Quaternion target);

    /**
     * Get the quaternion conjugate
     * @method conjugate
     * @param {Quaternion} target
     * @return {Quaternion}
     */
    Quaternion conjugate(Quaternion target);

    /**
     * Normalize the quaternion. Note that this changes the values of the quaternion.
     * @method normalize
     */
    Quaternion normalize();

    /**
     * Approximation of quaternion normalization. Works best when quat is already almost-normalized.
     * @method normalizeFast
     * @see http://jsperf.com/fast-quaternion-normalization
     * @author unphased, https://github.com/unphased
     */
    Quaternion normalizeFast();

    /**
     * Multiply the quaternion by a vector
     * @method vmult
     * @param {Vec3} v
     * @param {Vec3} target Optional
     * @return {Vec3}
     */
    Vec3 vmult(Vec3 v, Vec3 target);

    /**
     * Copies value of source to this quaternion.
     * @method copy
     * @param {Quaternion} source
     * @return {Quaternion} this
     */
    Quaternion copy(Quaternion source);


    /**
     * Convert the quaternion to euler angle representation. Order: YZX, as this page describes: http://www.euclideanspace.com/maths/standards/index.htm
     * @method toEuler
     * @param {Vec3} target. "YZX" is default.
     */
    Quaternion toEuler(Vec3 target);

    /**
     * Convert the quaternion to euler angle representation. Order: YZX, as this page describes: http://www.euclideanspace.com/maths/standards/index.htm
     * @method toEuler
     * @param {Vec3} target
     * @param {EulerOrder} order.
     */
    void toEuler(Vec3 target, EulerOrder order);;

    /**
     * See http://www.mathworks.com/matlabcentral/fileexchange/20696-function-to-convert-between-dcm-euler-angles-quaternions-and-euler-vectors/content/SpinCalc.m
     * @method setFromEuler
     * @param {Number} x
     * @param {Number} y
     * @param {Number} z
     * @param {EulerOrder} order The order to apply angles: 'XYZ' or 'YXZ' or any other combination
     */
    Quaternion setFromEuler(float x, float y, float z, EulerOrder order);

    /**
     * @method clone
     * @return {Quaternion}
     */
    Quaternion clone();

    /**
     * Performs a spherical linear interpolation between two quat
     *
     * @method slerp
     * @param {Quaternion} toQuat second operand
     * @param {Number} t interpolation amount between the self quaternion and toQuat
     * @param {Quaternion} [target] A quaternion to store the result in. If not provided, a new one will be created.
     * @returns {Quaternion} The "target" object
     */
    Quaternion slerp(Quaternion toQuat, float t, Quaternion target);

    /**
     * Rotate an absolute orientation quaternion given an angular velocity and a time step.
     * @param  {Vec3} angularVelocity
     * @param  {number} dt
     * @param  {Vec3} angularFactor
     * @param  {Quaternion} target
     * @return {Quaternion} The "target" object
     */
    Quaternion integrate(Vec3 angularVelocity, float dt, Vec3 angularFactor, Quaternion target);
};

}

#endif
