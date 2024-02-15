#ifndef Vec3_h
#define Vec3_h

#include <string>

namespace Cannon::Math {

class Mat3;

class Vec3 {
private:

public:
    /**
     * @static
     * @property {Vec3} ZERO
     */
    static const Vec3 ZERO;

    /**
     * @static
     * @property {Vec3} UNIT_X
     */
    static const Vec3 UNIT_X;

    /**
     * @static
     * @property {Vec3} UNIT_Y
     */
    static const Vec3 UNIT_Y;

    /**
     * @static
     * @property {Vec3} UNIT_Z
     */
    static const Vec3 UNIT_Z;
    
    float x;
    float y;
    float z;

    /**
     * 3-dimensional vector
     * @class Vec3
     * @constructor
     */
    Vec3(){ x = 0.0; y = 0.0; z = 0.0; };

    /**
     * 3-dimensional vector
     * @class Vec3
     * @constructor
     * @param {Number} x
     * @param {Number} y
     * @param {Number} z
     * @author schteppe
     * @example
     *     var v = new Vec3(1, 2, 3);
     *     console.log('x=' + v.x); // x=1
     */
    Vec3(float x, float y, float z): x(x), y(y), z(z) {};

    /**
     * Vector cross product
     * @method cross
     * @param {Vec3} v
     * @param {Vec3} target Optional. Target to save in.
     * @return {Vec3}
     */
    Vec3 cross(Vec3 v, Vec3 target);

    /**
     * Set the vectors' 3 elements
     * @method set
     * @param {Number} x
     * @param {Number} y
     * @param {Number} z
     * @return Vec3
     */
    Vec3 set(float x, float y, float z);

    /**
     * Set all components of the vector to zero.
     * @method setZero
     */
    void setZero();

    /**
     * Vector addition
     * @method vadd
     * @param {Vec3} v
     * @param {Vec3} target Optional.
     * @return {Vec3}
     */
    Vec3 vadd(Vec3 v, Vec3 target);

    /**
     * Vector subtraction
     * @method vsub
     * @param {Vec3} v
     * @param {Vec3} target Optional. Target to save in.
     * @return {Vec3}
     */
    Vec3 vsub(Vec3 v, Vec3 target);

    /**
     * Get the cross product matrix a_cross from a vector, such that a x b = a_cross * b = c
     * @method crossmat
     * @see http://www8.cs.umu.se/kurser/TDBD24/VT06/lectures/Lecture6.pdf
     * @return {Mat3}
     */
    Mat3 crossmat();

    /**
     * Normalize the vector. Note that this changes the values in the vector.
     * @method normalize
     * @return {Number} Returns the norm of the vector
     */
    float normalize();

    /**
     * Get the version of this vector that is of length 1.
     * @method unit
     * @param {Vec3} target Optional target to save in
     * @return {Vec3} Returns the unit vector
     */
    Vec3 unit(Vec3 target);

    /**
     * Get the length of the vector
     * @method length
     * @return {Number}
     */
    float length();

    /**
     * Get the squared length of the vector.
     * @method lengthSquared
     * @return {Number}
     */
    float lengthSquared();

    /**
     * Get distance from this point to another point
     * @method distanceTo
     * @param  {Vec3} p
     * @return {Number}
     */
    float distanceTo(Vec3 p);

    /**
     * Get squared distance from this point to another point
     * @method distanceSquared
     * @param  {Vec3} p
     * @return {Number}
     */
    float distanceSquared(Vec3 p);

    /**
     * Multiply the vector with an other vector, component-wise.
     * @method mult
     * @param {Number} vector
     * @param {Vec3} target The vector to save the result in.
     * @return {Vec3}
     */
    Vec3 vmul(float vector, Vec3 target);

    /**
     * Multiply the vector with a scalar.
     * @method scale
     * @param {Number} scalar
     * @param {Vec3} target
     * @return {Vec3}
     */
    Vec3 scale(float scalar, Vec3 target);

    /**
     * Scale a vector and add it to this vector. Save the result in "target". (target = this + vector * scalar)
     * @method addScaledVector
     * @param {Number} scalar
     * @param {Vec3} vector
     * @param {Vec3} target The vector to save the result in.
     * @return {Vec3}
     */
    Vec3 addScaledVector(float scalar, Vec3 vector, Vec3 target);

    /**
     * Calculate dot product
     * @method dot
     * @param {Vec3} v
     * @return {Number}
     */
    float dot(Vec3 v);

    /**
     * @method isZero
     * @return bool
     */
    bool isZero();

    /**
     * Make the vector point in the opposite direction.
     * @method negate
     * @param {Vec3} target Optional target to save in
     * @return {Vec3}
     */
    Vec3 negate(Vec3 target);

    /**
     * Compute two artificial tangents to the vector
     * @method tangents
     * @param {Vec3} t1 Vector object to save the first tangent in
     * @param {Vec3} t2 Vector object to save the second tangent in
     */
    void tangents(Vec3 t1, Vec3 t2);

    /**
     * Converts to a more readable format
     * @method toString
     * @return string
     */
    std::string toString();

    /**
     * Converts to an array
     * @method toArray
     * @return Array
     */
    float* toArray();

    /**
     * Copies value of source to this vector.
     * @method copy
     * @param {Vec3} source
     * @return {Vec3} this
     */
    Vec3 copy(Vec3 source);


    /**
     * Do a linear interpolation between two vectors
     * @method lerp
     * @param {Vec3} v
     * @param {Number} t A number between 0 and 1. 0 will make this function return u, and 1 will make it return v. Numbers in between will generate a vector in between them.
     * @param {Vec3} target
     */
    void lerp(Vec3 v, float t, Vec3 target);

    /**
     * Check if a vector equals is almost equal to another one.
     * @method almostEquals
     * @param {Vec3} v
     * @param {Number} precision
     * @return bool
     */
    bool almostEquals(Vec3 v, float precision);

    /**
     * Check if a vector is almost zero
     * @method almostZero
     * @param {Number} precision
     */
    bool almostZero(float precision);

    /**
     * Check if the vector is anti-parallel to another vector.
     * @method isAntiparallelTo
     * @param  {Vec3}  v
     * @param  {Number}  precision Set to zero for exact comparisons
     * @return {Boolean}
     */
    bool isAntiparallelTo(Vec3 v, float precision);

    /**
     * Clone the vector
     * @method clone
     * @return {Vec3}
     */
    Vec3 clone();
};

}

#endif
