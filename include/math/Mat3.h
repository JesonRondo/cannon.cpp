#ifndef Mat3_h
#define Mat3_h

namespace Cannon::Math {

class Vec3;
class Quaternion;

class Mat3 {
private:
    float elements_[9]; // array of nine elements

public:
    /**
     * A 3x3 matrix.
     * @class Mat3
     * @constructor
     */
    Mat3(): elements_{ 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f } {};

    /**
     * A 3x3 matrix.
     * @class Mat3
     * @constructor
     * @param array elements Array of nine elements. Optional.
     */
    Mat3(float elements[9]): elements_{
        elements[0], elements[1], elements[2],
        elements[3], elements[4], elements[5],
        elements[6], elements[7], elements[8] } {};

    /**
     * Sets the matrix to identity
     * @method identity
     * @todo Should perhaps be renamed to setIdentity() to be more clear.
     * @todo Create another function that immediately creates an identity matrix eg. eye()
     */
    void identity();

    /**
     * Set all elements to zero
     * @method setZero
     */
    void setZero();

    /**
     * Sets the matrix diagonal elements from a Vec3
     * @method setTrace
     * @param {Vec3} vec3
     */
    void setTrace(Vec3 vec3);

    /**
     * Gets the matrix diagonal elements
     * @method getTrace
     * @return {Vec3}
     */
    Vec3 getTrace(Vec3 target);

    /**
     * Matrix-Vector multiplication
     * @method vmult
     * @param {Vec3} v The vector to multiply with
     * @param {Vec3} target Optional, target to save the result in.
     */
    Vec3 vmult(Vec3 v, Vec3 target);

    /**
     * Matrix-scalar multiplication
     * @method smult
     * @param {Number} s
     */
    void smult(float s);

    /**
     * Matrix multiplication
     * @method mmult
     * @param {Mat3} m Matrix to multiply with from left side.
     * @return {Mat3} The result.
     */
    Mat3 mmult(Mat3 m, Mat3 target);

    /**
     * Scale each column of the matrix
     * @method scale
     * @param {Vec3} v
     * @return {Mat3} The result.
     */
    Mat3 scale(Vec3 v, Mat3 target);

    /**
     * Solve Ax=b
     * @method solve
     * @param {Vec3} b The right hand side
     * @param {Vec3} target Optional. Target vector to save in.
     * @return {Vec3} The solution x
     * @todo should reuse arrays
     */
    Vec3 solve(Vec3 b, Vec3 target);

    /**
     * Get an element in the matrix by index. Index starts at 0, not 1!!!
     * @method e
     * @param {Number} row
     * @param {Number} column
     * @return {Number}
     */
    float e(float row, float column);
    
    /**
     * Get an element in the matrix by index. Index starts at 0, not 1!!!
     * @method setE
     * @param {Number} row
     * @param {Number} column
     * @param {Number} value, the matrix element will be set to this value.
     * @return {Number}
     */
    float setE(float row, float column, float value);

    /**
     * Copy another matrix into this matrix object.
     * @method copy
     * @param {Mat3} source
     * @return {Mat3} this
     */
    Mat3 copy(Mat3 source);

    /**
     * Returns a string representation of the matrix.
     * @method toString
     * @return string
     */
    char* toString();

    /**
     * reverse the matrix
     * @method reverse
     * @param {Mat3} target Optional. Target matrix to save in.
     * @return {Mat3} The solution x
     */
    Mat3 reverse(Mat3 target);

    /**
     * Set the matrix from a quaterion
     * @method setRotationFromQuaternion
     * @param {Quaternion} q
     */
    Mat3 setRotationFromQuaternion(Quaternion q);

    /**
     * Transpose the matrix
     * @method transpose
     * @param  {Mat3} target Where to store the result.
     * @return {Mat3} The target Mat3, or a new Mat3 if target was omitted.
     */
    Mat3 transpose(Mat3 target);
};

}

#endif
