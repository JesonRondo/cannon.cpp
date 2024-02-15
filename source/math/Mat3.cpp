#include <vector>

#include "math/Mat3.h"
#include "math/Vec3.h"
#include "math/Quaternion.h"

using namespace Cannon::Math;

void Mat3::identity() {
    auto e = elements;
    e[0] = 1;
    e[1] = 0;
    e[2] = 0;

    e[3] = 0;
    e[4] = 1;
    e[5] = 0;

    e[6] = 0;
    e[7] = 0;
    e[8] = 1;
};

void Mat3::setZero() {
    auto e = elements;
    e[0] = 0;
    e[1] = 0;
    e[2] = 0;
    e[3] = 0;
    e[4] = 0;
    e[5] = 0;
    e[6] = 0;
    e[7] = 0;
    e[8] = 0;
};

void Mat3::setTrace(Vec3* vec3) {
    auto e = elements;
    e[0] = vec3->x;
    e[4] = vec3->y;
    e[8] = vec3->z;
};

Vec3* Mat3::getTrace(Vec3* target) {
    auto e = elements;
    target->x = e[0];
    target->y = e[4];
    target->z = e[8];

    return target;
};

Vec3* Mat3::vmult(Vec3* v, Vec3* target) {
    auto e = elements;
    auto x = v->x;
    auto y = v->y;
    auto z = v->z;
    target->x = e[0]*x + e[1]*y + e[2]*z;
    target->y = e[3]*x + e[4]*y + e[5]*z;
    target->z = e[6]*x + e[7]*y + e[8]*z;

    return target;
};

void Mat3::smult(float s) {
    for (int i = 0; i < elements.size(); i++) {
        elements[i] *= s;
    }
};

Mat3* Mat3::mmult(Mat3* m, Mat3* target) {
    for (int i = 0; i < 3; i++) {
        for(int j = 0; j < 3; j++) {
            float sum = 0.0;
            for(int k = 0; k < 3; k++) {
                sum += m->e(k, i) * this->e(j, k);
            }
            target->setE(j, i, sum);
        }
    }
    return target;
};

Mat3* Mat3::scale(Vec3* v, Mat3* target) {
    for(auto i = 0; i != 3; i++) {
        target->setE(i, 0, v->x * this->e(i, 0));
        target->setE(i, 1, v->y * this->e(i, 1));
        target->setE(i, 2, v->z * this->e(i, 2));
    }
    return target;
};

// Vec3* Mat3::solve(Vec3* b, Vec3* target) {
//     // Construct equations
//     int nr = 3; // num rows
//     int nc = 4; // num cols
//     std::vector<float> eqns;

//     for (int i = 0; i < nr * nc; i++) {
//         eqns.push_back(0);
//     }

//     for (int i = 0; i < 3; i++) {
//         for (int j = 0; j < 3; j++) {
//             eqns[i + nc * j] = this->e(j, i);
//         }
//     }

//     eqns[3 + 4 * 0] = b->x;
//     eqns[3 + 4 * 1] = b->y;
//     eqns[3 + 4 * 2] = b->z;

//     // Compute right upper triangular version of the matrix - Gauss elimination
//     int n = 3, k = n, np;
//     int kp = 4; // num rows
//     int p;
//     do {
//         int i = k - n;
//         if (eqns[i + nc * i] == 0) {
//             // the pivot is null, swap lines
//             for (int j = i + 1; j < k; j++) {
//                 if (eqns[i + nc * j] != 0) {
//                     np = kp;
//                     do { // do ligne( i ) = ligne( i ) + ligne( k )
//                         p = kp - np;
//                         eqns[p + nc * i] += eqns[p + nc * j];
//                     } while (--np);
//                     break;
//                 }
//             }
//         }
//         if (eqns[i + nc * i] != 0) {
//             for (int j = i + 1; j < k; j++) {
//                 float multiplier = eqns[i + nc * j] / eqns[i + nc * i];
//                 np = kp;
//                 do {  // do ligne( k ) = ligne( k ) - multiplier * ligne( i )
//                     p = kp - np;
//                     eqns[p + nc * j] = p <= i ? 0 : eqns[p + nc * j] - eqns[p + nc * i] * multiplier;
//                 } while (--np);
//             }
//         }
//     } while (--n);

//     if (eqns[2 * nc + 2] == 0 || eqns[1 * nc + 1] == 0 || eqns[0 * nc + 0] == 0) {
//         throw "Could not solve equation! Got x=[" + target->toString() + "], b=[" + b->toString() + "], A=[" + this->toString() + "]";
//     }

//     // Get the solution
//     target->z = eqns[2 * nc + 3] / eqns[2 * nc + 2];
//     target->y = (eqns[1 * nc + 3] - eqns[1 * nc + 2] * target->z) / eqns[1 * nc + 1];
//     target->x = (eqns[0 * nc + 3] - eqns[0 * nc + 2] * target->z - eqns[0 * nc + 1] * target->y) / eqns[0 * nc + 0];

//     return target;
// };

float Mat3::e(int row, int column) {
    return elements[column + 3 * row];
};

void Mat3::setE(int row, int column, float value) {
    elements[column + 3 * row] = value;
};

Mat3* Mat3::copy(Mat3* source) {
    for (int i = 0; i < source->elements.size(); i++) {
        elements[i] = source->elements[i];
    }
    return this;
};

std::string Mat3::toString() {
    std::string r = "";
    std::string sep = ",";
    for (int i = 0; i < 9; i++) {
        r += std::to_string(elements[i]) + sep;
    }
    return r;
};

Mat3* Mat3::reverse(Mat3* target) {
    // Construct equations
    int nr = 3; // num rows
    int nc = 6; // num cols
    std::vector<float> eqns;

    for(int i = 0; i < nr * nc; i++) {
        eqns.push_back(0);
    }

    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++){
            eqns[i + nc * j] = elements[i + 3 * j];
        }
    }

    eqns[3 + 6 * 0] = 1;
    eqns[3 + 6 * 1] = 0;
    eqns[3 + 6 * 2] = 0;
    eqns[4 + 6 * 0] = 0;
    eqns[4 + 6 * 1] = 1;
    eqns[4 + 6 * 2] = 0;
    eqns[5 + 6 * 0] = 0;
    eqns[5 + 6 * 1] = 0;
    eqns[5 + 6 * 2] = 1;

    // Compute right upper triangular version of the matrix - Gauss elimination
    int n = 3, k = n, np;
    int kp = nc; // num rows
    do {
        int i = k - n;
        if (eqns[i + nc * i] == 0) {
            // the pivot is null, swap lines
            for (int j = i + 1; j < k; j++) {
                if (eqns[i + nc * j] != 0) {
                    np = kp;
                    do { // do line( i ) = line( i ) + line( k )
                        int p = kp - np;
                        eqns[p + nc * i] += eqns[p + nc * j];
                    } while (--np);
                    break;
                }
            }
        }
        if (eqns[i + nc * i] != 0) {
            for (int j = i + 1; j < k; j++) {
                if (eqns[i + nc * i] == 0) {
                    throw "Could not reverse! A=[" + this->toString() + "]";
                }
                float multiplier = eqns[i + nc * j] / eqns[i + nc * i];
                np = kp;
                do { // do line( k ) = line( k ) - multiplier * line( i )
                    int p = kp - np;
                    eqns[p + nc * j] = p <= i ? 0 : eqns[p + nc * j] - eqns[p + nc * i] * multiplier;
                } while (--np);
            }
        }
    } while (--n);

    // eliminate the upper left triangle of the matrix
    int i = 2;
    do {
        int j = i-1;
        do {
            if (eqns[i + nc * i] == 0) {
                throw "Could not reverse! A=[" + this->toString() + "]";
            }

            float multiplier = eqns[i + nc * j] / eqns[i + nc * i];
            np = nc;
            do {
                int p = nc - np;
                eqns[p + nc * j] = eqns[p + nc * j] - eqns[p + nc * i] * multiplier;
            } while (--np);
        } while (j--);
    } while (--i);

    // operations on the diagonal
    i = 2;
    do {
        if (eqns[i + nc * i] == 0) {
            throw "Could not reverse! A=[" + this->toString() + "]";
        }

        float multiplier = 1 / eqns[i + nc * i];
        np = nc;
        do {
            int p = nc - np;
            eqns[p + nc * i] = eqns[p + nc * i] * multiplier;
        } while (--np);
    } while (i--);

    i = 2;
    do {
        int j = 2;
        do {
            float p = eqns[nr + j + nc * i];
            target->setE(i, j, p);
        } while (j--);
    } while (i--);

    return target;
};

Mat3* Mat3::setRotationFromQuaternion(Quaternion* q) {
    float x = q->x, y = q->y, z = q->z, w = q->w,
        x2 = x + x, y2 = y + y, z2 = z + z,
        xx = x * x2, xy = x * y2, xz = x * z2,
        yy = y * y2, yz = y * z2, zz = z * z2,
        wx = w * x2, wy = w * y2, wz = w * z2;
    auto e = elements;

    e[3 * 0 + 0] = 1 - (yy + zz);
    e[3 * 0 + 1] = xy - wz;
    e[3 * 0 + 2] = xz + wy;

    e[3 * 1 + 0] = xy + wz;
    e[3 * 1 + 1] = 1 - (xx + zz);
    e[3 * 1 + 2] = yz - wx;

    e[3 * 2 + 0] = xz - wy;
    e[3 * 2 + 1] = yz + wx;
    e[3 * 2 + 2] = 1 - (xx + yy);

    return this;
};

Mat3* Mat3::transpose(Mat3* target) {
    auto Mt = target->elements;
    auto M = elements;

    for (auto i = 0; i != 3; i++) {
        for (auto j = 0; j != 3; j++) {
            Mt[3 * i + j] = M[3 * j + i];
        }
    }

    return target;
};
