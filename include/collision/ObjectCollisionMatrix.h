#ifndef ObjectCollisionMatrix_h
#define ObjectCollisionMatrix_h

#include <map>

namespace Cannon::Collision {

class ObjectCollisionMatrix {
private:
    /**
     * The matrix storage
     * @property matrix
     * @type {Object}
     */
	std::map<std::string, bool> matrix_;

public:
    /**
     * Records what objects are colliding with each other
     * @class ObjectCollisionMatrix
     * @constructor
     */
    ObjectCollisionMatrix() {};

    /**
     * @method get
     * @param  {Number} i
     * @param  {Number} j
     * @return {Boolean}
     */
    bool get(int i, int j);

    /**
     * @method set
     * @param  {Number} i
     * @param  {Number} j
     */
    void set(int i, int j);
    
    /**
     * @method set
     * @param  {Number} i
     * @param  {Number} j
     */
    void del(int i, int j);

    /**
     * Empty the matrix
     * @method reset
     */
    void reset();

    /**
     * Set max number of objects
     * @method setNumObjects
     * @param {Number} n
     */
    void setNumObjects(int n);
};

}

#endif
