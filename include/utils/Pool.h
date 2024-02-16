#ifndef Pool_h
#define Pool_h

#include <vector>

namespace Cannon::Utils {

template <typename T>
class Pool {
private:
    /**
     * The pooled objects
     * @property {Array} objects
     */
    std::vector<T> objects_;

public:
    /**
     * For pooling objects that can be reused.
     * @class Pool
     * @constructor
     */
    Pool() {};

    /**
     * Release an object after use
     * @method release
     * @param {Object} obj
     */
    Pool* release(std::vector<T> obj);

    /**
     * Get an object
     * @method get
     * @return {mixed}
     */
    T get();

    /**
     * Construct an object. Should be implmented in each subclass.
     * @method constructObject
     * @return {mixed}
     */
    virtual T constructObject() = 0;

    /**
     * @method resize
     * @param {number} size
     * @return {Pool} Self, for chaining
     */
    Pool* resize(int size);
};

}

#endif
