#ifndef EventTarget_h
#define Body_h

#include <map>

namespace Cannon::Utils {

template <typename T>
class TupleDictionary {
private:
    std::map<std::string, T> data_;
    std::vector<std::string> dataKeys_;

public:
    /**
     * @class TupleDictionary
     * @constructor
     */
    TupleDictionary() {};

    /**
     * @method get
     * @param  {Number} i
     * @param  {Number} j
     * @return {Object}
     */
    T get(int i, int j);

    /**
     * @method set
     * @param  {Number} i
     * @param  {Number} j
     * @param {Object} value
     */
    void set(int i, int j, T value);

    /**
     * @method del
     * @param  {Number} i
     * @param  {Number} j
     * @returns {Boolean} is remove
     */
    bool del(int i, int j);

    /**
     * @method reset
     */
    void reset();

    /**
     * @method getLength
     */
    int getLength();

    /**
     * @method getKeyByIndex
     * @param {Number} index
     */
    std::string getKeyByIndex(int index);

    /**
     * @method getDataByKey
     * @param {String} Key
     */
    T getDataByKey(std::string Key);
};

} // namespace Cannon::Utils

#endif
