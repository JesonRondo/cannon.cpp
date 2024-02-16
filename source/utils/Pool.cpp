#include "utils/Pool.h"

using namespace Cannon::Utils;

template <typename T>
Pool<T>* Pool<T>::release(std::vector<T> obj) {
    objects_.push_back(obj);
    return this;
}

template <typename T>
T Pool<T>::get() {
    if (objects_.size() == 0) {
        objects_.push_back(constructObject());
    }

    T obj = objects_.back();
    objects_.pop_back();
    return obj;
}

template <typename T>
Pool<T>* Pool<T>::resize(int size) {    
    while (objects_.size() > size) {
        objects_.pop_back();
    }

    while (objects_.size() < size) {
        objects_.push_back(constructObject());
    }

    return this;
}
