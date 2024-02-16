#include "utils/EventTarget.h"

#include <algorithm>

using namespace Cannon::Utils;

EventTarget* EventTarget::addEventListener(std::string type, EventListener listener) {
    if (listeners_.find(type) == listeners_.end()) {
        listeners_[type] = std::vector<EventListener>();
    }

    if (std::find(listeners_[type].begin(), listeners_[type].end(), listener) == listeners_[type].end()) {
        listeners_[type].push_back(listener);
    }

    return this;
}

bool EventTarget::hasEventListener(std::string type, EventListener listener) {
    if (listeners_.find(type) == listeners_.end()) {
        return false;
    }

    return std::find(listeners_[type].begin(), listeners_[type].end(), listener) != listeners_[type].end();
}

bool EventTarget::hasAnyEventListener(std::string type) {
    return listeners_.find(type) != listeners_.end();
}

EventTarget* EventTarget::removeEventListener(std::string type, EventListener listener) {
    if (listeners_.find(type) == listeners_.end()) {
        return this;
    }

    auto it = std::find(listeners_[type].begin(), listeners_[type].end(), listener);
    if (it != listeners_[type].end()) {
        listeners_[type].erase(it);
    }

    return this;
}

EventTarget* EventTarget::dispatchEvent(Event event) {
    if (listeners_.find(event.type) == listeners_.end()) {
        return this;
    }

    event.target = this;
    for (auto listener : listeners_[event.type]) {
        listener(event);
    }

    return this;
}
