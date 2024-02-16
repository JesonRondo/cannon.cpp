#ifndef EventTarget_h
#define Body_h

#include <map>
#include <string>
#include <vector>

namespace Cannon::Utils {

struct Event {
    std::string type;
    EventTarget* target;

    Event(std::string type) : type(type), target(nullptr) {}
    Event(std::string type, EventTarget* target) : type(type), target(target) {}
};

typedef void (*EventListener)(Event event);

class EventTarget {
private:
    std::map<std::string, std::vector<EventListener>> listeners_;

public:
    /**
     * Base class for objects that dispatches events.
     * @class EventTarget
     * @constructor
     */
    EventTarget() {};

    /**
     * Add an event listener
     * @method addEventListener
     * @param  {String} type
     * @param  {Function} listener
     * @return {EventTarget} The self object, for chainability.
     */
    EventTarget* addEventListener(std::string type, EventListener listener);

    /**
     * Check if an event listener is added
     * @method hasEventListener
     * @param  {String} type
     * @param  {Function} listener
     * @return {Boolean}
     */
    bool hasEventListener(std::string type, EventListener listener);

    /**
     * Check if any event listener of the given type is added
     * @method hasAnyEventListener
     * @param  {String} type
     * @return {Boolean}
     */
    bool hasAnyEventListener(std::string type);

    /**
     * Remove an event listener
     * @method removeEventListener
     * @param  {String} type
     * @param  {Function} listener
     * @return {EventTarget} The self object, for chainability.
     */
    EventTarget* removeEventListener(std::string type, EventListener listener);

    /**
     * Emit an event.
     * @method dispatchEvent
     * @param  {Object} event
     * @param  {String} event.type
     * @return {EventTarget} The self object, for chainability.
     */
    EventTarget* dispatchEvent(Event event);
};

} // namespace Cannon::Utils

#endif
