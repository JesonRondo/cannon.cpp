#include "objects/Body.h"

using namespace Cannon;

int Objects::Body::idCounter = 0;

/**
 * Dispatched after two bodies collide. This event is dispatched on each
 * of the two bodies involved in the collision.
 * @event collide
 * @param {Body} body The body that was involved in the collision.
 * @param {ContactEquation} contact The details of the collision.
 */
const std::string Objects::Body::COLLIDE_EVENT_NAME = "collide";

const Utils::Event wakeupEvent("wakeup");

const Utils::Event sleepyEvent("sleepy");

const Utils::Event sleepEvent("sleep");
