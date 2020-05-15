#include "gesture_group.hpp"

using namespace gestures;

GestureGroup::GestureGroup(std::string name, std::list<GestureItem> gestures) : name(name), gestures(gestures) {
}