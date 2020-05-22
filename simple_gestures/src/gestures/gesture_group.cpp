#include "simple_gestures/gestures/gesture_group.hpp"

using namespace gestures;

GestureGroup::GestureGroup(std::string name) : name(name) {
}

GestureGroup::operator std::string() const {
    return name;
}

bool GestureGroup::operator==(const GestureGroup& gestureGroup) const {
    return name == gestureGroup.name;
}