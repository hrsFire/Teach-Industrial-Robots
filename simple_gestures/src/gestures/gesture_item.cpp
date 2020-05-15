#include "gesture_item.hpp"

using namespace gestures;

GestureItem::GestureItem(Gesture gesture, std::vector<std::string> affectedItems, bool preventConflicts) :
        gesture(gesture), affectedItems(affectedItems), preventConflicts(preventConflicts) {
}