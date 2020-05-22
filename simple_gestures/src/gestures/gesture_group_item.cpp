#include "gesture_group_item.hpp"

using namespace gestures;

GestureGroupItem::GestureGroupItem(std::string name, std::list<GestureItem> gestures, std::list<GestureGroup> excludedGroups) :
        name(name), gestures(gestures), excludedGroups(excludedGroups) {
}