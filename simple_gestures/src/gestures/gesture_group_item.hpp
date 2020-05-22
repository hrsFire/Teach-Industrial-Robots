#ifndef GESTURES_GESTURE_GROUP_ITEM_HPP
#define GESTURES_GESTURE_GROUP_ITEM_HPP

#include <list>
#include "gesture_item.hpp"
#include "simple_gestures/gestures/gesture_group.hpp"

namespace gestures {
    class GestureGroupItem {
    public:
        GestureGroupItem(std::string name, std::list<GestureItem> gestures, std::list<GestureGroup> excludedGroups);
        const std::string name;
        std::list<GestureItem> gestures;
        std::list<GestureGroup> excludedGroups;
    };
}

#endif //GESTURES_GESTURE_GROUP_ITEM_HPP