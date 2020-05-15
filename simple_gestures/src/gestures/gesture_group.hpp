#ifndef GESTURES_GESTURE_GROUP_HPP
#define GESTURES_GESTURE_GROUP_HPP

#include <list>
#include "gesture_item.hpp"

namespace gestures {
    class GestureGroup {
    public:
        GestureGroup(std::string name, std::list<GestureItem> gestures);
        const std::string name;
        std::list<GestureItem> gestures;
    };
}

#endif //GESTURES_GESTURE_GROUP_HPP