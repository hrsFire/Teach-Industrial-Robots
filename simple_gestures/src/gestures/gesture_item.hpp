#ifndef GESTURES_GESTURE_ITEM_HPP
#define GESTURES_GESTURE_ITEM_HPP

#include <vector>
#include <chrono>
#include "simple_gestures/gestures/gesture.hpp"

namespace gestures {
    class GestureItem {
    public:
        GestureItem(Gesture gesture, std::vector<std::string> affectedItems, bool preventConflicts);
        const Gesture gesture;
        const std::vector<std::string> affectedItems;
        const bool preventConflicts;
        std::chrono::system_clock::time_point startTime;
        bool isActive = false;
    };
}

#endif //GESTURES_GESTURE_ITEM_HPP