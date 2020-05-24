#ifndef GESTURES_GESTURES_ENGINE_HPP
#define GESTURES_GESTURES_ENGINE_HPP

#include <vector>
#include <list>
#include <unordered_set>
#include <chrono>
#include "gesture.hpp"
#include "gestures_base.hpp"
#include "gesture_group.hpp"

#ifndef NDEBUG
#include <iostream>
#endif //NDEBUG

namespace gestures {
    class GesturesEngine {
    public:
        GesturesEngine(GesturesBase* gesturesImpl);
        ~GesturesEngine();
        // excludedGroups: Describes the gesture groups which can't be executed with this gesture group. If a gesture in a gesture group was
        //                 already executed in this gestures cycle this gesture group will not be executed and therefore will be skipped.
        GestureGroup AddGestureGroup(std::string name, uint priority, std::list<GestureGroup> excludedGroups);
        // affectedItems: Describes the items which are affected through the gesture. If an item was previously affected in the current gestures
        //                cycle a further action for the gesture which would also affect the item will not be executed. This prevents gestures
        //                from intervering with each other which would most likely lead to conflicting behaviour.
        bool AddGesture(Gesture gesture, GestureGroup group, uint priorityInGroup, std::vector<std::string> affectedItems, bool preventConflicts = true);
        void Start();
        void Stop();
    private:
        bool isRunning = false;
        GesturesBase* gesturesImpl = nullptr;
        std::list<void>* groups;
    };
}

#endif //GESTURES_GESTURES_ENGINE_HPP