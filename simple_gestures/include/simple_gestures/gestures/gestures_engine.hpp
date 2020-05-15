#ifndef GESTURES_GESTURES_ENGINE_HPP
#define GESTURES_GESTURES_ENGINE_HPP

#include <vector>
#include <list>
#include <unordered_set>
#include "gesture.hpp"
#include "gestures_base.hpp"
#include "../src/gestures/gesture_group.hpp"

namespace gestures {
    class GesturesEngine {
    public:
        GesturesEngine(GesturesBase* gesturesImpl);
        ~GesturesEngine();
        // affectedItems: Describes the items which are affected through the gesture. If an item was previously affected in the current gestures
        //                cycle a further action for the gesture which would also affect the item will not be executed. This prevents gestures
        //                from intervering with each other which would most likely lead to conflicting behaviour.
        void AddGesture(Gesture gesture, std::string group, uint groupPriority, uint priorityInGroup, std::vector<std::string> affectedItems, bool preventConflicts = true);
        void Start();
        void Stop();
    private:
        bool isRunning = false;
        GesturesBase* gesturesImpl = nullptr;
        std::list<GestureGroup> groups;
    };
}

#endif //GESTURES_GESTURES_ENGINE_HPP