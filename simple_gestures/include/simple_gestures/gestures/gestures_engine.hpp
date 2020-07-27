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
    /**
     * This class manages gestures and executes the actions when the gestures are recognized.
     */
    class GesturesEngine {
    public:
        /**
         * Default constructor for the gestures engine.
         * @param gesturesImpl  A custom gesture.
         */
        GesturesEngine(GesturesBase* gesturesImpl);
        /**
         * Destructor for the gestures engine.
         */
        ~GesturesEngine();
        /**
         * Defines and adds a gesture group to the gestures engine.
         * @param name            The name of the gestures group.
         * @param priority        The priority of the gestures group. If the priority is the same as an existing one the new gestures group will be added before the existing one.
         * @param excludedGroups  The groups which should be checked before executing this gestures group. Therefore, if an excluded group has already been executed before, this gesture will not be executed.
         * @return The gestures group which was added to the gestures engine.
         */
        GestureGroup AddGestureGroup(std::string name, uint priority, std::list<GestureGroup> excludedGroups);
        /**
         * Adds a gesture with a specific gestures group to the gestures engine.
         * @param gesture           The gesture to add to the gestures engine.
         * @param group             The group where the gesture should be added in the gestures engine.
         * @param priorityInGroup   The priority of the gesture in the gestures group.
         * @param affectedItems     The items which are affected by the execution of the gesture. Any string is valid as an item key. This prevents gestures from intervering with each other which would most likely lead to conflicting behaviour.
         * @param preventConflicts  Specifies whether the gestures engine should prevent the execution of the gesture if a gesture with the same affected items has already been executed before.
         * @return void
         */
        void AddGesture(Gesture gesture, GestureGroup group, uint priorityInGroup, std::vector<std::string> affectedItems, bool preventConflicts = true);
        /**
         * Starts the gestures engine. This method call blocks the current thread.
         * @return void
         */
        void Start();
        /**
         * Stops the gestures engine.
         * @return void
         */
        void Stop();
    private:
        bool isRunning = false;
        GesturesBase* gesturesImpl = nullptr;
        std::list<void>* groups;
    };
}

#endif //GESTURES_GESTURES_ENGINE_HPP
