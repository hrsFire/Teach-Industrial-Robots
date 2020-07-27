#ifndef GESTURES_GESTURE_HPP
#define GESTURES_GESTURE_HPP

#include <functional>
#include <chrono>
#include "gestures_base.hpp"

namespace gestures {
    /**
     * This class offers the possibility to link a gesture with an action.
     */
    class Gesture {
    public:
        /**
         * Default constructor for a custom gesture which is linked with an action.
         * @param checkGesture  A function which evaluates to True if the gesture has been recognized.
         * @param action        The action to activate when the custom checkGesture function has returned True.
         */
        Gesture(std::function<bool(const GesturesQuery&)> checkGesture, std::function<void(std::chrono::milliseconds duration)> action);
    private:
        std::function<bool(const GesturesQuery& gesturesImpl)> isGesture;
        std::function<void(std::chrono::milliseconds duration)> doAction;
        friend class GesturesEngine;
    };
}

#endif //GESTURES_GESTURE_HPP
