#ifndef GESTURES_GESTURE_HPP
#define GESTURES_GESTURE_HPP

#include <functional>
#include "gestures_base.hpp"

namespace gestures {
    class Gesture {
    public:
        Gesture(std::function<bool(GesturesQuery&)> checkGesture, std::function<void()> action);
    private:
        std::function<bool(GesturesQuery&)> isGesture;
        std::function<void()> doAction;
        friend class GesturesEngine;
    };
}

#endif //GESTURES_GESTURE_HPP