#ifndef GESTURES_GESTURE_HPP
#define GESTURES_GESTURE_HPP

#include <functional>
#include <chrono>
#include "gestures_base.hpp"

namespace gestures {
    class Gesture {
    public:
        Gesture(std::function<bool(GesturesQuery&)> checkGesture, std::function<void(std::chrono::milliseconds duration)> action);
    private:
        std::function<bool(GesturesQuery&)> isGesture;
        std::function<void(std::chrono::milliseconds duration)> doAction;
        friend class GesturesEngine;
    };
}

#endif //GESTURES_GESTURE_HPP