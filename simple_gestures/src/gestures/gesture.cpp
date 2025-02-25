#include "simple_gestures/gestures/gesture.hpp"

using namespace gestures;

Gesture::Gesture(std::function<bool(const GesturesQuery&)> checkGesture, std::function<void(std::chrono::milliseconds duration)> action) :
        isGesture(checkGesture), doAction(action) {
}