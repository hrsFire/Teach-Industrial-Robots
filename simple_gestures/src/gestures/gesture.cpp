#include "simple_gestures/gestures/gesture.hpp"

using namespace gestures;

Gesture::Gesture(std::function<bool(GesturesQuery&)> checkGesture, std::function<void()> action) : isGesture(checkGesture), doAction(action) {
}