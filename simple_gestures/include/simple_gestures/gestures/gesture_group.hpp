#ifndef GESTURES_GESTURE_GROUP_HPP
#define GESTURES_GESTURE_GROUP_HPP

#include <string>
#include <vector>

namespace gestures {
    class GestureGroup {
    public:
        operator std::string() const;
        bool operator==(const GestureGroup& gestureGroup) const;
    private:
        GestureGroup(std::string name);
        const std::string name;
        friend class GesturesEngine;
    };
}

#endif //GESTURES_GESTURE_GROUP_HPP