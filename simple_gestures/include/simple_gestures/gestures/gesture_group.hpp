#ifndef GESTURES_GESTURE_GROUP_HPP
#define GESTURES_GESTURE_GROUP_HPP

#include <string>
#include <vector>

/**
* This namespace provides numerous stuff for gesture recognition.
*/
namespace gestures {
    /**
     * This class provides the possibility to group similar gestures.
     */
    class GestureGroup {
    public:
        /**
         * Casts the gesture group to a string representation.
         * @return String representation for the gesture group.
         */
        operator std::string() const;
        /**
         * Compares the items of the two gesture groups.
         * @param[in] gestureGroup  A gesture group.
         * @return True if all gesture items are equal. Otherwise False is returned.
         */
        bool operator==(const GestureGroup& gestureGroup) const;
    private:
        GestureGroup(std::string name);
        const std::string name;
        friend class GesturesEngine;
    };
}

#endif //GESTURES_GESTURE_GROUP_HPP
