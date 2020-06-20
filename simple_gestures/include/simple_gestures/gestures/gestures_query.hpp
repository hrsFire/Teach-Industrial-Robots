#ifndef GESTURES_GESTURES_QUERY_HPP
#define GESTURES_GESTURES_QUERY_HPP

namespace gestures {
    class GesturesQuery {
    public:
        // minDistance and maxDistance are specified in millimeters
        virtual bool IsGesture(uint32_t startJointIndex, uint32_t endJointIndex, uint32_t jointIndex, double minDistance, double maxDistance) const = 0;
        virtual bool IsGesture(uint32_t jointIndex1, uint32_t jointIndex2, double minDistance, double maxDistance) const = 0;
        virtual bool IsJointVisible(uint32_t jointIndex) const = 0;
    };
}

#endif //GESTURES_GESTURES_QUERY_HPP