#ifndef GESTURES_GESTURES_QUERY_HPP
#define GESTURES_GESTURES_QUERY_HPP

namespace gestures {
    /**
     * This class offers an interface for implementing gesture recognition without the burden of fiddling around with the low-level implementation of a gesture recognition system. This also abstracts away the low-level implementation and offers a way to switch between different gesture recognition systems.
     */
    class GesturesQuery {
    public:
        /**
         * Checks if a joint is intersecting with two other joints. This is implemented by checking if a point is intersecting with a line..
         * @param startJointIndex  The ID of the joint which should be used as the start point of the line.
         * @param endJointIndex    The ID of the joint which should be used as the end point of the line.
         * @param jointIndex       The ID of the joint which should be used as the point which intersects with the line.
         * @param minDistance      The minimum distance from the line to the intersecting point. The distance is specified in millimeters.
         * @param maxDistance      The maximum distance from the line to the intersecting point. The distance is specified in millimeters.
         * @return True if the gesture was recognized. Otherwise False is returned.
         */
        virtual bool IsGesture(uint32_t startJointIndex, uint32_t endJointIndex, uint32_t jointIndex, double minDistance, double maxDistance) const = 0;
        /**
         * Checks if a joint is intersecting with another joint. This is implemented by checking the distance from one point to another.
         * @param jointIndex1  The ID of the joint which should be used as the first point.
         * @param jointIndex2  The ID of the joint which should be used as the second point.
         * @param minDistance  The minimum distance from one point to the other one. The distance is specified in millimeters.
         * @param maxDistance  The maximum distance from one point to the other one. The distance is specified in millimeters.
         * @return True if the gesture was recognized. Otherwise False is returned.
         */
        virtual bool IsGesture(uint32_t jointIndex1, uint32_t jointIndex2, double minDistance, double maxDistance) const = 0;
        /**
         * Checks if a joint is visible for the gesture recognition system.
         * @param jointIndex  The ID of the joint which should be used to check if the joint is visible.
         * @return True if the joint is visible. Otherwise False is returned.
         */
        virtual bool IsJointVisible(uint32_t jointIndex) const = 0;
    };
}

#endif //GESTURES_GESTURES_QUERY_HPP
