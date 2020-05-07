#ifndef GESTURES_GESTURES_BASE_HPP
#define GESTURES_GESTURES_BASE_HPP

namespace gestures {
    class GesturesBase {
    public:
        virtual void NextCycle() = 0;
        virtual bool IsNewDataAvailable() = 0;
        virtual bool MarkDataAsUsed() {
            dataWasUsed = true;
        };
        // minDistance and maxDistance are specified in millimeters
        virtual bool IsGesture(uint32_t startJointIndex, uint32_t endJointIndex, uint32_t jointIndex, double minDistance, double maxDistance) = 0;
        virtual bool IsGesture(uint32_t jointIndex1, uint32_t jointIndex2, double minDistance, double maxDistance) = 0;
    protected:
        bool dataWasUsed = false;
    };
}

#endif //GESTURES_GESTURES_BASE_HPP