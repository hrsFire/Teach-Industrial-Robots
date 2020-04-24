#ifndef GESTURES_GESTURES_BASE_HPP
#define GESTURES_GESTURES_BASE_HPP

namespace gestures {
    class GesturesBase
    {
    public:
        virtual void NextCycle() = 0;
        virtual bool IsNewDataAvailable() = 0;
        virtual bool MarkDataAsUsed() {
            dataWasUsed = true;
        };
        virtual double PerpendicularDistance(uint32_t startJointIndex, uint32_t endJointIndex, uint32_t jointIndex) = 0;
        virtual void ShowDebugInfo() = 0;
    protected:
        bool dataWasUsed = false;
    };
}

#endif //GESTURES_GESTURES_BASE_HPP