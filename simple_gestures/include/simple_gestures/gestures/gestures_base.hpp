#ifndef GESTURES_GESTURES_BASE_HPP
#define GESTURES_GESTURES_BASE_HPP

#include "gestures_query.hpp"

namespace gestures {
    class GesturesBase : public GesturesQuery {
    public:
        virtual ~GesturesBase() {
        };
        virtual void NextCycle() = 0;
        virtual bool IsNewDataAvailable() = 0;
        virtual bool MarkDataAsUsed() {
            dataWasUsed = true;
        };
    protected:
        bool dataWasUsed = false;
    };
}

#endif //GESTURES_GESTURES_BASE_HPP