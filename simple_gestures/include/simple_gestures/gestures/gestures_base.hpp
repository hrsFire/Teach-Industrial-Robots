#ifndef GESTURES_GESTURES_BASE_HPP
#define GESTURES_GESTURES_BASE_HPP

#include "gestures_query.hpp"

namespace gestures {
    /**
     * This class provides the possibility to define a custom gesture.
     */
    class GesturesBase : public GesturesQuery {
    public:
        /**
         * Destructor for a custom gesture.
         */
        virtual ~GesturesBase() {
        };
        /**
         * Checks if new depth data is available and prepares the depth data.
         * @return void
         */
        virtual void NextCycle() = 0;
        /**
         * Checks if new depth data is available.
         * @return True if new depth data is available. Otherwise False is returned.
         */
        virtual bool IsNewDataAvailable() = 0;
        /**
         * Marks the data that they have already been used.
         * @return void
         */
        virtual void MarkDataAsUsed() {
            dataWasUsed = true;
        };
    protected:
        /**
         * Specifies if the depth data was already been used.
         */
        bool dataWasUsed = false;
    };
}

#endif //GESTURES_GESTURES_BASE_HPP
