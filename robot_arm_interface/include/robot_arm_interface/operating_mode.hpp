#ifndef INTERBOTIX_OPERATING_MODE_HPP
#define INTERBOTIX_OPERATING_MODE_HPP

#include <string>

namespace robot_arm {
    class OperatingMode {
    public:
        static OperatingMode NONE();
        static OperatingMode POSITION();
        static OperatingMode EXT_POSITION();
        static OperatingMode VELOCITY();
        static OperatingMode CURRENT();
        static OperatingMode PWM();
        operator std::string();
    private:
        enum class Mode {
            NONE,
            POSITION,
            EXT_POSITION,
            VELOCITY,
            CURRENT,
            PWM  // pulse width modulation
        };
        OperatingMode(Mode mode);
        Mode mode;
    };
}

#endif //INTERBOTIX_OPERATING_MODE_HPP