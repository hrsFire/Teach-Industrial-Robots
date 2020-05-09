#ifndef INTERBOTIX_OPERATING_MODE_HPP
#define INTERBOTIX_OPERATING_MODE_HPP

#include <string>

namespace robot_arm {
    class OperatingMode {
    public:
        static OperatingMode NONE();
        static OperatingMode POSITION();
        static OperatingMode POSITION_MULTIPLE_REVOLUTIONS();
        static OperatingMode VELOCITY();
        static OperatingMode CURRENT();
        static OperatingMode PWM();
        operator std::string();
    private:
        enum class Mode {
            NONE,
            POSITION,
            POSITION_MULTIPLE_REVOLUTIONS,
            VELOCITY,
            CURRENT,
            PWM  // pulse width modulation
        };
        OperatingMode(Mode mode);
        Mode mode;
    };
}

#endif //INTERBOTIX_OPERATING_MODE_HPP