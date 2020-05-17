#ifndef INTERBOTIX_OPERATING_MODE_HPP
#define INTERBOTIX_OPERATING_MODE_HPP

#include <string>

namespace robot_arm {
    class OperatingMode {
    public:
        // All motor commands will be ignored
        static OperatingMode NONE();
        // All commands will be treated as goal positions (cannot go past 1 revolution)
        static OperatingMode POSITION();
        // All commands will be treated as goal positions (multiple revolutions allowed)
        static OperatingMode POSITION_MULTIPLE_REVOLUTIONS();
        // All commands will be treated as goal velocities [rad/s]
        static OperatingMode VELOCITY();
        // All commands will be treated as goal currents [mA]
        static OperatingMode CURRENT();
        // All commands will be treated as goal pwms (pulse width modulation)
        static OperatingMode PWM();
        operator std::string();
        bool operator==(const OperatingMode& operatingMode) const;
    private:
        enum class Mode {
            NONE,
            POSITION,
            POSITION_MULTIPLE_REVOLUTIONS,
            VELOCITY,
            CURRENT,
            PWM
        };
        OperatingMode(Mode mode);
        Mode mode;
    };
}

#endif //INTERBOTIX_OPERATING_MODE_HPP