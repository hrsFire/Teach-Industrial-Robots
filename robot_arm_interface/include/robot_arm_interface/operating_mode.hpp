#ifndef ROBOT_ARM_OPERATING_MODE_HPP
#define ROBOT_ARM_OPERATING_MODE_HPP

#include <string>

namespace robot_arm {
    /**
     * This class provides all available operating modes for the different robot arm models.
     */
    class OperatingMode {
    public:
        /**
         * Default constructor for an operating mode for different robot arm models.
         * @param operatingMode  The specific operating mode.
         */
        OperatingMode(std::string operatingMode);
        /**
         * Returns an invalid operating mode. All motor commands will be ignored.
         * @return An invalid operating mode.
         */
        static OperatingMode NONE();
        /**
         * Returns the "position" operating mode. All commands will be treated as position goals. The joint cannot go past one revolution.
         * @return The "position" operating mode.
         */
        static OperatingMode POSITION();
        /**
         * Returns the "position with multiple revolutions" operating mode. All commands will be treated as position goals. Multiple revolutions are allowed for the joint.
         * @return The "position with multiple revolutions" operating mode.
         */
        static OperatingMode POSITION_MULTIPLE_REVOLUTIONS();
        /**
         * Returns the "velocity" operating mode. All commands will be treated as velocity [rad/s] goals.
         * @return The "velocity" operating mode.
         */
        static OperatingMode VELOCITY();
        /**
         * Returns the "current" operating mode. All commands will be treated as current [mA] goals.
         * @return The "current" operating mode.
         */
        static OperatingMode CURRENT();
        /**
         * Returns the "pwm" operating mode. All commands will be treated as pwm (pulse width modulation) goals.
         * @return The "pwm" operating mode.
         */
        static OperatingMode PWM();
        /**
         * Casts the operating mode to a string representation.
         * @return String representation for the operating mode.
         */
        operator std::string() const;
        /**
         * Compares two operating modes.
         * @param operatingMode  An operating mode.
         * @return Bool value for the operating mode comparison.
         */
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

#endif //ROBOT_ARM_OPERATING_MODE_HPP
