#include "robot_arm_interface/operating_mode.hpp"

using namespace robot_arm;

OperatingMode OperatingMode::NONE() {
    return OperatingMode(Mode::NONE);
}

OperatingMode OperatingMode::POSITION() {
    return OperatingMode(Mode::POSITION);
}

OperatingMode OperatingMode::EXT_POSITION() {
    return OperatingMode(Mode::EXT_POSITION);
}

OperatingMode OperatingMode::VELOCITY() {
    return OperatingMode(Mode::VELOCITY);
}

OperatingMode OperatingMode::CURRENT() {
    return OperatingMode(Mode::CURRENT);
}

OperatingMode OperatingMode::PWM() {
    return OperatingMode(Mode::PWM);
}

OperatingMode::operator std::string() {
    switch (mode)
    {
        case Mode::NONE:
            return "none";
        case Mode::POSITION:
            return "position";
        case Mode::EXT_POSITION:
            return "ext_position";
        case Mode::VELOCITY:
            return "velocity";
        case Mode::CURRENT:
            return "current";
        case Mode::PWM:
            return "pwm";
        default:
            return "";
    }
}

OperatingMode::OperatingMode(Mode mode) {
    this->mode = mode;
}