#include "robot_arm_interface/operating_mode.hpp"

using namespace robot_arm;

OperatingMode::OperatingMode(std::string operatingMode) {
    if (operatingMode == "none") {
        this->mode = Mode::NONE;
    } else if (operatingMode == "position") {
        this->mode = Mode::POSITION;
    } else if (operatingMode == "ext_position") {
        this->mode = Mode::POSITION;
    } else if (operatingMode == "velocity") {
        this->mode = Mode::POSITION;
    } else if (operatingMode == "current") {
        this->mode = Mode::POSITION;
    } else if (operatingMode == "pwm") {
        this->mode = Mode::POSITION;
    } else {
        this->mode = Mode::NONE;
    }
}

OperatingMode OperatingMode::NONE() {
    return OperatingMode(Mode::NONE);
}

OperatingMode OperatingMode::POSITION() {
    return OperatingMode(Mode::POSITION);
}

OperatingMode OperatingMode::POSITION_MULTIPLE_REVOLUTIONS() {
    return OperatingMode(Mode::POSITION_MULTIPLE_REVOLUTIONS);
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

OperatingMode::operator std::string() const {
    switch (mode) {
        case Mode::NONE:
            return "none";
        case Mode::POSITION:
            return "position";
        case Mode::POSITION_MULTIPLE_REVOLUTIONS:
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

bool OperatingMode::operator==(const OperatingMode& operatingMode) const {
    return mode == operatingMode.mode;
}

OperatingMode::OperatingMode(Mode mode) {
    this->mode = mode;
}