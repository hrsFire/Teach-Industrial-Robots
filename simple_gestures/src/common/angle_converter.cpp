#include "angle_converter.hpp"

using namespace common;

double AngleConverter::DegreeFromRadian(double radian) {
    return (radian) * 180.0 / M_PI;
}

double AngleConverter::RadianFromDegree(double degree) {
    return (degree) * M_PI / 180.0;
}