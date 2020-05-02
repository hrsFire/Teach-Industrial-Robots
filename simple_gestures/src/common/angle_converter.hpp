#ifndef COMMON_ANGLE_CONVERTER_HPP
#define COMMON_ANGLE_CONVERTER_HPP

#include <math.h>

namespace common {
    class AngleConverter {
    public:
        static double DegreeFromRadian(double radian);
        static double RadianFromDegree(double degree);
    };
}

#endif //COMMON_ANGLE_CONVERTER_HPP