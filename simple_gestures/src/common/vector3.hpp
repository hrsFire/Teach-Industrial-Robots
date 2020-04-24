
#ifndef COMMON_VECTOR3_HPP
#define COMMON_VECTOR3_HPP

#include <glm/vec3.hpp>
#include <glm/geometric.hpp>

namespace common {
    class Vector3 {
    public:
        // Calculates the perpendicular distance between a line, which has a start point and an end point, and an independent point.
        // https://math.stackexchange.com/questions/1905533/find-perpendicular-distance-from-point-to-line-in-3d/1905794
        static double PerpendicularDistance(glm::vec3 A, glm::vec3 B, glm::vec3 point);
    };
}

#endif //COMMON_VECTOR3_HPP