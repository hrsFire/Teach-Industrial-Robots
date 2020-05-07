
#ifndef COMMON_VECTOR3_HPP
#define COMMON_VECTOR3_HPP

#include <glm/vec3.hpp>
#include <glm/geometric.hpp>

namespace common {
    class Vector3 {
    public:
        // Calculates the perpendicular distance between a line, which has a start point and an end point, and an independent point.
        // https://math.stackexchange.com/questions/1905533/find-perpendicular-distance-from-point-to-line-in-3d/1905794
        static float PerpendicularDistance(glm::vec3 startPoint, glm::vec3 endPoint, glm::vec3 point);
        // Calculates the deviation on the line. If >= 0 then the point is between the start and end point. Otherwise it isn't.
        static float DeviationOnLine(glm::vec3 startPoint, glm::vec3 endPoint, glm::vec3 point);
        static float Distance(glm::vec3 point1, glm::vec3 point2);
        static glm::vec3 ProjectedPoint(glm::vec3 startPoint, glm::vec3 endPoint, glm::vec3 point);
    };
}

#endif //COMMON_VECTOR3_HPP