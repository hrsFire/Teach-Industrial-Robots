#include "vector3.hpp"

using namespace common;

float Vector3::PerpendicularDistance(glm::vec3 startPoint, glm::vec3 endPoint, glm::vec3 point) {
    // https://math.stackexchange.com/questions/1905533/find-perpendicular-distance-from-point-to-line-in-3d
    glm::vec3 p = ProjectedPoint(startPoint, endPoint, point);

    return Distance(p, endPoint);
}

float Vector3::DeviationOnLine(glm::vec3 startPoint, glm::vec3 endPoint, glm::vec3 point) {
    glm::vec3 p = ProjectedPoint(startPoint, endPoint, point);
    // https://math.stackexchange.com/questions/1091020/finding-midpoint-of-rectangle-in-3d-vectors
    glm::vec3 midpoint = glm::vec3((startPoint.x + endPoint.x) / 2.0,
        (startPoint.y + endPoint.y) / 2.0,
        (startPoint.z + endPoint.z) / 2.0);

    float distanceToStart = Distance(startPoint, midpoint);
    float distanceToProjection = Distance(p, midpoint);

    return distanceToStart - distanceToProjection;
}

float Vector3::Distance(glm::vec3 point1, glm::vec3 point2) {
    return glm::distance(point1, point2);
}

glm::vec3 Vector3::ProjectedPoint(glm::vec3 startPoint, glm::vec3 endPoint, glm::vec3 point) {
    if (glm::all(glm::equal(startPoint, endPoint))) {
        throw "'startPoint' and 'endPoint' can't be the same: Division by zero";
    }

    glm::vec3 d = (endPoint - startPoint) / Distance(endPoint, startPoint);
    glm::vec3 v = point - startPoint;
    float t = glm::dot(v, d);

    return startPoint + t * d;
}
