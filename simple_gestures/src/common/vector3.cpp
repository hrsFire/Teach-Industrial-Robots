#include "vector3.hpp"
#include <iostream>

using namespace common;

float Vector3::PerpendicularDistance(glm::vec3 startPoint, glm::vec3 endPoint, glm::vec3 point) {
    glm::vec3 p = ProjectedPoint(startPoint, endPoint, point);

    return glm::distance(p, endPoint);
}

float Vector3::DeviationOnLine(glm::vec3 startPoint, glm::vec3 endPoint, glm::vec3 point) {
    glm::vec3 p = ProjectedPoint(startPoint, endPoint, point);
    // https://math.stackexchange.com/questions/1091020/finding-midpoint-of-rectangle-in-3d-vectors
    glm::vec3 midpoint = glm::vec3((startPoint.x + endPoint.x) / 2.0,
        (startPoint.y + endPoint.y) / 2.0,
        (startPoint.z + endPoint.z) / 2.0);

    float distanceToStart = glm::distance(startPoint, midpoint);
    float distanceToProjection = glm::distance(p, midpoint);

    return distanceToStart - distanceToProjection;
}

glm::vec3 Vector3::ProjectedPoint(glm::vec3 startPoint, glm::vec3 endPoint, glm::vec3 point) {
    glm::vec3 d = (point - startPoint) / glm::distance(point, startPoint);
    glm::vec3 v = endPoint - startPoint;
    float t = glm::dot(v, d);

    return startPoint + t * d;
}