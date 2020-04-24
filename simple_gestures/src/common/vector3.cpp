#include "vector3.hpp"

using namespace common;

double Vector3::PerpendicularDistance(glm::vec3 startPoint, glm::vec3 endPoint, glm::vec3 point) {
    glm::vec3 d = (point - startPoint) / glm::distance(point, startPoint);
    glm::vec3 v = endPoint - startPoint;
    double t = glm::dot(v, d);
    glm::vec3 p = startPoint + (float)t * d;

    return glm::distance(p, endPoint);
}