#pragma once
#include "vec3.hpp"
#include "vec4.hpp"
#include "ext/quaternion_geometric.hpp"


namespace Physics {

    struct Plane {
        glm::vec3 norm;
        float dist;

        explicit Plane(const glm::vec3& point, const glm::vec3& norm)
            : norm(norm), dist(glm::length(point)) {
        }

        Plane(const glm::vec3& a, const glm::vec3& b, const glm::vec3& c) {
            auto v1 = a - b;
            auto v2 = a - c;
            norm = glm::cross(v1, v2);
            dist = (a * norm).length();
        }

        glm::vec3 point() const {
            return norm * dist;
        }
    };

} // namespace Physics
