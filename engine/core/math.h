#pragma once
#include "vec3.hpp"
#include "gtc/type_ptr.inl"


namespace Math {

    inline glm::vec3 safe_normal(const glm::vec3& v) {
        const auto len_sq = v.x * v.x + v.y * v.y + v.z * v.z;
        if (len_sq <= 1e-6) { return glm::vec3(0, 0, 0); }
        return v / sqrt(len_sq);
    }

    inline glm::quat rot_vec3(const glm::vec3& norm) {
        constexpr auto up = glm::vec3(0.0f, 0.0f, 1.0f);
        const auto axis = glm::cross(up, norm);
        const float angle = glm::acos(glm::dot(up, norm));

        return glm::angleAxis(angle, glm::normalize(axis));
    }

}
