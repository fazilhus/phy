#pragma once
#include "vec3.hpp"

#include "physics/physicsresource.h"

namespace Math {

    inline float len_sq(const glm::vec3& v) { return v.x * v.x + v.y * v.y + v.z * v.z; }

    inline glm::vec3 safe_normal(const glm::vec3& v) {
        const auto len_sq = v.x * v.x + v.y * v.y + v.z * v.z;
        if (len_sq <= Physics::epsilon) { return glm::vec3(0, 0, 0); }
        return v / sqrt(len_sq);
    }

    inline glm::vec2 norm_screen_pos(const glm::vec2& sp, const glm::vec2& ss) {
        return glm::vec2(2.0f * (sp / ss) - 1.0f);
    }

    inline glm::quat rot_vec3(const glm::vec3& norm) {
        constexpr auto up = glm::vec3(0.0f, 0.0f, 1.0f);
        const auto axis = glm::cross(up, norm);
        const float angle = glm::acos(glm::dot(up, norm));

        return glm::angleAxis(angle, glm::normalize(axis));
    }

    inline float min(const float a, const float b) { return a < b ? a : b; }
    inline float min(const float a, const float b, const float c) { return min(min(a, b), c); }

    inline float max(const float a, const float b) { return a > b ? a : b; }
    inline float max(const float a, const float b, const float c) { return max(max(a, b), c); }

}
