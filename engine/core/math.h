#pragma once
#include "vec3.hpp"


namespace Math {

    inline glm::vec3 safe_normal(const glm::vec3& v) {
        const auto len_sq = v.x * v.x + v.y * v.y + v.z * v.z;
        if (len_sq <= 1e-6) {
            return glm::vec3(0, 0, 0);
        }
        return v / sqrt(len_sq);
    }

}
