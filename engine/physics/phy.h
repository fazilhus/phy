#pragma once
#include "vec3.hpp"


namespace Physics {

    struct HitInfo {
        glm::vec3 pos;
        glm::vec3 norm;
    };

    constexpr auto epsilon = 1e-6f;

} // namespace Physics
