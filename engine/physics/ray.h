#pragma once
#include "vec3.hpp"

namespace Physics {

    struct Ray {
        glm::vec3 orig, dir;

        explicit Ray(const glm::vec3& orig, const glm::vec3& dir) : orig(orig), dir(dir) {}
    };

} // namespace Physics