#pragma once
#include "vec3.hpp"
#include "vec4.hpp"


namespace Physics {

    struct Plane {
        glm::vec3 point = glm::vec3(0);
        glm::vec3 norm = glm::vec3(0, 1, 0);
    };

} // namespace Physics
