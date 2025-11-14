#pragma once
#include "physicsresource.h"


namespace Physics {

    struct Ray;

    struct State {
        glm::vec3 pos = glm::vec3(0), vel = glm::vec3(0);
    };

    struct Deriv {
        glm::vec3 dx = glm::vec3(0), dv = glm::vec3(0);
    };

    struct Colliders {
        std::vector<ColliderMeshId> meshes;
        std::vector<glm::mat4> transforms;
        std::vector<State> states;
        std::vector<Deriv> derivs;
    };

    const Colliders& get_colliders();

    ColliderId create_collider(ColliderMeshId cm_id, const glm::mat4& t);
    void set_transform(ColliderId collider, const glm::mat4& t);

    bool cast_ray(const Ray& ray, HitInfo& hit);
    bool cast_ray(const glm::vec3& start, const glm::vec3& dir, HitInfo& hit);

    void step();

} // namespace Physics
