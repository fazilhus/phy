#pragma once
#include "physicsresource.h"


namespace Physics {

    struct Ray;

    struct State {
        struct Dyn {
            glm::vec3 pos = glm::vec3(0);
            glm::vec3 vel = glm::vec3(0);
            glm::vec3 force_dir = glm::vec3(0);
            glm::vec3 impulse_dir = glm::vec3(0);
            glm::quat rot = glm::quat();
            glm::quat angular_vel = glm::quat();
            glm::quat torque = glm::quat();
            float force_size = 0.0f;
            float impulse_size = 0.0f;
        };

        Dyn dyn;
        glm::vec3 orig = glm::vec3(0);
        float drag = 0.99f;
        float inv_mass = 1.0f;
    };

    struct Deriv {
        glm::vec3 dpos = glm::vec3(0), dvel = glm::vec3(0);
    };

    struct Colliders {
        std::vector<ColliderMeshId> meshes;
        std::vector<glm::mat4> transforms;
        std::vector<State> states;
    };

    constexpr auto gravity = glm::vec3(0, -0.1f, 0);

    const Colliders& get_colliders();
    Colliders& colliders();

    ColliderId create_collider(ColliderMeshId cm_id, const glm::vec3& orig, const glm::mat4& t);
    void set_transform(ColliderId collider, const glm::mat4& t);

    bool cast_ray(const Ray& ray, HitInfo& hit);
    bool cast_ray(const glm::vec3& start, const glm::vec3& dir, HitInfo& hit);

    void add_force(ColliderId collider, const glm::vec3& f);
    void add_impulse(ColliderId collider, const glm::vec3& loc, const glm::vec3& dir);
    void step(float dt);

} // namespace Physics
