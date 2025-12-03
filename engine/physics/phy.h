#pragma once
#include "physicsresource.h"


namespace Physics {
    struct Simplex;

    struct Ray;

    enum class ShapeType : uint8_t {
        Box = 0,
        // Sphere,
        Custom,
    };

    struct State {
        struct Dyn {
            glm::vec3 pos = glm::vec3(0);
            glm::vec3 vel = glm::vec3(0);
            glm::quat rot = glm::quat();
            glm::quat angular_vel = glm::quat();
            glm::vec3 impulse_accum = glm::vec3(0);
            glm::vec3 torque_accum = glm::vec3(0);

            Dyn& set_pos(const glm::vec3& p);
            Dyn& set_vel(const glm::vec3& v);
            Dyn& set_rot(const glm::quat& r);
        };

        Dyn dyn;
        glm::mat3 inv_inertia_shape = glm::mat3(0);
        glm::vec3 orig = glm::vec3(0);
        float drag = 0.99f;
        float mass = 1.0f;
        float inv_mass = 1.0f;

        State& set_inertia_tensor(const glm::mat3& m);
        State& set_orig(const glm::vec3& o);
        State& set_drag(float d);
        State& set_mass(float m);
    };

    struct Deriv {
        glm::vec3 dpos = glm::vec3(0), dvel = glm::vec3(0);
    };

    struct AABB;

    struct Colliders {
        std::vector<ColliderMeshId> meshes;
        std::vector<AABB> aabbs;
        std::vector<glm::mat4> transforms;
        std::vector<State> states;
    };

    constexpr auto gravity = glm::vec3(0, -0.00981f, 0);

    const Colliders& get_colliders();
    Colliders& colliders();

    ColliderId create_collider(
        ColliderMeshId cm_id, const glm::vec3& orig, const glm::vec3& translation, const glm::quat& rotation,
        ShapeType type = ShapeType::Box
        );
    void set_transform(ColliderId collider, const glm::mat4& t);

    bool cast_ray(const Ray& ray, HitInfo& hit);
    bool cast_ray(const glm::vec3& start, const glm::vec3& dir, HitInfo& hit);

    void add_center_impulse(ColliderId collider, const glm::vec3& dir);
    void add_impulse(ColliderId collider, const glm::vec3& loc, const glm::vec3& dir);
    void step(float dt);
    void update_aabbs();

    struct AABBPair {
        ColliderId a, b;
    };

    void sort_and_sweep(std::vector<AABBPair>& aabb_pairs);

    bool gjk(ColliderId a_id, ColliderId b_id, Simplex& out_simplex);
    CollisionInfo epa(const Simplex& simplex, ColliderId a_id, ColliderId b_id);

} // namespace Physics
