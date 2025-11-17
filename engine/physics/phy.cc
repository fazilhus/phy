#include "config.h"

#include "phy.h"

#include "physicsmesh.h"
#include "core/idpool.h"
#include "core/math.h"
#include "physics/ray.h"


namespace Physics {

    static Util::IdPool<ColliderId> collider_id_pool;
    static Colliders colliders_;

    const Colliders& get_colliders() { return colliders_; }
    Colliders& colliders() { return colliders_; }

    ColliderId create_collider(ColliderMeshId cm_id, const glm::mat4& t) {
        ColliderId id;
        if (collider_id_pool.Allocate(id)) {
            colliders_.meshes.emplace_back(cm_id);
            colliders_.transforms.emplace_back(t);
            colliders_.states.emplace_back(State{.dyn = {.pos = t[3]}});
        }
        else {
            colliders_.meshes[id.index] = cm_id;
            colliders_.transforms[id.index] = t;
            colliders_.states[id.index] = State{.dyn = {.pos = t[3]}};
        }
        return id;
    }

    void set_transform(const ColliderId collider, const glm::mat4& t) {
        assert(collider_id_pool.IsValid(collider));
        colliders_.transforms[collider.index] = t;
    }

    bool cast_ray(const Ray& ray, HitInfo& hit) {
        std::vector<HitInfo> aabb_hits;
        const auto inv_dir = 1.0f / ray.dir;
        for (std::size_t i = 0; i < colliders_.meshes.size(); ++i) {
            const auto c = ColliderId(i);
            const auto cm = colliders_.meshes[i];
            const auto& aabb = get_collider_meshes().simple[cm.index];
            HitInfo temp_hit;
            if (aabb.intersect(ray, temp_hit, colliders_.transforms[i], inv_dir)) {
                aabb_hits.emplace_back(temp_hit);
                aabb_hits.back().collider = c;
                aabb_hits.back().mesh = cm;
            }
        }

        if (aabb_hits.empty()) { return false; }

        std::ranges::sort(
            aabb_hits
            , [](const HitInfo& lhs, const HitInfo& rhs)-> bool { return lhs.t < rhs.t; }
            );

        HitInfo best_hit;
        for (const auto it: aabb_hits) {
            auto& cm = get_collider_meshes().complex[it.mesh.index];
            const auto t = colliders_.transforms[it.collider.index];
            const auto inv_t = glm::inverse(t);
            const auto model_ray = Ray(
                inv_t * glm::vec4(ray.orig, 1.0f), Math::safe_normal(glm::quat(inv_t) * glm::vec4(ray.dir, 1.0f))
                );

            for (auto& p: cm.primitives) { for (auto& tri: p.triangles) { tri.selected = false; } }

            HitInfo temp_hit;
            if (cm.intersect(model_ray, temp_hit)) {
                if (temp_hit.t < best_hit.t) {
                    best_hit = temp_hit;
                    best_hit.collider = it.collider;
                    best_hit.mesh = it.mesh;
                    best_hit.pos = t * glm::vec4(best_hit.pos, 1.0f);
                }
            }
        }

        if (best_hit.hit()) {
            hit = best_hit;
            get_collider_meshes().complex[hit.mesh.index].primitives[hit.prim_n].triangles[hit.tri_n].selected = true;
            return true;
        }

        return false;
    }

    bool cast_ray(const glm::vec3& start, const glm::vec3& dir, HitInfo& hit) { return cast_ray(Ray(start, dir), hit); }

    void add_force(ColliderId collider, const glm::vec3& f) {
        auto& state = colliders_.states[collider.index];
        state.dyn.force_dir += f;
        state.dyn.force_size += glm::length(f);
    }

    void add_impulse(ColliderId collider, const glm::vec3& i) {
        auto& state = colliders_.states[collider.index];
        state.dyn.impulse_dir += i;
        state.dyn.impulse_size += glm::length(i);
    }

    void step(const float dt) {
        for (std::size_t i = 0; i < colliders_.states.size(); ++i) {
            auto& state = colliders_.states[i];
            const auto acc = Math::safe_normal(state.dyn.force_dir) * state.dyn.force_size * state.inv_mass;
            const auto impulse = Math::safe_normal(state.dyn.impulse_dir) * state.dyn.impulse_size * state.inv_mass;
            state.dyn.vel += impulse * dt + acc * dt;
            state.dyn.pos += state.dyn.vel * dt;

            state.dyn.force_dir = glm::vec3(0);
            state.dyn.impulse_dir = glm::vec3(0);
            state.dyn.force_size = 0.0f;
            state.dyn.impulse_size = 0.0f;

            auto& t = colliders_.transforms[i];
            t = glm::translate(state.dyn.pos);
        }
    }


} // namespace Physics
