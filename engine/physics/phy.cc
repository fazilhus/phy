#include "config.h"

#include "phy.h"

#include "physicsmesh.h"
#include "core/idpool.h"
#include "core/math.h"
#include "physics/ray.h"


namespace Physics {

    static Util::IdPool<ColliderId> collider_id_pool;
    static Colliders colliders;

    const Colliders& get_colliders() { return colliders; }

    ColliderId create_collider(ColliderMeshId cm_id, const glm::mat4& t) {
        ColliderId id;
        if (collider_id_pool.Allocate(id)) {
            colliders.meshes.emplace_back(cm_id);
            colliders.transforms.emplace_back(t);
            colliders.states.emplace_back(t[3], glm::vec3());
            colliders.derivs.emplace_back();
        }
        else {
            colliders.meshes[id.index] = cm_id;
            colliders.transforms[id.index] = t;
            colliders.states[id.index] = {t[3], {}};
            colliders.derivs[id.index] = {};
        }
        return id;
    }

    void set_transform(const ColliderId collider, const glm::mat4& t) {
        assert(collider_id_pool.IsValid(collider));
        colliders.transforms[collider.index] = t;
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

    namespace Internal {

        void eval(const std::size_t idx) {}

    }

    void step() {

    }


} // namespace Physics