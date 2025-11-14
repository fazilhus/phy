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
        HitInfo best_hit;
        const auto inv_dir = 1.0f / ray.dir;
        for (std::size_t i = 0; i < colliders.meshes.size(); ++i) {
            const auto c = ColliderId(i);
            const auto cm = colliders.meshes[i];
            const auto& aabb = get_collider_meshes().simple[cm.index];
            HitInfo temp_hit;
            if (aabb.intersect(ray, temp_hit, colliders.transforms[i], inv_dir)) {
                if (temp_hit.t < best_hit.t) {
                    best_hit = temp_hit;
                    best_hit.collider = c;
                    best_hit.mesh = cm;
                }
            }
        }

        if (!best_hit.hit()) { return false; }

        auto& cm = get_collider_meshes().complex[best_hit.mesh.index];
        const auto t = colliders.transforms[best_hit.collider.index];
        const auto inv_t = glm::inverse(t);
        const auto model_ray = Ray(inv_t * glm::vec4(ray.orig, 1.0f), Math::safe_normal(glm::quat(inv_t) * glm::vec4(ray.dir, 1.0f)));

        for (auto& p : cm.primitives) {
            for (auto& tri : p.triangles) {
                tri.selected = false;
            }
        }

        if (cm.intersect(model_ray, hit)) {
            hit.collider = best_hit.collider;
            hit.mesh = best_hit.mesh;
            hit.pos = t * glm::vec4(hit.pos, 1.0f);
            cm.primitives[hit.prim_n].triangles[hit.tri_n].selected = true;
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