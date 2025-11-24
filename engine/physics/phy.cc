#include "config.h"

#include "phy.h"

#include "physicsmesh.h"
#include "core/idpool.h"
#include "core/math.h"
#include "core/util.h"
#include "physics/ray.h"


namespace Physics {

    static Util::IdPool<ColliderId> collider_id_pool;
    static Colliders colliders_;

    State::Dyn& State::Dyn::set_pos(const glm::vec3& p) {
        this->pos = p;
        return *this;
    }

    State::Dyn& State::Dyn::set_vel(const glm::vec3& v) {
        this->vel = v;
        return *this;
    }

    State::Dyn& State::Dyn::set_rot(const glm::vec3& r) {
        this->rot = r;
        return *this;
    }

    State& State::set_inertia_tensor(const glm::mat3& m) {
        this->inv_inertia_shape = m;
        return *this;
    }

    State& State::set_orig(const glm::vec3& o) {
        this->orig = o;
        return *this;
    }

    State& State::set_drag(float d) {
        this->drag = d;
        return *this;
    }

    State& State::set_mass(float m) {
        this->mass = m;
        this->inv_mass = 1.0f / m;
        return *this;
    }

    const Colliders& get_colliders() { return colliders_; }
    Colliders& colliders() { return colliders_; }

    namespace Internal {

        glm::mat3 create_inertia_tensor(const ShapeType type, const float m, const ColliderMesh& cm) {
            switch (type) {
            case ShapeType::Box:
                return glm::inverse(glm::mat3({
                    {1.0f / 12.0f * m * (cm.height * cm.height + cm.depth * cm.depth), 0, 0},
                    {0, 1.0f / 12.0f * m * (cm.width * cm.width + cm.depth * cm.depth), 0},
                    {0, 0, 1.0f / 12.0f * m * (cm.width * cm.width + cm.height * cm.height)}
                }));
                // case ShapeType::Sphere:
                //     return {
                //         {},
                //         {},
                //         {}
                //     };
            case ShapeType::Custom:
                return {
                    {},
                    {},
                    {}
                };
            default:
                assert(false && "unreachable");
            }
        }

    }

    ColliderId create_collider(ColliderMeshId cm_id, const glm::vec3& orig, const glm::mat4& t, const ShapeType type) {
        ColliderId id;
        if (collider_id_pool.Allocate(id)) {
            colliders_.meshes.emplace_back(cm_id);
            colliders_.transforms.emplace_back(t);
            colliders_.aabbs.emplace_back(get_collider_meshes().simple[cm_id.index]);
            State s;
            s.set_mass(1.0f).set_drag(0.99f).set_orig(orig);
            s.set_inertia_tensor(Internal::create_inertia_tensor(type, s.mass, get_collider_meshes().complex[cm_id.index]));
            s.dyn.set_pos(t[3]);
            colliders_.states.emplace_back(s);
        }
        else {
            colliders_.meshes[id.index] = cm_id;
            colliders_.transforms[id.index] = t;
            colliders_.aabbs[id.index] = get_collider_meshes().simple[cm_id.index];
            auto& s = colliders_.states[id.index];
            s.set_mass(1.0f).set_drag(0.99f).set_orig(orig);
            s.set_inertia_tensor(Internal::create_inertia_tensor(type, s.mass, get_collider_meshes().complex[cm_id.index]));
            s.dyn.set_pos(t[3]);
        }
        return id;
    }

    void set_transform(const ColliderId collider, const glm::mat4& t) {
        assert(collider_id_pool.IsValid(collider));
        colliders_.transforms[collider.index] = t;
    }

    bool cast_ray(const Ray& ray, HitInfo& hit) {
        std::vector<HitInfo> aabb_hits;
        for (std::size_t i = 0; i < colliders_.meshes.size(); ++i) {
            const auto c = ColliderId(i);
            const auto cm = colliders_.meshes[i];
            const auto& aabb = colliders_.aabbs[i];
            HitInfo temp_hit;
            if (aabb.intersect(ray, temp_hit)) {
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
                inv_t * glm::vec4(ray.orig, 1.0f), Math::safe_normal(glm::quat(inv_t) * ray.dir)
                );

            for (auto& p: cm.primitives) { for (auto& tri: p.triangles) { tri.selected = false; } }

            HitInfo temp_hit;
            if (cm.intersect(model_ray, temp_hit)) {
                if (temp_hit.t < best_hit.t) {
                    best_hit = temp_hit;
                    best_hit.collider = it.collider;
                    best_hit.mesh = it.mesh;
                    best_hit.local_dir = model_ray.dir;
                    best_hit.pos = t * glm::vec4(best_hit.local_pos, 1.0f);
                    best_hit.norm = t * glm::vec4(best_hit.local_norm, 1.0f);
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

    void add_force(const ColliderId collider, const glm::vec3& f) {
        auto& state = colliders_.states[collider.index];
        state.dyn.force_accum += f;
    }

    void add_impulse(const ColliderId collider, const glm::vec3& loc, const glm::vec3& dir) {
        auto& state = colliders_.states[collider.index];
        state.dyn.impulse_accum += 0.01f * dir;

        state.dyn.torque_accum += glm::cross(
            loc - state.dyn.pos,
            dir
            );
    }

    void step(const float dt) {
        for (std::size_t i = 0; i < colliders_.states.size(); ++i) {
            auto& state = colliders_.states[i];
            const auto rotm = glm::mat3_cast(state.dyn.rot);
            const auto inv_inertia_tensor = rotm * state.inv_inertia_shape * glm::transpose(rotm);

            const auto acc = state.dyn.force_accum * state.inv_mass;
            const auto impulse = state.dyn.impulse_accum * state.inv_mass;
            state.dyn.vel += impulse + acc * dt;
            state.dyn.pos += state.dyn.vel * dt;

            state.dyn.angular_vel += inv_inertia_tensor * state.dyn.torque_accum * dt;
            state.dyn.rot = glm::normalize(state.dyn.rot + 0.5f * glm::quat(0.0f, state.dyn.angular_vel) * state.dyn.rot * dt);

            state.dyn.force_accum = glm::vec3(0);
            state.dyn.impulse_accum = glm::vec3(0);
            state.dyn.torque_accum = glm::vec3(0);

            auto& t = colliders_.transforms[i];
            t = glm::translate(glm::mat4(1.0f), state.dyn.pos) * glm::mat4_cast(state.dyn.rot);
        }
    }

    void update_aabbs() {
        for (std::size_t i = 0; i < colliders_.aabbs.size(); ++i) {
            const auto cmid = colliders_.meshes[i];
            auto& aabb = colliders_.aabbs[i];
            aabb = rotate_aabb_affine(get_collider_meshes().simple[cmid.index], colliders_.transforms[i]);
        }
    }


} // namespace Physics
