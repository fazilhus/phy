#include "config.h"

#include "phy.h"

#include "core/idpool.h"
#include "core/math.h"
#include "physics/ray.h"
#include "physics/simplex.h"
#include "physics/physicsmesh.h"
#include "physics/physicsresource.h"



namespace Physics {
    static Util::IdPool<ColliderId> collider_id_pool;
    static Colliders colliders_;
    static auto sort_axis = 0;

    State::Dyn& State::Dyn::set_pos(const glm::vec3& p) {
        this->pos = p;
        return *this;
    }

    State::Dyn& State::Dyn::set_vel(const glm::vec3& v) {
        this->vel = v;
        return *this;
    }

    State::Dyn& State::Dyn::set_rot(const glm::quat& r) {
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
                return glm::inverse(
                    glm::mat3(
                        {
                            {1.0f / 12.0f * m * (cm.height * cm.height + cm.depth * cm.depth), 0, 0},
                            {0, 1.0f / 12.0f * m * (cm.width * cm.width + cm.depth * cm.depth), 0},
                            {0, 0, 1.0f / 12.0f * m * (cm.width * cm.width + cm.height * cm.height)}
                        }
                        )
                    );
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
                return {};
            }
        }

    }


    ColliderId create_collider(
        ColliderMeshId cm_id, const glm::vec3& orig, const glm::vec3& translation, const glm::quat& rotation,
        const float mass, const ShapeType type
        ) {
        ColliderId id;
        const auto mat = glm::translate(translation) * glm::mat4(rotation);
        if (collider_id_pool.Allocate(id)) {
            colliders_.meshes.emplace_back(cm_id);
            colliders_.transforms.emplace_back(mat);
            colliders_.aabbs.emplace_back(get_collider_meshes().simple[cm_id.index]);
            State s;
            s.set_mass(mass).set_drag(0.99f).set_orig(orig);
            s.set_inertia_tensor(
                Internal::create_inertia_tensor(type, s.mass, get_collider_meshes().complex[cm_id.index])
                );
            s.dyn.set_pos(translation);
            s.dyn.set_rot(rotation);
            colliders_.states.emplace_back(s);
        }
        else {
            colliders_.meshes[id.index] = cm_id;
            colliders_.transforms[id.index] = mat;
            colliders_.aabbs[id.index] = get_collider_meshes().simple[cm_id.index];
            auto& s = colliders_.states[id.index];
            s.set_mass(mass).set_drag(0.99f).set_orig(orig);
            s.set_inertia_tensor(
                Internal::create_inertia_tensor(type, s.mass, get_collider_meshes().complex[cm_id.index])
                );
            s.dyn.set_pos(translation);
            s.dyn.set_rot(rotation);
        }
        return id;
    }

    void set_transform(const ColliderId collider, const glm::mat4& t) {
        assert(collider_id_pool.IsValid(collider));
        colliders_.transforms[collider.index] = t;
    }

    bool cast_ray(const Ray& ray, HitInfo& hit) {
        std::vector<HitInfo> aabb_hits;
        for (uint32_t i = 0; i < colliders_.meshes.size(); ++i) {
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
                inv_t * glm::vec4(ray.orig, 1.0f), Math::safe_normal(inv_t * glm::vec4(ray.dir, 0.0f))
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
                    best_hit.norm = t * glm::vec4(best_hit.local_norm, 0.0f);
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

    void add_center_impulse(const ColliderId collider, const glm::vec3& dir) {
        auto& state = colliders_.states[collider.index];
        state.dyn.impulse_accum += dir;
    }

    void add_impulse(const ColliderId collider, const glm::vec3& loc, const glm::vec3& dir) {
        auto& state = colliders_.states[collider.index];
        state.dyn.impulse_accum += dir;

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

            const auto impulse = state.dyn.impulse_accum * state.inv_mass;
            state.dyn.vel += impulse;
            state.dyn.pos += state.dyn.vel * dt;

            state.dyn.angular_vel += glm::quat(0.0f, inv_inertia_tensor * state.dyn.torque_accum);
            state.dyn.rot = glm::normalize(state.dyn.rot + 0.5f * state.dyn.angular_vel * state.dyn.rot * dt);

            state.dyn.impulse_accum = glm::vec3(0);
            state.dyn.torque_accum = glm::vec3(0);

            colliders_.transforms[i] = glm::translate(state.dyn.pos) * glm::mat4_cast(state.dyn.rot);
        }
    }

    void update_aabbs() {
        for (std::size_t i = 0; i < colliders_.aabbs.size(); ++i) {
            const auto cmid = colliders_.meshes[i];
            auto& aabb = colliders_.aabbs[i];
            aabb = rotate_aabb_affine(get_collider_meshes().simple[cmid.index], colliders_.transforms[i]);
        }
    }

    void sort_and_sweep(std::vector<AABBPair>& aabb_pairs) {
        aabb_pairs.clear();
        std::vector<uint32_t> indices(colliders_.aabbs.size());
        for (uint32_t i = 0; i < indices.size(); ++i) { indices[i] = i; }

        std::ranges::sort(
            indices,
            [&](const uint32_t a, const uint32_t b)-> bool {
                const auto min_a = colliders_.aabbs[a].min_bound[sort_axis];
                const auto min_b = colliders_.aabbs[b].min_bound[sort_axis];
                return min_a < min_b;
            }
            );

        auto sum = glm::vec3(0), sum2 = glm::vec3(0);
        glm::vec3 v;
        for (std::size_t ii = 0; ii < indices.size(); ++ii) {
            const auto i = indices[ii];
            const auto a = colliders_.aabbs[i];
            const auto point = 0.5f * (a.min_bound + a.max_bound);
            sum += point;
            sum2 += point * point;

            for (std::size_t jj = ii + 1; jj < indices.size(); ++jj) {
                const auto j = indices[jj];
                const auto b = colliders_.aabbs[j];
                if (b.min_bound[sort_axis] > a.max_bound[sort_axis]) {
                    break;
                }
                if (a.intersect(b)) {
                    aabb_pairs.emplace_back(AABBPair{ColliderId(i), ColliderId(j)});
                }
            }

            v = sum2 - sum * sum / static_cast<float>(indices.size());
            sort_axis = 0;
            if (v.y > v.x) sort_axis = 1;
            if (v.z > v.y) sort_axis = 2;
        }
    }

    SupportPoint support(const ColliderId a_id, const ColliderId b_id, const glm::vec3& dir) {
        const auto& collider_a = get_collider_meshes().complex[colliders_.meshes[a_id.index].index];
        const auto& collider_b = get_collider_meshes().complex[colliders_.meshes[b_id.index].index];
        const auto& transform_a = colliders_.transforms[a_id.index];
        const auto& transform_b = colliders_.transforms[b_id.index];
        const auto a = collider_a.furthest_along(transform_a, dir);
        const auto b = collider_b.furthest_along(transform_b, -dir);
        return { a - b, a, b };
    }

    bool gjk(const ColliderId a_id, const ColliderId b_id, Simplex& out_simplex) {
        auto s = support(a_id, b_id, glm::vec3(1, 0, 0));
        out_simplex = {};
        out_simplex.add_point(s);
        auto dir = -s.point;
        for (auto i = 0; i < 64; ++i) {
            s = support(a_id, b_id, dir);
            out_simplex.add_point(s);
            if (glm::dot(out_simplex[0].point, dir) < epsilon_f) { return false; }
            if (next_simplex(out_simplex, dir)) { return true; }
        }
        return false;
    }

    struct Face {
        union {
            SupportPoint v[3];
            struct {
                SupportPoint v0, v1, v2;
            };
        };
        glm::vec3 normal;
    };

    struct Edge {
        union {
            SupportPoint v[2];
            struct {
                SupportPoint v0, v1;
            };
        };
    };

    CollisionInfo epa(const Simplex& simplex, const ColliderId a_id, const ColliderId b_id) {
        constexpr auto max_iterations = 64;
        constexpr auto max_faces = 64;
        constexpr auto max_edges = 32;

        std::vector<Face> faces;
        faces.emplace_back(Face{
            .v0 = simplex[0],
            .v1 = simplex[1],
            .v2 = simplex[2],
            .normal = glm::normalize(glm::cross(simplex[1].point - simplex[0].point, simplex[2].point - simplex[0].point))
        });
        faces.emplace_back(Face{
            .v0 = simplex[0],
            .v1 = simplex[2],
            .v2 = simplex[3],
            .normal = glm::normalize(glm::cross(simplex[2].point - simplex[0].point, simplex[3].point - simplex[0].point))
        });
        faces.emplace_back(Face{
            .v0 = simplex[0],
            .v1 = simplex[3],
            .v2 = simplex[1],
            .normal = glm::normalize(glm::cross(simplex[3].point - simplex[0].point, simplex[1].point - simplex[0].point))
        });
        faces.emplace_back(Face{
            .v0 = simplex[1],
            .v1 = simplex[3],
            .v2 = simplex[2],
            .normal = glm::normalize(glm::cross(simplex[3].point - simplex[1].point, simplex[2].point - simplex[1].point))
        });
        std::vector<Edge> loose_edges;

        std::size_t min_face{};
        auto min_dist{max_f};

        CollisionInfo ret{};

        for (auto i = 0; i < max_iterations; ++i) {
            min_dist = glm::dot(faces[0].v0.point, faces[0].normal);
            min_face = 0;
            for (auto j = 0; j < faces.size(); ++j) {
                if (const auto dist = glm::dot(faces[j].v0.point, faces[j].normal);
                    dist < min_dist) {
                    min_dist = dist;
                    min_face = j;
                }
            }

            const auto dir = faces[min_face].normal;
            const auto s = support(a_id, b_id, dir);

            if (glm::dot(s.point, dir) - min_dist < epsilon_f) {
                ret.normal = -faces[min_face].normal * glm::dot(s.point, dir);
                break;
            }

            loose_edges.clear();
            for (auto j = 0; j < faces.size(); ++j) {
                if (glm::dot(faces[j].v0.point, faces[j].normal) < epsilon_f) {
                    j++;
                    continue;
                }

                for (auto k = 0; k < 3; ++k) {
                    Edge curr_edge{};
                    curr_edge.v0 = faces[j].v[k];
                    curr_edge.v1 = faces[j].v[(k + 1) % 3];
                    auto found_edge{false};

                    for (auto& it : loose_edges) {
                        if (it.v0.point == curr_edge.v0.point && it.v1.point == curr_edge.v1.point) {
                            it.v0 = loose_edges.back().v0;
                            it.v1 = loose_edges.back().v1;
                            loose_edges.pop_back();
                            found_edge = true;
                            break;
                        }
                    }

                    if (!found_edge) {
                        if (loose_edges.size() > max_edges) {
                            break;
                        }

                        loose_edges.emplace_back(curr_edge);
                    }
                }

                faces[j] = faces.back();
                faces.pop_back();
            }

            for (auto& it : loose_edges) {
                if (faces.size() > max_faces) {
                    break;
                }

                auto new_face = Face{
                    .v0 = it.v0,
                    .v1 = it.v1,
                    .v2 = s,
                    .normal = glm::normalize(glm::cross(it.v0.point - it.v1.point, it.v0.point - s.point))
                };
                if (glm::dot(new_face.v0.point, new_face.normal) < epsilon_f) {
                    const auto tmp = new_face.v0;
                    new_face.v0 = new_face.v1;
                    new_face.v1 = tmp;
                    new_face.normal *= -1.0f;
                }
                faces.push_back(new_face);
            }
        }

        ret.normal = -faces[min_face].normal * glm::dot(faces[min_face].v0.point, faces[min_face].normal);
        const auto& s0 = faces[min_face].v0;
        const auto& s1 = faces[min_face].v1;
        const auto& s2 = faces[min_face].v2;
        const auto a = s0.point;
        const auto b = s1.point;
        const auto c = s2.point;

        const auto ab = b - a;
        const auto ac = c - a;
        const auto ao = -a;

        const auto abab = glm::dot(ab, ab);
        const auto abac = glm::dot(ab, ac);
        const auto acac = glm::dot(ac, ac);
        const auto aoab = glm::dot(ao, ab);
        const auto aoac = glm::dot(ao, ac);

        const auto denom = abac * abac - abab * acac;
        auto s = (abac * aoac - acac * aoab) / denom;
        auto t = (abac * aoab - abab * aoac) / denom;

        s = Math::clamp(s, 0.0f, 1.0f);
        t = Math::clamp(t, 0.0f, 1.0f);
        if (const auto sum = s + t;
            sum - 1.0f > epsilon_f) {
            s /= sum;
            t /= sum;
            }
        const auto r = 1.0f - s - t;

        ret.contact_point_a = r * s0.a + s * s1.a + t * s2.a;
        ret.contact_point_b = r * s0.b + s * s1.b + t * s2.b;
        ret.contact_point = 0.5f * (ret.contact_point_a + ret.contact_point_b);
        ret.penetration_depth = min_dist;
        ret.has_collision = true;

        if (min_dist == max_f) {
            std::cout << "aaaaaaaaaa\n";
        }

        return ret;
    }

    void collision_solver(const CollisionInfo& ci, ColliderId a_id, ColliderId b_id) {

    }


} // namespace Physics
