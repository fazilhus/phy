#include "config.h"

#include "phy.h"

#include "physicsmesh.h"
#include "core/idpool.h"
#include "core/math.h"
#include "physics/ray.h"
#include "physics/simplex.h"


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
        const ShapeType type
        ) {
        ColliderId id;
        const auto mat = glm::translate(translation) * glm::mat4(rotation);
        if (collider_id_pool.Allocate(id)) {
            colliders_.meshes.emplace_back(cm_id);
            colliders_.transforms.emplace_back(mat);
            colliders_.aabbs.emplace_back(get_collider_meshes().simple[cm_id.index]);
            State s;
            s.set_mass(1.0f).set_drag(0.99f).set_orig(orig);
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
            s.set_mass(1.0f).set_drag(0.99f).set_orig(orig);
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

    void add_force(const ColliderId collider, const glm::vec3& f) {
        auto& state = colliders_.states[collider.index];
        state.dyn.force_accum += f;
    }

    void add_impulse(const ColliderId collider, const glm::vec3& loc, const glm::vec3& dir) {
        auto& state = colliders_.states[collider.index];
        state.dyn.impulse_accum += 0.01f * dir;

        const auto r = loc - state.dyn.pos;

        state.dyn.torque_accum += glm::quat(
            0.0f, glm::cross(
                r,
                dir
                )
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

            state.dyn.angular_vel += glm::quat(inv_inertia_tensor) * state.dyn.torque_accum * dt;
            state.dyn.rot = glm::normalize(state.dyn.rot + 0.5f * state.dyn.angular_vel * state.dyn.rot * dt);

            state.dyn.force_accum = glm::vec3(0);
            state.dyn.impulse_accum = glm::vec3(0);
            state.dyn.torque_accum = glm::quat();

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
            if (v[1] > v[0]) sort_axis = 1;
            if (v[2] > v[1]) sort_axis = 2;
        }
    }

    SupportPoint support(const ColliderId a_id, const ColliderId b_id, const glm::vec3& dir) {
        const auto& collider_a = get_collider_meshes().complex[colliders_.meshes[a_id.index].index];
        const auto& collider_b = get_collider_meshes().complex[colliders_.meshes[b_id.index].index];
        const auto& transform_a = colliders_.transforms[a_id.index];
        const auto& transform_b = colliders_.transforms[b_id.index];
        const auto n_dir = glm::normalize(dir);
        const auto a = collider_a.furthest_along(transform_a, n_dir);
        const auto b = collider_b.furthest_along(transform_b, -n_dir);
        return { a - b, a, b };
    }

    bool gjk(const ColliderId a_id, const ColliderId b_id, Simplex& out_simplex) {
        auto s = support(a_id, b_id, glm::vec3(1, 0, 0));
        out_simplex = {};
        out_simplex.add_point(s);
        auto dir = -s.point;
        for (std::size_t i = 0; i < 65; ++i) {
            s = support(a_id, b_id, dir);
            if (glm::dot(s.point, dir) < epsilon) { return false; }
            out_simplex.add_point(s);
            if (next_simplex(out_simplex, dir)) { return true; }
        }
        return false;
    }

    std::pair<std::vector<glm::vec4>, std::size_t> face_normals(
        const std::vector<SupportPoint>& polytope, const std::vector<std::size_t>& faces
        ) {
        std::vector<glm::vec4> normals;
        std::size_t min_tri = 0;
        auto min_dist = FLT_MAX;

        for (std::size_t i = 0; i < faces.size(); i += 3) {
            const auto a = polytope[faces[i + 0]].point;
            const auto b = polytope[faces[i + 1]].point;
            const auto c = polytope[faces[i + 2]].point;
            auto norm = glm::normalize(glm::cross(b - a, c - a));
            auto dist = glm::dot(norm, a);

            if (dist < epsilon) {
                norm *= -1.0f;
                dist *= -1.0f;
            }

            normals.emplace_back(norm, dist);

            if (dist < min_dist) {
                min_dist = dist;
                min_tri = i / 3;
            }
        }

        return { normals, min_tri };
    }

    struct Edge {
        std::size_t a, b;

        bool operator==(const Edge& other) const {
            return a == other.a && b == other.b || a == other.b && b == other.a;
        }
    };

    void add_if_unique_edge(
        std::vector<Edge>& edges, const std::vector<std::size_t>& faces, const std::size_t a, const std::size_t b
        ) {
        const auto reverse = std::ranges::find(
            edges
            ,
            Edge{faces[a], faces[b]}
            );

        if (reverse != edges.end()) {
            edges.erase(reverse);
        }
        else {
            edges.emplace_back(faces[a], faces[b]);
        }
    }

    CollisionInfo epa(const Simplex& simplex, const ColliderId a_id, const ColliderId b_id) {
        std::vector<SupportPoint> polytope(simplex.begin(), simplex.end());
        std::vector<size_t> faces = {
            0, 1, 2,
            0, 3, 1,
            0, 2, 3,
            1, 3, 2
        };
        auto [normals, min_face] = face_normals(polytope, faces);
        glm::vec3 min_norm;
        auto min_dist{FLT_MAX};

        std::vector<Edge> unique_edges;
        std::vector<std::size_t> new_faces;

        while (min_dist == FLT_MAX) {
            min_norm = xyz(normals[min_face]);
            min_dist = normals[min_face].w;

            const auto s = support(a_id, b_id, min_norm);
            const auto s_dist = glm::dot(min_norm, s.point);

            if (fabs(s_dist - min_dist) > epsilon) {
                min_dist = FLT_MAX;
            }

            unique_edges.clear();
            for (std::size_t i = 0; i < normals.size(); ++i) {
                const auto dns = glm::dot(normals[i], glm::vec4(s.point, 1.0f));
                const auto dnf = glm::dot(normals[i], glm::vec4(polytope[faces[i * 3]].point, 1.0f));
                if (dns > dnf) {
                    const auto f = i * 3;
                    add_if_unique_edge(unique_edges, faces, f + 0, f + 1);
                    add_if_unique_edge(unique_edges, faces, f + 1, f + 2);
                    add_if_unique_edge(unique_edges, faces, f + 2, f + 0);

                    faces[f + 2] = faces.back();
                    faces.pop_back();
                    faces[f + 1] = faces.back();
                    faces.pop_back();
                    faces[f + 0] = faces.back();
                    faces.pop_back();

                    normals[i] = normals.back();
                    normals.pop_back();

                    i--;
                }
            }

            new_faces.clear();
            for (auto [e1, e2] : unique_edges) {
                new_faces.push_back(e1);
                new_faces.push_back(e2);
                new_faces.push_back(polytope.size());
            }

            polytope.push_back(s);
            auto [new_normals, new_min_face] = face_normals(polytope, new_faces);

            auto old_min_distance{FLT_MAX};
            for (std::size_t i = 0; i < normals.size(); i++) {
                if (normals[i].w < old_min_distance) {
                    old_min_distance = normals[i].w;
                    min_face = i;
                }
            }

            if (new_normals.empty()) {
                return {};
            }
            if (new_normals[new_min_face].w < old_min_distance) { min_face = new_min_face + normals.size(); }

            faces.insert(faces.end(), new_faces.begin(), new_faces.end());
            normals.insert(normals.end(), new_normals.begin(), new_normals.end());
        }

        const auto& s0 = polytope[faces[min_face * 3]];
        const auto& s1 = polytope[faces[min_face * 3 + 1]];
        const auto& s2 = polytope[faces[min_face * 3 + 2]];
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
            sum - 1.0f > epsilon) {
            s /= sum;
            t /= sum;
        }
        const auto r = 1.0f - s - t;

        CollisionInfo ret;
        ret.contact_point_a = r * s0.a + s * s1.a + t * s2.a;
        ret.contact_point_b = r * s0.b + s * s1.b + t * s2.b;
        ret.contact_point = 0.5f * (ret.contact_point_a + ret.contact_point_b);
        ret.normal = min_norm;
        ret.penetration_depth = min_dist + epsilon;
        ret.has_collision = true;

        return ret;
    }


} // namespace Physics
