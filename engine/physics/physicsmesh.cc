#include "config.h"
#include "physicsmesh.h"

#include "plane.h"
#include "core/idpool.h"
#include "core/math.h"
#include "fx/gltf.h"


namespace Physics {

    static Util::IdPool<ColliderMeshId> collider_mesh_id_pool;
    static Util::IdPool<ColliderId> collider_id_pool;

    static Colliders colliders;
    static ColliderMeshes collider_meshes;


    namespace Internal {

        template <typename CompType>
        void LoadColliderMeshPrimitive(
            const fx::gltf::Document& doc, AABB* aabb, ColliderMesh* mesh, std::size_t prim_n
            ) {
            const auto& prim = doc.meshes[0].primitives[prim_n];

            const auto& ib_access = doc.accessors[prim.indices];
            const auto& ib_view = doc.bufferViews[ib_access.bufferView];
            const auto& ib = doc.buffers[ib_view.buffer];

            const auto& vb_access = doc.accessors[prim.attributes.find("POSITION")->second];
            const auto& vb_view = doc.bufferViews[vb_access.bufferView];
            const auto& vb = doc.buffers[vb_view.buffer];

            const auto num_indices = ib_access.count;
            const auto ibuf = reinterpret_cast<const CompType*>(&ib.data[ib_access.byteOffset + ib_view.byteOffset]);
            const auto vbuf = reinterpret_cast<const float*>(&vb.data[vb_access.byteOffset + vb_view.byteOffset]);

            const auto dim = (vb_access.type == fx::gltf::Accessor::Type::Vec3) ? 3 : 4;
            for (auto i = 0; i < num_indices; i += 3) {
                ColliderMesh::Triangle t;
                t.v0 = glm::vec3(
                    vbuf[dim * ibuf[i]],
                    vbuf[dim * ibuf[i] + 1],
                    vbuf[dim * ibuf[i] + 2]
                    );
                t.v1 = glm::vec3(
                    vbuf[dim * ibuf[i + 1]],
                    vbuf[dim * ibuf[i + 1] + 1],
                    vbuf[dim * ibuf[i + 1] + 2]
                    );
                t.v2 = glm::vec3(
                    vbuf[dim * ibuf[i + 2]],
                    vbuf[dim * ibuf[i + 2] + 1],
                    vbuf[dim * ibuf[i + 2] + 2]
                    );

                t.norm = glm::cross(
                    t.v0 - t.v1,
                    t.v0 - t.v2
                    );
                mesh->primitives[prim_n].triangles.emplace_back(t);
            }

            aabb->grow(glm::vec3(vb_access.min[0], vb_access.min[1], vb_access.min[2]));
            aabb->grow(glm::vec3(vb_access.max[0], vb_access.max[1], vb_access.max[2]));
        }

    }


    bool ColliderMesh::Triangle::intersect(const Ray& r, HitInfo& hit) const {
        const auto e1 = this->v1 - this->v0;
        const auto e2 = this->v2 - this->v0;
        const auto re2n = glm::cross(r.dir, e2);

        const auto det = glm::dot(e1, re2n);
        if (std::fabs(det) < epsilon) { return false; }

        const auto inv_det = 1.0f / det;
        const auto s = r.orig - this->v0;
        const auto u = inv_det * glm::dot(s, re2n);

        if ((u < 0 && abs(u) > epsilon) || (u > 1 && abs(u - 1) > epsilon)) { return false; }

        const auto se1n = glm::cross(s, e1);
        const auto v = inv_det * glm::dot(r.dir, se1n);

        if ((v < 0 && abs(v) > epsilon) || (u + v > 1 && abs(u + v - 1) > epsilon)) { return false; }

        const auto t = inv_det * glm::dot(e2, se1n);

        hit.t = t;
        hit.pos = r.orig + r.dir * t;

        return true;
    }

    bool ColliderMesh::intersect(const Ray& r, HitInfo& hit) const {
        for (std::size_t i = 0; i < this->primitives.size(); ++i) {
            const auto& p = this->primitives[i];
            for (std::size_t j = 0; j < p.triangles.size(); ++j) {
                const auto& t = p.triangles[j];
                HitInfo temp_hit;
                if (t.intersect(r, temp_hit) && temp_hit.t < hit.t) {
                    hit = temp_hit;
                    hit.prim_n = i;
                    hit.tri_n = j;
                }
            }
        }

        return hit.hit();
    }

    void AABB::grow(const glm::vec3& p) {
        this->min_bound = glm::min(this->min_bound, p);
        this->max_bound = glm::max(this->max_bound, p);
    }

    bool AABB::intersect(const Ray& r, HitInfo& hit, const glm::mat4& trans, const glm::vec3& inv_dir) const {
        const glm::vec3 wmi = trans * glm::vec4(this->min_bound, 1.0f);
        const glm::vec3 wma = trans * glm::vec4(this->max_bound, 1.0f);
        auto tmin{0.0f}, tmax{FLT_MAX};

        for (auto i = 0; i < 3; ++i) {
            auto t1 = (wmi[i] - r.orig[i]) * inv_dir[i];
            auto t2 = (wma[i] - r.orig[i]) * inv_dir[i];
            if (inv_dir[i] < 0.0f) { std::swap(t1, t2); }
            tmin = Math::max(tmin, Math::min(t1, t2));
            tmax = Math::min(tmax, Math::max(t1, t2));
        }

        hit.t = (tmin < tmax && tmax > 0.0f) ? tmin : tmax;
        return tmin < tmax && tmax > 0.0f;
    }

    ColliderMeshId load_collider_mesh(const std::string& filepath) {
        ColliderMeshId mesh_id;
        AABB* aabb;
        ColliderMesh* mesh;
        if (collider_mesh_id_pool.Allocate(mesh_id)) {
            collider_meshes.simple.emplace_back();
            collider_meshes.complex.emplace_back();
        }
        aabb = &collider_meshes.simple[mesh_id.index];
        mesh = &collider_meshes.complex[mesh_id.index];

        fx::gltf::Document doc;
        try {
            if (filepath.ends_with("glb")) { doc = fx::gltf::LoadFromBinary(filepath); }
            else { doc = fx::gltf::LoadFromText(filepath); }
        }
        catch (const std::exception& err) {
            printf(err.what());
#if _DEBUG
            assert(false);
#endif
            collider_mesh_id_pool.Deallocate(mesh_id);
            return {};
        }

        const auto primitive_count = doc.meshes[0].primitives.size();
        mesh->primitives.resize(primitive_count);
        for (std::size_t i = 0; i < primitive_count; ++i) {
            const auto& prim = doc.meshes[0].primitives[i];
            const auto ib_accessor = doc.accessors[prim.indices];
            mesh->primitives[i].triangles.reserve(ib_accessor.count / 3);
        }

        for (std::size_t i = 0; i < primitive_count; ++i) {
            const auto& prim = doc.meshes[0].primitives[i];
            switch (doc.accessors[prim.indices].componentType) {
            case fx::gltf::Accessor::ComponentType::Byte:
                Internal::LoadColliderMeshPrimitive<int8_t>(doc, aabb, mesh, i);
                break;
            case fx::gltf::Accessor::ComponentType::UnsignedByte:
                Internal::LoadColliderMeshPrimitive<uint8_t>(doc, aabb, mesh, i);
                break;
            case fx::gltf::Accessor::ComponentType::Short:
                Internal::LoadColliderMeshPrimitive<int16_t>(doc, aabb, mesh, i);
                break;
            case fx::gltf::Accessor::ComponentType::UnsignedShort:
                Internal::LoadColliderMeshPrimitive<uint16_t>(doc, aabb, mesh, i);
                break;
            case fx::gltf::Accessor::ComponentType::UnsignedInt:
                Internal::LoadColliderMeshPrimitive<uint32_t>(doc, aabb, mesh, i);
                break;
            default:
                assert(false); // not supported
                break;
            }
        }

        return mesh_id;
    }

    const Colliders& get_colliders() { return colliders; }

    const ColliderMeshes& get_collider_meshes() { return collider_meshes; }

    ColliderId create_collider(ColliderMeshId cm_id, const glm::mat4& t) {
        ColliderId id;
        if (collider_id_pool.Allocate(id)) {
            colliders.meshes.emplace_back(cm_id);
            colliders.transforms.emplace_back(t);
        }
        else {
            colliders.meshes[id.index] = cm_id;
            colliders.transforms[id.index] = t;
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
            const auto& aabb = collider_meshes.simple[cm.index];
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

        auto& cm = collider_meshes.complex[best_hit.mesh.index];
        const auto t = colliders.transforms[best_hit.collider.index];
        const auto inv_t = glm::inverse(t);
        const auto model_ray = Ray(inv_t * glm::vec4(ray.orig, 1.0f), inv_t * glm::vec4(ray.dir, 1.0f));

        for (auto& p : cm.primitives) {
            for (auto& tri : p.triangles) {
                tri.selected = false;
            }
        }

        cm.intersect(model_ray, hit);
        hit.collider = best_hit.collider;
        hit.mesh = best_hit.mesh;
        hit.pos = t * glm::vec4(hit.pos, 1.0f);
        if (hit.hit()) {
            cm.primitives[hit.prim_n].triangles[hit.tri_n].selected = true;
        }
        return hit.hit();
    }

    bool cast_ray(const glm::vec3& start, const glm::vec3& dir, HitInfo& hit) { return cast_ray(Ray(start, dir), hit); }

} // namespace Physics
