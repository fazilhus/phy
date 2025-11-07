#include "config.h"
#include "physicsmesh.h"

#include "core/idpool.h"
#include "fx/gltf.h"


namespace Physics {

    static Util::IdPool<ColliderMeshId> collider_mesh_id_pool;
    static Util::IdPool<ColliderId> collider_id_pool;

    static Colliders colliders;
    static std::vector<ColliderMeshId> collider_meshes;

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
            const auto vbuf = reinterpret_cast<const CompType*>(&vb.data[vb_access.byteOffset + vb_view.byteOffset]);

            const auto dim = (vb_access.type == fx::gltf::Accessor::Type::Vec3) ? 3 : 4;
            for (std::size_t i = 0; i < num_indices; ++i) {
                auto& it = mesh->primitives[prim_n].triangles[i];
                it.v0 = glm::vec3(
                    vbuf[dim * ibuf[i]],
                    vbuf[dim * ibuf[i] + 1],
                    vbuf[dim * ibuf[i] + 2]
                    );
                it.v1 = glm::vec3(
                    vbuf[dim * ibuf[i + 1]],
                    vbuf[dim * ibuf[i + 1] + 1],
                    vbuf[dim * ibuf[i + 1] + 2]
                    );
                it.v2 = glm::vec3(
                    vbuf[dim * ibuf[i + 2]],
                    vbuf[dim * ibuf[i + 2] + 1],
                    vbuf[dim * ibuf[i + 2] + 2]
                    );

                it.norm = glm::cross(
                    it.v0 - it.v1,
                    it.v0 - it.v2
                    );
            }

            aabb->grow(
                glm::vec3(vb_access.min[0], vb_access.min[1], vb_access.min[2]),
                glm::vec3(vb_access.max[0], vb_access.max[1], vb_access.max[2])
                );
        }

    }


    void AABB::grow(const glm::vec3& new_min_bound, const glm::vec3& new_max_bound) {
        this->min_bound = glm::min(this->min_bound, new_min_bound);
        this->max_bound = glm::max(this->max_bound, new_max_bound);
    }

    bool AABB::intersect(const Ray& r, HitInfo& hit) const { return {}; }

    ColliderMeshId load_collider_mesh(const std::string& filepath) {
        ColliderMeshId mesh_id;
        AABB* aabb;
        ColliderMesh* mesh;
        if (collider_mesh_id_pool.Allocate(mesh_id)) {
            colliders.simple.emplace_back();
            colliders.complex.emplace_back();
        }
        aabb = &colliders.simple[mesh_id.index];
        mesh = &colliders.complex[mesh_id.index];

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
            mesh->primitives[i].triangles.resize(ib_accessor.count);
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

    const Colliders& get_colliders() {
        return colliders;
    }

    ColliderId create_collider(ColliderMeshId cm_id, const glm::mat4& t) {
        ColliderId id;
        if (collider_id_pool.Allocate(id)) {
            collider_meshes.emplace_back(cm_id);
            colliders.transforms.emplace_back(t);
        } else {
            collider_meshes[id.index] = cm_id;
            colliders.transforms[id.index] = t;
        }
        return id;
    }

    void set_transform(const ColliderId collider, const glm::mat4& t) {
        assert(collider_id_pool.IsValid(collider));
        colliders.transforms[collider.index] = t;
    }

} // namespace Physics
