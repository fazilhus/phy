#pragma once
#include "phy.h"
#include "ray.h"
#include "vec3.hpp"


namespace Physics {

    struct ColliderId {
        uint32_t index: 22; // 4M concurrent colliders
        uint32_t generation: 10; // 1024 generations per index

        constexpr static ColliderId Create(uint32_t id) {
            ColliderId ret{id & 0x003FFFFF, (id & 0xFFC00000) >> 22};
            return ret;
        }

        static ColliderId Create(uint32_t index, uint32_t generation) {
            ColliderId ret;
            ret.index = index;
            ret.generation = generation;
            return ret;
        }

        explicit constexpr operator uint32_t() const {
            return ((generation << 22) & 0xFFC00000ul) + (index & 0x003FFFFFul);
        }

        static constexpr ColliderId Invalid() { return Create(0xFFFFFFFF); }
        constexpr uint32_t HashCode() const { return index; }
        const bool operator==(const ColliderId& rhs) const { return uint32_t(*this) == uint32_t(rhs); }
        const bool operator!=(const ColliderId& rhs) const { return uint32_t(*this) != uint32_t(rhs); }
        const bool operator<(const ColliderId& rhs) const { return index < rhs.index; }
        const bool operator>(const ColliderId& rhs) const { return index > rhs.index; }
    };

    struct ColliderMeshId {
        uint32_t index: 22; // 4M concurrent meshes
        uint32_t generation: 10; // 1024 generations per index

        constexpr static ColliderMeshId Create(uint32_t id) {
            ColliderMeshId ret{id & 0x003FFFFF, (id & 0xFFC00000) >> 22};
            return ret;
        }

        explicit constexpr operator uint32_t() const {
            return ((generation << 22) & 0xFFC00000ul) + (index & 0x003FFFFFul);
        }

        static constexpr ColliderMeshId Invalid() { return Create(0xFFFFFFFF); }
        constexpr uint32_t HashCode() const { return index; }
        const bool operator==(const ColliderMeshId& rhs) const { return uint32_t(*this) == uint32_t(rhs); }
        const bool operator!=(const ColliderMeshId& rhs) const { return uint32_t(*this) != uint32_t(rhs); }
        const bool operator<(const ColliderMeshId& rhs) const { return index < rhs.index; }
        const bool operator>(const ColliderMeshId& rhs) const { return index > rhs.index; }
    };

    struct ColliderMesh {
        struct Triangle {
            union {
                glm::vec3 vertices[3];

                struct {
                    glm::vec3 v0, v1, v2;
                };
            };

            glm::vec3 norm;
        };

        struct Primitive {
            std::vector<Triangle> triangles;
        };

        std::vector<Primitive> primitives;
    };

    struct AABB {
        glm::vec3 min_bound{FLT_MAX}, max_bound{-FLT_MAX};

        void grow(const glm::vec3& p);

        bool intersect(const Ray& r, HitInfo& hit) const;
    };

    struct ColliderMeshes {
        std::vector<AABB> simple;
        std::vector<ColliderMesh> complex;
    };

    struct Colliders {
        std::vector<ColliderMeshId> meshes;
        std::vector<glm::mat4> transforms;
    };

    const Colliders& get_colliders();
    const ColliderMeshes& get_collider_meshes();

    ColliderId create_collider(ColliderMeshId cm_id, const glm::mat4& t);
    ColliderMeshId load_collider_mesh(const std::string& filepath);
    void set_transform(ColliderId collider, const glm::mat4& t);

} // namespace Physics