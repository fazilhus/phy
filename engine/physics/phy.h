#pragma once
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

    struct HitInfo {
        glm::vec3 pos = glm::vec3(0, 0, 0);
        glm::vec3 norm = glm::vec3(0, 0, 0);
        float dist_sq = FLT_MAX;
        ColliderId collider;

        bool hit() const {
            return dist_sq < FLT_MAX;
        }
    };

    constexpr auto epsilon = std::numeric_limits<float>::epsilon();

} // namespace Physics
