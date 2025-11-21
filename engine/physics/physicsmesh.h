#pragma once
#include <array>

#include "phy.h"
#include "ray.h"
#include "vec3.hpp"


namespace Physics {

    struct ColliderMesh {
        struct Triangle {
            union {
                glm::vec3 vertices[3];

                struct {
                    glm::vec3 v0, v1, v2;
                };
            };

            glm::vec3 center;
            glm::vec3 norm;
            bool selected = false;

            bool intersect(const Ray& r, HitInfo& hit) const;
        };

        struct Primitive {
            std::vector<Triangle> triangles;
        };

        glm::vec3 center = glm::vec3(0);
        float radius = 0.0f;
        float width = 0.0f;
        float height = 0.0f;
        float depth = 0.0f;
        std::size_t num_of_vertices = 0;

        std::vector<Primitive> primitives;

        bool intersect(const Ray& r, HitInfo& hit) const;
    };

    struct AABB {
        union {
            glm::vec3 corners[2];
            struct {
                glm::vec3 min_bound, max_bound;
            };
        };

        explicit AABB() : min_bound(FLT_MAX), max_bound(-FLT_MAX) {}

        void grow(const glm::vec3& p);
        void grow_rot(const glm::mat4& t);

        bool intersect(const Ray& r, HitInfo& hit) const;
    };

    AABB rotate_aabb_affine(const AABB& orig, const glm::mat4& t);

    struct ColliderMeshes {
        std::vector<AABB> simple;
        std::vector<ColliderMesh> complex;
    };

    ColliderMeshes& get_collider_meshes();

    ColliderMeshId load_collider_mesh(const std::string& filepath);

} // namespace Physics