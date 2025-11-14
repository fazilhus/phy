#pragma once
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

        std::vector<Primitive> primitives;

        bool intersect(const Ray& r, HitInfo& hit) const;
    };

    struct AABB {
        glm::vec3 min_bound{FLT_MAX}, max_bound{-FLT_MAX};

        void grow(const glm::vec3& p);

        bool intersect(const Ray& r, HitInfo& hit, const glm::mat4& trans, const glm::vec3& inv_dir) const;
    };

    struct ColliderMeshes {
        std::vector<AABB> simple;
        std::vector<ColliderMesh> complex;
    };

    ColliderMeshes& get_collider_meshes();

    ColliderMeshId load_collider_mesh(const std::string& filepath);

} // namespace Physics