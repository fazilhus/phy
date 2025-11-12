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

            glm::vec3 norm;

            bool intersect(const Ray& r, HitInfo& hit) const;
        };

        struct Primitive {
            std::vector<Triangle> triangles;
        };

        std::vector<Primitive> primitives;
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

    struct Colliders {
        std::vector<ColliderMeshId> meshes;
        std::vector<glm::mat4> transforms;
    };

    const Colliders& get_colliders();
    const ColliderMeshes& get_collider_meshes();

    ColliderId create_collider(ColliderMeshId cm_id, const glm::mat4& t);
    ColliderMeshId load_collider_mesh(const std::string& filepath);
    void set_transform(ColliderId collider, const glm::mat4& t);

    bool cast_ray(const Ray& ray, HitInfo& hit);
    bool cast_ray(const glm::vec3& start, const glm::vec3& dir, HitInfo& hit);

} // namespace Physics