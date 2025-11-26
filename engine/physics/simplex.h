#pragma once
#include <array>


namespace Physics {

    struct Simplex {
        std::size_t size;
        std::array<glm::vec3, 4> points;

        explicit Simplex() : size(0), points() {};

        Simplex& operator=(std::initializer_list<glm::vec3> init);

        void add_point(const glm::vec3& point);

        // auto begin() const { return this->points.begin(); }
        // auto end() const { return this->points.end() - (4 - this->size); }
    };

    bool next_simplex(Simplex& s, glm::vec3& dir);

} // namespace Physics