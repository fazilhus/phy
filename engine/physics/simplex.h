#pragma once
#include <array>


namespace Physics {

    struct Simplex {
    private:
        std::size_t m_size;
        std::array<glm::vec3, 4> m_points;

    public:
        explicit Simplex() : m_size(0), m_points() {};

        Simplex& operator=(const std::initializer_list<glm::vec3>& init);

        void add_point(const glm::vec3& point);

        const glm::vec3& operator[](int i) const { return this->m_points[i]; }
        glm::vec3& operator[](int i) { return this->m_points[i]; }
        size_t size() const { return this->m_size; }

        auto begin() const { return this->m_points.begin(); }
        auto end() const { return this->m_points.end() - (4 - this->m_size); }
    };

    bool next_simplex(Simplex& s, glm::vec3& dir);

} // namespace Physics