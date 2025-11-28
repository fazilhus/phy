#include "config.h"
#include "simplex.h"

#include "core/math.h"


namespace Physics {

    Simplex& Simplex::operator=(std::initializer_list<glm::vec3> init) {
        this->size = 0;
        for (const auto& p : init) {
            points[this->size++] = p;
        }
        return *this;
    }

    void Simplex::add_point(const glm::vec3& point) {
        this->points = {point, this->points[0], this->points[1], this->points[2]};
        this->size = Math::min(this->size + 1, 4);
    }

    bool same_dir(const glm::vec3& a, const glm::vec3& b) {
        return glm::dot(a, b) > 0;
    }

    bool simplex_line(Simplex& s, glm::vec3& dir) {
        const auto a = s.points[0];
        const auto b = s.points[1];

        const auto ab = b - a;
        const auto ao = -a;

        if (same_dir(ab, ao)) {
            s = { a, b };
            dir = glm::cross(glm::cross(ab, ao), ab);
        }
        else {
            s = { a };
            dir = ao;
        }
        return false;
    }

    bool simplex_triangle(Simplex& s, glm::vec3& dir) {
        const auto a = s.points[0];
        const auto b = s.points[1];
        const auto c = s.points[2];

        const auto ab = b - a;
        const auto ac = c - a;
        const auto ao = -a;

        const auto abc = glm::cross(ab, ac);

        if (same_dir(abc, ac)) {
            if (same_dir(ac, ao)) {
                s = { a, c };
                dir = glm::cross(glm::cross(ac, ao), ac);
            } else {
                if (same_dir(ab, ao)) {
                    s = { a, b };
                    dir = glm::cross(glm::cross(ab, ao), ab);
                }
                else {
                    s = { a };
                    dir = ao;
                }
            }
        } else {
            if (same_dir(ab, abc)) {
                if (same_dir(ab, ao)) {
                    s = { a, b };
                    dir = glm::cross(glm::cross(ab, ao), ab);
                }
                else {
                    s = { a };
                    dir = ao;
                }
            } else {
                if (same_dir(abc, ao)) {
                    s = { a, b, c };
                    dir = abc;
                } else {
                    s = { a, c, b };
                    dir = -abc;
                }
            }
        }
        return false;
    }

    bool simplex_tetrahedron(Simplex& s, glm::vec3& dir) {
        // TODO can be optimized to use less checks
        const auto a = s.points[0];
        const auto b = s.points[1];
        const auto c = s.points[2];
        const auto d = s.points[3];

        const auto ab = b - a;
        const auto ac = c - a;
        const auto ad = d - a;
        const auto ao = -a;

        const auto abc = glm::cross(ab, ac);
        const auto acd = glm::cross(ac, ad);
        const auto adb = glm::cross(ad, ab);

        if (same_dir(abc, ao)) {
            return simplex_triangle(s = { a, b, c }, dir);
        }

        if (same_dir(acd, ao)) {
            return simplex_triangle(s = { a, c, d }, dir);
        }

        if (same_dir(adb, ao)) {
            return simplex_triangle(s = { a, d, b }, dir);
        }

        return true;
    }

    bool next_simplex(Simplex& s, glm::vec3& dir) {
        switch (s.size) {
        case 2: return simplex_line(s, dir);
        case 3: return simplex_triangle(s, dir);
        case 4: return simplex_tetrahedron(s, dir);
        default: return false;
        }
    }

} // namespace Physics