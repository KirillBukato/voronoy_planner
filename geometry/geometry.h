#pragma once

#include <cmath>
#include <vector>

namespace geom {
    struct Point {
        double x;
        double y;

        bool operator<(const Point& p) const {
            return std::tie(x, y) < std::tie(p.x, p.y);
        }

        bool operator==(const Point& p) const {
            return x == p.x && y == p.y;
        }
    };

    inline double DistanceSq(const Point& a, const Point& b) {
        return (a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y);
    }

    inline double Distance(const Point& a, const Point& b) {
        return std::sqrt(DistanceSq(a, b));
    }

    inline bool Equals(const Point& a, const Point& b) {
        return std::abs(a.x - b.x) <= 1e-9 && std::abs(a.y - b.y) <= 1e-9;
    }

    using Polyline = std::vector<geom::Point>;

    struct Polygon {
        std::vector<Point> vertices;

        Polygon() = default;

        explicit Polygon(std::vector<Point> vertices)
            : vertices{std::move(vertices)}
        { }

        std::size_t size() const {
            return vertices.size();
        }

        bool empty() const {
            return vertices.empty();
        }
    };

} // namespace geom
