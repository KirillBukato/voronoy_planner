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
    };

    double DistanceSq(const Point& a, const Point& b) {
        return (a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y);
    }

    double Distance(const Point& a, const Point& b) {
        return std::sqrt(DistanceSq(a, b));
    }

    bool Equals(const Point& a, const Point& b) {
        return std::abs(a.x - b.x) <= 1e-9 && std::abs(a.y - b.y) <= 1e-9;
    }

    using Polyline = std::vector<geom::Point>;

} // namespace geom

