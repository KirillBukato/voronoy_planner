#pragma once

#include <algorithm>
#include <cmath>
#include <limits>
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

    // Squared distance from point p to segment [a, b]
    inline double DistanceSqPointSegment(const Point& p, const Point& a, const Point& b) {
        const double dx = b.x - a.x;
        const double dy = b.y - a.y;
        const double lenSq = dx * dx + dy * dy;
        if (lenSq < 1e-18) return DistanceSq(p, a);
        const double t = std::max(0.0, std::min(1.0,
            ((p.x - a.x) * dx + (p.y - a.y) * dy) / lenSq));
        const Point proj{a.x + t * dx, a.y + t * dy};
        return DistanceSq(p, proj);
    }

    // Minimum distance from point p to any edge of the polygon
    inline double DistancePointPolygon(const Point& p, const Polygon& poly) {
        const auto& v = poly.vertices;
        const std::size_t n = v.size();
        if (n == 0) return std::numeric_limits<double>::max();
        double minSq = std::numeric_limits<double>::max();
        for (std::size_t i = 0, j = n - 1; i < n; j = i++) {
            minSq = std::min(minSq, DistanceSqPointSegment(p, v[j], v[i]));
        }
        return std::sqrt(minSq);
    }

    // Minimum clearance of point p from all polygons
    inline double MinClearance(const Point& p, const std::vector<Polygon>& polygons) {
        double minDist = std::numeric_limits<double>::max();
        for (const auto& poly : polygons) {
            minDist = std::min(minDist, DistancePointPolygon(p, poly));
        }
        return minDist;
    }

    // Returns true if point p is strictly inside the polygon (ray casting algorithm)
    inline bool ContainsPoint(const Polygon& poly, const Point& p) {
        const auto& v = poly.vertices;
        const std::size_t n = v.size();
        if (n == 0) return false;
        bool inside = false;
        for (std::size_t i = 0, j = n - 1; i < n; j = i++) {
            const Point& vi = v[i];
            const Point& vj = v[j];
            if (((vi.y > p.y) != (vj.y > p.y)) &&
                (p.x < (vj.x - vi.x) * (p.y - vi.y) / (vj.y - vi.y) + vi.x)) {
                inside = !inside;
            }
        }
        return inside;
    }

    // Returns true if segments [p1,p2] and [p3,p4] properly intersect
    inline bool SegmentsIntersect(const Point& p1, const Point& p2,
                                   const Point& p3, const Point& p4) {
        const auto cross = [](const Point& o, const Point& a, const Point& b) {
            return (a.x - o.x) * (b.y - o.y) - (a.y - o.y) * (b.x - o.x);
        };
        const double d1 = cross(p3, p4, p1);
        const double d2 = cross(p3, p4, p2);
        const double d3 = cross(p1, p2, p3);
        const double d4 = cross(p1, p2, p4);
        if (((d1 > 0 && d2 < 0) || (d1 < 0 && d2 > 0)) &&
            ((d3 > 0 && d4 < 0) || (d3 < 0 && d4 > 0))) {
            return true;
        }
        return false;
    }

    // Returns true if segment [a,b] crosses any polygon edge OR its midpoint is inside the polygon
    inline bool SegmentIntersectsPolygon(const Point& a, const Point& b,
                                          const Polygon& poly) {
        const auto& v = poly.vertices;
        const std::size_t n = v.size();
        if (n == 0) return false;
        for (std::size_t i = 0, j = n - 1; i < n; j = i++) {
            if (SegmentsIntersect(a, b, v[j], v[i])) return true;
        }
        const Point mid{(a.x + b.x) * 0.5, (a.y + b.y) * 0.5};
        return ContainsPoint(poly, mid);
    }

    // Returns true if segment [a,b] passes through any of the given polygons
    inline bool EdgePassesThroughAnyPolygon(const Point& a, const Point& b,
                                             const std::vector<Polygon>& polygons) {
        for (const auto& poly : polygons) {
            if (SegmentIntersectsPolygon(a, b, poly)) return true;
        }
        return false;
    }

} // namespace geom
