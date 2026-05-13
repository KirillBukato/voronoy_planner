#pragma once

// ─────────────────────────────────────────────────────────────────────────────
// segment_diagram.h
//
// Builds a Voronoi diagram for polygon obstacle edges using Boost.Polygon,
// and returns an undirected adjacency graph over the finite Voronoi vertices.
//
// Responsibilities:
//   1. Compute scene bounds from polygons + start/end
//   2. Convert polygon edges to integer segments (×SCALE)
//   3. Run boost::polygon::construct_voronoi
//   4. Extract finite edges into an adjacency graph
//
// Public function:
//   seg_diag::BuildVoronoiGraph(polygons, start, end)
//     → seg_diag::VoronoiGraph { vertices, adjacency }
// ─────────────────────────────────────────────────────────────────────────────

#include <algorithm>
#include <cmath>
#include <limits>
#include <unordered_map>
#include <vector>

#include <boost/polygon/voronoi.hpp>

#include "geometry/geometry.h"

// ── Integer types (internal to Boost traits) ──────────────────────────────────
namespace seg_diag_detail {

    using coord_t = int32_t;

    struct IPoint {
        coord_t x, y;
        IPoint() = default;
        IPoint(coord_t x, coord_t y)
            : x(x)
            , y(y) {
        }
    };

    struct ISegment {
        IPoint p0, p1;
        ISegment() = default;
        ISegment(IPoint a, IPoint b)
            : p0(a)
            , p1(b) {
        }
    };

} // namespace seg_diag_detail

// ── Boost.Polygon traits (must be in global namespace) ────────────────────────
namespace boost {
    namespace polygon {

        template <>
        struct geometry_concept<seg_diag_detail::IPoint> {
            typedef point_concept type;
        };

        template <>
        struct point_traits<seg_diag_detail::IPoint> {
            typedef seg_diag_detail::coord_t coordinate_type;
            static inline coordinate_type get(const seg_diag_detail::IPoint& p, orientation_2d o) {
                return (o == HORIZONTAL) ? p.x : p.y;
            }
        };

        template <>
        struct geometry_concept<seg_diag_detail::ISegment> {
            typedef segment_concept type;
        };

        template <>
        struct segment_traits<seg_diag_detail::ISegment> {
            typedef seg_diag_detail::coord_t coordinate_type;
            typedef seg_diag_detail::IPoint point_type;
            static inline point_type get(const seg_diag_detail::ISegment& s, direction_1d d) {
                return (d == LOW) ? s.p0 : s.p1;
            }
        };

    } // namespace polygon
} // namespace boost

// ── Public API ────────────────────────────────────────────────────────────────
namespace seg_diag {

    // Scale factor: 1 world unit = SCALE integer units.
    // Safe world-coordinate range: |coord| < 2^24 / SCALE ≈ 167 units.
    static constexpr double SCALE = 1e5;

    // A Voronoi vertex in world coordinates.
    struct Vertex {
        double x, y;
    };

    // Adjacency graph over Voronoi vertices.
    // graph[i] maps neighbour index → edge length (world units).
    using Graph = std::vector<std::unordered_map<size_t, double>>;

    struct VoronoiGraph {
        std::vector<Vertex> vertices; // world-space positions
        Graph adjacency;              // undirected weighted graph
    };

    // ─────────────────────────────────────────────────────────────────────────────
    // BuildVoronoiGraph
    //
    // Converts polygon obstacle edges to a segment Voronoi diagram and returns
    // the finite-edge adjacency graph.  Edges far outside the scene bounding box
    // (2× scene extent margin) are discarded.
    // ─────────────────────────────────────────────────────────────────────────────
    inline VoronoiGraph BuildVoronoiGraph(
        const std::vector<geom::Polygon>& polygons,
        const geom::Point& start,
        const geom::Point& end) {
        using IPoint = seg_diag_detail::IPoint;
        using ISegment = seg_diag_detail::ISegment;
        using coord_t = seg_diag_detail::coord_t;
        using VD = boost::polygon::voronoi_diagram<double>;

        // ── 1. Scene bounds ───────────────────────────────────────────────────────
        double x0 = std::min(start.x, end.x), y0 = std::min(start.y, end.y);
        double x1 = std::max(start.x, end.x), y1 = std::max(start.y, end.y);
        for (const auto& poly : polygons) {
            for (const auto& pt : poly.vertices) {
                x0 = std::min(x0, pt.x);
                y0 = std::min(y0, pt.y);
                x1 = std::max(x1, pt.x);
                y1 = std::max(y1, pt.y);
            }
        }
        const double margin = std::max(x1 - x0, y1 - y0) * 2.0;

        // ── 2. Convert polygon edges to integer segments ──────────────────────────
        const auto to_int = [](double v) -> coord_t {
            return static_cast<coord_t>(std::round(v * SCALE));
        };

        std::vector<ISegment> segs;
        for (const auto& poly : polygons) {
            const size_t n = poly.vertices.size();
            if (n < 2) {
                continue;
            }
            for (size_t i = 0; i < n; i++) {
                const auto& a = poly.vertices[i];
                const auto& b = poly.vertices[(i + 1) % n];
                IPoint pa{to_int(a.x), to_int(a.y)};
                IPoint pb{to_int(b.x), to_int(b.y)};
                if (pa.x == pb.x && pa.y == pb.y) {
                    continue;
                }
                segs.push_back({pa, pb});
            }
        }

        if (segs.empty()) {
            return {};
        }

        // ── 3. Build Voronoi diagram ──────────────────────────────────────────────
        VD vd;
        boost::polygon::construct_voronoi(segs.begin(), segs.end(), &vd);

        if (vd.num_vertices() == 0) {
            return {};
        }

        // ── 4. Extract vertices and finite edges into adjacency graph ─────────────
        VoronoiGraph result;
        result.vertices.reserve(vd.num_vertices());
        for (const auto& v : vd.vertices()) {
            result.vertices.push_back({v.x() / SCALE, v.y() / SCALE});
        }

        // Map VD vertex pointer → index
        std::unordered_map<const VD::vertex_type*, size_t> vmap;
        vmap.reserve(vd.num_vertices());
        {
            size_t i = 0;
            for (const auto& v : vd.vertices()) {
                vmap[&v] = i++;
            }
        }

        result.adjacency.resize(vd.num_vertices());

        const auto out_of_bounds = [&](double x, double y) {
            return x < x0 - margin || x > x1 + margin ||
                   y < y0 - margin || y > y1 + margin;
        };

        for (const auto& edge : vd.edges()) {
            if (!edge.is_primary() || !edge.is_finite()) {
                continue;
            }
            const auto* va = edge.vertex0();
            const auto* vb = edge.vertex1();
            if (!va || !vb) {
                continue;
            }

            auto ia = vmap.find(va), ib = vmap.find(vb);
            if (ia == vmap.end() || ib == vmap.end()) {
                continue;
            }

            const double ax = va->x() / SCALE, ay = va->y() / SCALE;
            const double bx = vb->x() / SCALE, by = vb->y() / SCALE;
            if (out_of_bounds(ax, ay) || out_of_bounds(bx, by)) {
                continue;
            }

            const double len = std::hypot(bx - ax, by - ay);
            if (len < 1e-9) {
                continue;
            }

            result.adjacency[ia->second][ib->second] = len;
            result.adjacency[ib->second][ia->second] = len;
        }

        return result;
    }

} // namespace seg_diag
