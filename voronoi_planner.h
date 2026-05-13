#pragma once

// ─────────────────────────────────────────────────────────────────────────────
// Segment Voronoi planner using Boost.Polygon voronoi_diagram.
//
// Public API:
//   geom::Polyline sdc::PlanPath(polygons, start, end)
//
// Internally uses Fortune's sweep line for segment sites (Boost.Polygon).
// Coordinates are scaled to int32 for exact arithmetic.
// Safe coordinate range: |world_coord| < ~167 units with SCALE=1e5.
// ─────────────────────────────────────────────────────────────────────────────

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>
#include <unordered_map>
#include <vector>

#include <boost/polygon/voronoi.hpp>

#include "geometry/geometry.h"
#include "dijkstra/dijkstra.h"

// ── Boost.Polygon integer types (internal) ────────────────────────────────────
namespace sdc_detail {

using coord_t = int32_t;
static constexpr double SCALE = 1e5;

struct IPoint {
    coord_t x, y;
    IPoint() = default;
    IPoint(coord_t x, coord_t y) : x(x), y(y) {}
};

struct ISegment {
    IPoint p0, p1;
    ISegment() = default;
    ISegment(IPoint a, IPoint b) : p0(a), p1(b) {}
};

} // namespace sdc_detail

// ── Boost.Polygon traits (must be in global namespace) ────────────────────────
namespace boost { namespace polygon {

template<>
struct geometry_concept<sdc_detail::IPoint> { typedef point_concept type; };

template<>
struct point_traits<sdc_detail::IPoint> {
    typedef sdc_detail::coord_t coordinate_type;
    static inline coordinate_type get(const sdc_detail::IPoint& p, orientation_2d o) {
        return (o == HORIZONTAL) ? p.x : p.y;
    }
};

template<>
struct geometry_concept<sdc_detail::ISegment> { typedef segment_concept type; };

template<>
struct segment_traits<sdc_detail::ISegment> {
    typedef sdc_detail::coord_t   coordinate_type;
    typedef sdc_detail::IPoint    point_type;
    static inline point_type get(const sdc_detail::ISegment& s, direction_1d d) {
        return (d == LOW) ? s.p0 : s.p1;
    }
};

}} // namespace boost::polygon

// ── Public API ────────────────────────────────────────────────────────────────
namespace sdc {

namespace detail {

using VD = boost::polygon::voronoi_diagram<double>;
using IPoint   = sdc_detail::IPoint;
using ISegment = sdc_detail::ISegment;
using coord_t  = sdc_detail::coord_t;
static constexpr double SCALE = sdc_detail::SCALE;

inline coord_t to_int(double v) {
    return static_cast<coord_t>(std::round(v * SCALE));
}

// Build undirected adjacency graph from finite VD edges within scene bounds.
// verts_out receives world-space (x,y) for each VD vertex in iteration order.
inline std::vector<std::unordered_map<size_t, double>> BuildGraph(
        const VD& vd,
        std::vector<std::pair<double,double>>& verts_out,
        double x0, double y0, double x1, double y1)
{
    const double margin = std::max(x1-x0, y1-y0) * 2.0;

    verts_out.clear();
    verts_out.reserve(vd.num_vertices());
    for (const auto& v : vd.vertices())
        verts_out.push_back({v.x() / SCALE, v.y() / SCALE});

    std::unordered_map<const VD::vertex_type*, size_t> vmap;
    vmap.reserve(vd.num_vertices());
    { size_t i = 0; for (const auto& v : vd.vertices()) vmap[&v] = i++; }

    std::vector<std::unordered_map<size_t, double>> graph(vd.num_vertices());

    for (const auto& edge : vd.edges()) {
        if (!edge.is_primary() || !edge.is_finite()) continue;
        const auto* va = edge.vertex0();
        const auto* vb = edge.vertex1();
        if (!va || !vb) continue;
        auto ia = vmap.find(va), ib = vmap.find(vb);
        if (ia == vmap.end() || ib == vmap.end()) continue;

        double ax = va->x()/SCALE, ay = va->y()/SCALE;
        double bx = vb->x()/SCALE, by = vb->y()/SCALE;
        auto oob = [&](double x, double y) {
            return x < x0-margin || x > x1+margin || y < y0-margin || y > y1+margin;
        };
        if (oob(ax,ay) || oob(bx,by)) continue;

        double len = std::hypot(bx-ax, by-ay);
        if (len < 1e-9) continue;
        graph[ia->second][ib->second] = len;
        graph[ib->second][ia->second] = len;
    }
    return graph;
}

// BFS: find the largest connected component; returns membership mask.
inline std::vector<bool> LargestComponent(
        const std::vector<std::unordered_map<size_t, double>>& graph)
{
    const size_t n = graph.size();
    std::vector<bool> visited(n, false), best(n, false);
    size_t best_sz = 0;
    for (size_t s = 0; s < n; s++) {
        if (visited[s]) continue;
        std::vector<size_t> comp;
        std::queue<size_t> q;
        q.push(s); visited[s] = true;
        while (!q.empty()) {
            size_t cur = q.front(); q.pop();
            comp.push_back(cur);
            for (const auto& [nb, _] : graph[cur])
                if (!visited[nb]) { visited[nb] = true; q.push(nb); }
        }
        if (comp.size() > best_sz) {
            best_sz = comp.size();
            std::fill(best.begin(), best.end(), false);
            for (size_t i : comp) best[i] = true;
        }
    }
    return best;
}

// Nearest vertex to (wx,wy) restricted to mask.
inline size_t NearestInMask(
        const std::vector<std::pair<double,double>>& verts,
        const std::vector<bool>& mask,
        double wx, double wy)
{
    size_t best = std::numeric_limits<size_t>::max();
    double best_d = std::numeric_limits<double>::max();
    for (size_t i = 0; i < verts.size(); i++) {
        if (!mask[i]) continue;
        double d = std::hypot(verts[i].first-wx, verts[i].second-wy);
        if (d < best_d) { best_d = d; best = i; }
    }
    return best;
}

} // namespace detail

// ─────────────────────────────────────────────────────────────────────────────
// PlanPath — compute a path from `start` to `end` that stays maximally far
// from the polygon obstacle boundaries (medial-axis routing).
// Returns a polyline: [start, voronoi_waypoints..., end].
// Falls back to a straight line if no Voronoi path is found.
// ─────────────────────────────────────────────────────────────────────────────
inline geom::Polyline PlanPath(
        const std::vector<geom::Polygon>& polygons,
        const geom::Point& start,
        const geom::Point& end)
{
    using namespace detail;

    // ── Scene bounds ──────────────────────────────────────────────────────────
    double x0 = std::min(start.x, end.x), y0 = std::min(start.y, end.y);
    double x1 = std::max(start.x, end.x), y1 = std::max(start.y, end.y);
    for (const auto& poly : polygons)
        for (const auto& pt : poly.vertices) {
            x0 = std::min(x0, pt.x); y0 = std::min(y0, pt.y);
            x1 = std::max(x1, pt.x); y1 = std::max(y1, pt.y);
        }

    // ── Convert polygon edges to integer segments ─────────────────────────────
    std::vector<ISegment> segs;
    for (const auto& poly : polygons) {
        const size_t n = poly.vertices.size();
        if (n < 2) continue;
        for (size_t i = 0; i < n; i++) {
            const auto& a = poly.vertices[i];
            const auto& b = poly.vertices[(i+1) % n];
            IPoint pa{to_int(a.x), to_int(a.y)};
            IPoint pb{to_int(b.x), to_int(b.y)};
            if (pa.x == pb.x && pa.y == pb.y) continue;
            segs.push_back({pa, pb});
        }
    }

    if (segs.empty()) return {start, end};

    // ── Build Voronoi diagram ─────────────────────────────────────────────────
    VD vd;
    boost::polygon::construct_voronoi(segs.begin(), segs.end(), &vd);
    if (vd.num_vertices() == 0) return {start, end};

    // ── Graph → largest component → Dijkstra ─────────────────────────────────
    std::vector<std::pair<double,double>> verts;
    auto graph = BuildGraph(vd, verts, x0, y0, x1, y1);
    auto mask  = LargestComponent(graph);

    const size_t src = NearestInMask(verts, mask, start.x, start.y);
    const size_t dst = NearestInMask(verts, mask, end.x,   end.y);
    if (src == std::numeric_limits<size_t>::max() ||
        dst == std::numeric_limits<size_t>::max()) return {start, end};

    auto indices = dijkstra::FindShortestPath(graph, src, dst);

    // ── Assemble polyline ─────────────────────────────────────────────────────
    geom::Polyline path;
    path.push_back(start);
    for (size_t i : indices)
        if (i < verts.size())
            path.push_back({verts[i].first, verts[i].second});
    path.push_back(end);
    return path;
}

} // namespace sdc
