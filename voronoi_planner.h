#pragma once

// ─────────────────────────────────────────────────────────────────────────────
// Segment Voronoi planner using Boost.Polygon voronoi_diagram.
//
// Boost.Polygon voronoi works with INTEGER coordinates.  We scale all input
// coordinates by SCALE (1e5) so that 1 unit of world-space = 1e5 integer units.
//
// Input:  polygons (obstacle boundaries) + start/end points
// Output: path that stays maximally far from obstacles (medial axis routing)
// ─────────────────────────────────────────────────────────────────────────────

#include <algorithm>
#include <cmath>
#include <limits>
#include <unordered_map>
#include <vector>
#include <queue>

#include <boost/polygon/voronoi.hpp>

#include "geometry/geometry.h"
#include "dijkstra/dijkstra.h"

namespace sdc {

// ── Integer coordinate type used by Boost.Polygon ────────────────────────────
using coord_t  = int32_t;
using fcoord_t = double;

static constexpr fcoord_t SCALE = 1e5;   // world → integer

inline coord_t to_int(fcoord_t v) {
    return static_cast<coord_t>(std::round(v * SCALE));
}
inline fcoord_t to_world(coord_t v) {
    return static_cast<fcoord_t>(v) / SCALE;
}

// ── Boost.Polygon point / segment types ──────────────────────────────────────
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

} // namespace sdc

// ── Boost.Polygon traits specialisation (must be in global namespace) ─────────
namespace boost { namespace polygon {

template<>
struct geometry_concept<sdc::IPoint> { typedef point_concept type; };

template<>
struct point_traits<sdc::IPoint> {
    typedef sdc::coord_t coordinate_type;
    static inline coordinate_type get(const sdc::IPoint& p, orientation_2d orient) {
        return (orient == HORIZONTAL) ? p.x : p.y;
    }
};

template<>
struct geometry_concept<sdc::ISegment> { typedef segment_concept type; };

template<>
struct segment_traits<sdc::ISegment> {
    typedef sdc::coord_t    coordinate_type;
    typedef sdc::IPoint     point_type;
    static inline point_type get(const sdc::ISegment& s, direction_1d dir) {
        return (dir == LOW) ? s.p0 : s.p1;
    }
};

}} // namespace boost::polygon

namespace sdc {

// ── Voronoi diagram type ──────────────────────────────────────────────────────
using VD = boost::polygon::voronoi_diagram<fcoord_t>;

// ── Lightweight edge/vertex storage for visualisation ────────────────────────
struct VoronoiEdgeViz {
    double x1, y1, x2, y2;
    bool   finite;
};

// ── Result returned by the planner ───────────────────────────────────────────
struct PlannerResult {
    geom::Polyline              path;
    std::vector<VoronoiEdgeViz> voronoi_edges;
};

// ─────────────────────────────────────────────────────────────────────────────
// Build adjacency graph from the Voronoi diagram.
// Only finite edges (both endpoints finite and within scene bounds) are used.
// Returns graph indexed by VD vertex order.
// Also fills verts_out with world-space positions of each vertex.
// ─────────────────────────────────────────────────────────────────────────────
inline std::vector<std::unordered_map<size_t, double>> BuildGraph(
        const VD& vd,
        std::vector<std::pair<double,double>>& verts_out,
        double scene_x0, double scene_y0,
        double scene_x1, double scene_y1)
{
    const double margin = (std::max(scene_x1-scene_x0, scene_y1-scene_y0)) * 2.0;

    // Collect vertex positions
    verts_out.clear();
    verts_out.reserve(vd.num_vertices());
    for (const auto& v : vd.vertices()) {
        verts_out.push_back({v.x() / SCALE, v.y() / SCALE});
    }

    // Map VD vertex pointer → index
    std::unordered_map<const VD::vertex_type*, size_t> vmap;
    vmap.reserve(vd.num_vertices());
    {
        size_t idx = 0;
        for (const auto& v : vd.vertices()) vmap[&v] = idx++;
    }

    std::vector<std::unordered_map<size_t, double>> graph(vd.num_vertices());

    for (const auto& edge : vd.edges()) {
        if (!edge.is_primary()) continue;
        if (!edge.is_finite()) continue;

        const VD::vertex_type* va = edge.vertex0();
        const VD::vertex_type* vb = edge.vertex1();
        if (!va || !vb) continue;

        auto ia = vmap.find(va);
        auto ib = vmap.find(vb);
        if (ia == vmap.end() || ib == vmap.end()) continue;

        double ax = va->x() / SCALE, ay = va->y() / SCALE;
        double bx = vb->x() / SCALE, by = vb->y() / SCALE;

        // Skip vertices far outside the scene
        auto out_of_bounds = [&](double x, double y) {
            return x < scene_x0 - margin || x > scene_x1 + margin ||
                   y < scene_y0 - margin || y > scene_y1 + margin;
        };
        if (out_of_bounds(ax, ay) || out_of_bounds(bx, by)) continue;

        double dx = bx - ax, dy = by - ay;
        double len = std::sqrt(dx*dx + dy*dy);
        if (len < 1e-9) continue;

        graph[ia->second][ib->second] = len;
        graph[ib->second][ia->second] = len;
    }

    return graph;
}

// ─────────────────────────────────────────────────────────────────────────────
// Find the largest connected component in the graph.
// Returns a set of node indices belonging to it.
// ─────────────────────────────────────────────────────────────────────────────
inline std::vector<bool> LargestComponent(
        const std::vector<std::unordered_map<size_t, double>>& graph)
{
    const size_t n = graph.size();
    std::vector<bool> visited(n, false);
    std::vector<bool> best_mask(n, false);
    size_t best_size = 0;

    for (size_t start = 0; start < n; start++) {
        if (visited[start]) continue;
        // BFS
        std::vector<size_t> comp;
        std::queue<size_t> q;
        q.push(start);
        visited[start] = true;
        while (!q.empty()) {
            size_t cur = q.front(); q.pop();
            comp.push_back(cur);
            for (const auto& [nb, _] : graph[cur]) {
                if (!visited[nb]) { visited[nb] = true; q.push(nb); }
            }
        }
        if (comp.size() > best_size) {
            best_size = comp.size();
            std::fill(best_mask.begin(), best_mask.end(), false);
            for (size_t idx : comp) best_mask[idx] = true;
        }
    }
    return best_mask;
}

// ─────────────────────────────────────────────────────────────────────────────
// Find the graph node nearest to (wx, wy), restricted to nodes in `mask`.
// ─────────────────────────────────────────────────────────────────────────────
inline size_t NearestInMask(
        const std::vector<std::pair<double,double>>& verts,
        const std::vector<bool>& mask,
        double wx, double wy)
{
    size_t best = std::numeric_limits<size_t>::max();
    double best_d = std::numeric_limits<double>::max();
    for (size_t i = 0; i < verts.size(); i++) {
        if (!mask[i]) continue;
        double dx = verts[i].first  - wx;
        double dy = verts[i].second - wy;
        double d  = dx*dx + dy*dy;
        if (d < best_d) { best_d = d; best = i; }
    }
    return best;
}

// ─────────────────────────────────────────────────────────────────────────────
// Collect finite Voronoi edges for visualisation.
// ─────────────────────────────────────────────────────────────────────────────
inline std::vector<VoronoiEdgeViz> CollectEdges(
        const VD& vd,
        double scene_x0, double scene_y0,
        double scene_x1, double scene_y1)
{
    const double margin = (std::max(scene_x1-scene_x0, scene_y1-scene_y0)) * 2.0;
    std::vector<VoronoiEdgeViz> result;
    result.reserve(vd.num_edges() / 2);
    for (const auto& edge : vd.edges()) {
        if (!edge.is_primary()) continue;
        if (!edge.is_finite()) continue;
        // Skip the twin to avoid drawing each edge twice
        if (&edge > edge.twin()) continue;
        const VD::vertex_type* va = edge.vertex0();
        const VD::vertex_type* vb = edge.vertex1();
        if (!va || !vb) continue;
        double ax = va->x()/SCALE, ay = va->y()/SCALE;
        double bx = vb->x()/SCALE, by = vb->y()/SCALE;
        auto out = [&](double x, double y) {
            return x < scene_x0-margin || x > scene_x1+margin ||
                   y < scene_y0-margin || y > scene_y1+margin;
        };
        if (out(ax,ay) || out(bx,by)) continue;
        result.push_back({ax, ay, bx, by, true});
    }
    return result;
}

// ─────────────────────────────────────────────────────────────────────────────
// Main planner entry point.
// ─────────────────────────────────────────────────────────────────────────────
inline PlannerResult ConstructPolylineUsingVoronoi(
        const std::vector<geom::Polygon>& polygons,
        const geom::Point& localization,
        const geom::Point& destination)
{
    PlannerResult result;

    // ── 1. Compute scene bounds ───────────────────────────────────────────────
    double sx0 = localization.x, sy0 = localization.y;
    double sx1 = destination.x,  sy1 = destination.y;
    if (sx0 > sx1) std::swap(sx0, sx1);
    if (sy0 > sy1) std::swap(sy0, sy1);
    for (const auto& poly : polygons) {
        for (const auto& pt : poly.vertices) {
            sx0 = std::min(sx0, pt.x); sy0 = std::min(sy0, pt.y);
            sx1 = std::max(sx1, pt.x); sy1 = std::max(sy1, pt.y);
        }
    }

    // ── 2. Convert polygon edges to integer segments ──────────────────────────
    std::vector<ISegment> segments;
    for (const auto& poly : polygons) {
        const size_t n = poly.vertices.size();
        if (n < 2) continue;
        for (size_t i = 0; i < n; i++) {
            const auto& a = poly.vertices[i];
            const auto& b = poly.vertices[(i + 1) % n];
            IPoint pa{to_int(a.x), to_int(a.y)};
            IPoint pb{to_int(b.x), to_int(b.y)};
            if (pa.x == pb.x && pa.y == pb.y) continue;
            segments.push_back({pa, pb});
        }
    }

    if (segments.empty()) {
        result.path = {localization, destination};
        return result;
    }

    // ── 3. Build Voronoi diagram ──────────────────────────────────────────────
    VD vd;
    boost::polygon::construct_voronoi(segments.begin(), segments.end(), &vd);

    if (vd.num_vertices() == 0) {
        result.path = {localization, destination};
        return result;
    }

    // ── 4. Collect visualisation edges ───────────────────────────────────────
    result.voronoi_edges = CollectEdges(vd, sx0, sy0, sx1, sy1);

    // ── 5. Build graph ────────────────────────────────────────────────────────
    std::vector<std::pair<double,double>> verts;
    auto graph = BuildGraph(vd, verts, sx0, sy0, sx1, sy1);

    // ── 6. Find largest connected component ──────────────────────────────────
    auto main_comp = LargestComponent(graph);

    // ── 7. Find nearest nodes in main component ───────────────────────────────
    const size_t src = NearestInMask(verts, main_comp, localization.x, localization.y);
    const size_t dst = NearestInMask(verts, main_comp, destination.x,  destination.y);

    if (src == std::numeric_limits<size_t>::max() ||
        dst == std::numeric_limits<size_t>::max()) {
        result.path = {localization, destination};
        return result;
    }

    // ── 8. Dijkstra ───────────────────────────────────────────────────────────
    std::vector<size_t> path_indices =
        dijkstra::FindShortestPath(graph, src, dst);

    // ── 9. Build output polyline ──────────────────────────────────────────────
    result.path.push_back(localization);
    for (size_t pi : path_indices) {
        if (pi < verts.size()) {
            result.path.push_back({verts[pi].first, verts[pi].second});
        }
    }
    result.path.push_back(destination);

    return result;
}

} // namespace sdc
