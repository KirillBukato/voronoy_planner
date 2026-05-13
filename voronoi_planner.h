#pragma once

#include <limits>
#include <queue>
#include <vector>

#include "geometry/geometry.h"
#include "voronoi_diagram/segment_diagram.h"
#include "dijkstra/dijkstra.h"

namespace sdc {

namespace detail {

// BFS: find the largest connected component; returns membership mask.
inline std::vector<bool> LargestComponent(const seg_diag::Graph& graph) {
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

// Nearest vertex to (wx, wy) restricted to mask.
inline size_t NearestInMask(
        const std::vector<seg_diag::Vertex>& verts,
        const std::vector<bool>& mask,
        double wx, double wy)
{
    size_t best = std::numeric_limits<size_t>::max();
    double best_d = std::numeric_limits<double>::max();
    for (size_t i = 0; i < verts.size(); i++) {
        if (!mask[i]) continue;
        double d = std::hypot(verts[i].x - wx, verts[i].y - wy);
        if (d < best_d) { best_d = d; best = i; }
    }
    return best;
}

} // namespace detail

// ─────────────────────────────────────────────────────────────────────────────
// PlanPath
// ─────────────────────────────────────────────────────────────────────────────
inline geom::Polyline PlanPath(
        const std::vector<geom::Polygon>& polygons,
        const geom::Point& start,
        const geom::Point& end)
{
    // Build Voronoi graph
    const seg_diag::VoronoiGraph vg = seg_diag::BuildVoronoiGraph(polygons, start, end);
    if (vg.vertices.empty()) return {start, end};

    // Find largest connected component (the medial axis between obstacles)
    const auto mask = detail::LargestComponent(vg.adjacency);

    // Find nearest Voronoi vertices to start and end
    const size_t src = detail::NearestInMask(vg.vertices, mask, start.x, start.y);
    const size_t dst = detail::NearestInMask(vg.vertices, mask, end.x,   end.y);
    if (src == std::numeric_limits<size_t>::max() ||
        dst == std::numeric_limits<size_t>::max()) return {start, end};

    // Dijkstra on the Voronoi graph
    const auto indices = dijkstra::FindShortestPath(vg.adjacency, src, dst);

    // Assemble polyline
    geom::Polyline path;
    path.push_back(start);
    for (size_t i : indices)
        if (i < vg.vertices.size())
            path.push_back({vg.vertices[i].x, vg.vertices[i].y});
    path.push_back(end);
    return path;
}

} // namespace sdc
