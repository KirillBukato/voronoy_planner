#pragma once

#include <limits>
#include <queue>
#include <vector>

#include "dijkstra/dijkstra.h"
#include "geometry/geometry.h"
#include "voronoi_diagram/segment_diagram.h"

namespace sdc {

    namespace detail {

        inline size_t Nearest(const std::vector<geom::Point>& verts, double wx, double wy) {
            size_t best = std::numeric_limits<size_t>::max();
            double best_d = std::numeric_limits<double>::max();
            for (size_t i = 0; i < verts.size(); i++) {
                double d = std::hypot(verts[i].x - wx, verts[i].y - wy);
                if (d < best_d) {
                    best_d = d;
                    best = i;
                }
            }
            return best;
        }

    } // namespace detail

    inline geom::Polyline PlanPath(
        const std::vector<geom::Polygon>& polygons, const geom::Point& start,
        const geom::Point& end) {

        const seg_diag::VoronoiGraph vg = seg_diag::BuildVoronoiGraph(polygons, start, end);
        if (vg.vertices.empty()) {
            return {start, end};
        }

        // Seed: nearest vertex to start (guaranteed on exterior medial axis)
        const size_t src = detail::Nearest(vg.vertices, start.x, start.y);
        const size_t dst = detail::Nearest(vg.vertices, end.x, end.y);

        const auto indices = dijkstra::FindShortestPath(vg.adjacency, src, dst);

        geom::Polyline path;
        path.push_back(start);
        for (size_t i : indices) {
            if (i < vg.vertices.size()) {
                path.push_back({vg.vertices[i].x, vg.vertices[i].y});
            }
        }
        path.push_back(end);
        return path;
    }

} // namespace sdc
