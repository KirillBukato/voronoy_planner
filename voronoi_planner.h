#pragma once

#include <limits>
#include <queue>
#include <vector>

#include "dijkstra/dijkstra.h"
#include "geometry/geometry.h"
#include "voronoi_diagram/segment_diagram.h"

namespace sdc {

    inline geom::Polyline PlanPath(const std::vector<geom::Polygon>& polygons, const geom::Point& start, const geom::Point& finish) {
        const seg_diag::VoronoiGraph vg = seg_diag::BuildVoronoiGraph(polygons, start, finish);
        if (vg.vertices.empty()) {
            return {start, finish};
        }

        const size_t src = geom::NearestPoint(vg.vertices, start);
        const size_t dst = geom::NearestPoint(vg.vertices, finish);

        const auto indices = dijkstra::FindShortestPath(vg.adjacency, src, dst);

        geom::Polyline path;
        path.push_back(start);
        for (size_t i : indices) {
            if (i < vg.vertices.size()) {
                path.push_back({vg.vertices[i].x, vg.vertices[i].y});
            }
        }
        path.push_back(finish);
        return path;
    }

} // namespace sdc
