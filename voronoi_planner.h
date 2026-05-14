#pragma once

#include <limits>
#include <queue>
#include <vector>

#include "dijkstra/dijkstra.h"
#include "geometry/geometry.h"
#include "voronoi_diagram/segment_diagram.h"

namespace sdc {

    inline geom::Polyline PlanPath(const std::vector<geom::Polygon>& polygons, const geom::Point& start, const geom::Point& finish) {
        const seg_diag::VoronoiGraph voronoi_diagram = seg_diag::BuildVoronoiGraph(polygons, start, finish);
        if (voronoi_diagram.vertices.empty()) {
            return {start, finish};
        }

        const size_t src = geom::NearestPoint(start, voronoi_diagram.vertices);
        const size_t dst = geom::NearestPoint(finish, voronoi_diagram.vertices);

        const auto indices = dijkstra::FindShortestPath(voronoi_diagram.adjacency, src, dst);

        geom::Polyline path;
        path.push_back(start);
        for (size_t i : indices) {
            if (i < voronoi_diagram.vertices.size()) {
                path.push_back({voronoi_diagram.vertices[i].x, voronoi_diagram.vertices[i].y});
            }
        }
        path.push_back(finish);
        return path;
    }

} // namespace sdc
