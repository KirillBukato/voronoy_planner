#pragma once

#include <limits>
#include <queue>
#include <vector>

#include "dijkstra/dijkstra.h"
#include "geometry/geometry.h"
#include "voronoi_diagram/segment_diagram.h"

namespace sdc {

    inline geom::Polyline PlanPath(const std::vector<geom::Polygon>& polygons, const geom::Point& start, const geom::Point& finish) {
        const seg_diag::VoronoiGraph voronoi_graph = seg_diag::BuildVoronoiGraph(polygons, start, finish);
        if (voronoi_graph.vertices.empty()) {
            return {start, finish};
        }

        const size_t src = geom::NearestPoint(start, voronoi_graph.vertices);
        const size_t dst = geom::NearestPoint(finish, voronoi_graph.vertices);

        const auto indices = dijkstra::FindShortestPath(voronoi_graph.adjacency, src, dst);

        geom::Polyline path;
        path.push_back(start);
        for (size_t i : indices) {
            if (i < voronoi_graph.vertices.size()) {
                path.push_back(voronoi_graph.vertices[i]);
            }
        }
        path.push_back(finish);
        return path;
    }

} // namespace sdc
