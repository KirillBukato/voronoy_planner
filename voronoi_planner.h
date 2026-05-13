#pragma once

#include <algorithm>
#include <deque>
#include <unordered_map>
#include "geometry/geometry.h"
#include "voronoi_diagram/sweepline.hpp"
#include "dijkstra/dijkstra.h"

namespace sdc {

    using VoronoiDiagram = sweepline<std::vector<geom::Point>::const_iterator, geom::Point, double>;
    using VoronoiVertex = VoronoiDiagram::vertex;
    using VoronoiEdge   = VoronoiDiagram::edge;
    constexpr double eps = 1e-5;

    inline std::vector<std::unordered_map<size_t, double>> PrepareVoronoiForDijkstra(
            const VoronoiDiagram& diagram) {
        std::vector<std::unordered_map<size_t, double>> graph(diagram.vertices_.size());
        for (const auto& edge : diagram.edges_) {
            if (edge.b == -1 || edge.e == -1) {
                continue;
            }
            const auto distance = geom::Distance(diagram.vertices_[edge.b].c,
                                                  diagram.vertices_[edge.e].c);
            graph[edge.e][edge.b] = distance;
            graph[edge.b][edge.e] = distance;
        }
        return graph;
    }

    // Copyable snapshot of the Voronoi diagram data (sweepline itself is non-copyable/movable)
    struct VoronoiData {
        std::deque<VoronoiVertex> vertices;
        std::deque<VoronoiEdge>   edges;
        VoronoiDiagram::pvertex   inf;
    };

    struct PlannerResult {
        geom::Polyline path;
        VoronoiData    voronoi;
        std::vector<geom::Point> sites;
    };

    inline PlannerResult ConstructPolylineUsingVoronoi(const std::vector<geom::Point>& points,
                                                const geom::Point& localization,
                                                const geom::Point& destination) {
        std::vector<geom::Point> sorted_points = points;
        std::sort(sorted_points.begin(), sorted_points.end());

        VoronoiDiagram diagram{eps};
        diagram(std::cbegin(sorted_points), std::cend(sorted_points));

        const auto src_idx = std::distance(
                diagram.vertices_.begin(),
                std::min_element(diagram.vertices_.begin(), diagram.vertices_.end(),
                                 [&localization](const auto& lhs, const auto& rhs) {
                                     geom::Point a{lhs.c.x, lhs.c.y};
                                     geom::Point b{rhs.c.x, rhs.c.y};
                                     return geom::Distance(localization, a) < geom::Distance(localization, b);
                                 }));
        const auto dst_idx = std::distance(
                diagram.vertices_.begin(),
                std::min_element(diagram.vertices_.begin(), diagram.vertices_.end(),
                                 [&destination](const auto& lhs, const auto& rhs) {
                                     geom::Point a{lhs.c.x, lhs.c.y};
                                     geom::Point b{rhs.c.x, rhs.c.y};
                                     return geom::Distance(destination, a) < geom::Distance(destination, b);
                                 }));

        std::vector<std::unordered_map<size_t, double>> graph = PrepareVoronoiForDijkstra(diagram);
        std::vector<size_t> path_indices = dijkstra::FindShortestPath(graph, src_idx, dst_idx);

        geom::Polyline path;
        path.push_back(localization);
        for (size_t index : path_indices) {
            path.push_back(diagram.vertices_[index].c);
        }
        path.push_back(destination);

        VoronoiData voronoi{diagram.vertices_, diagram.edges_, diagram.inf};

        return PlannerResult{std::move(path), std::move(voronoi), sorted_points};
    }

} // namespace sdc
