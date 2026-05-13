#pragma once

#include <unordered_map>
#include "geometry/geometry.h"
#include "voronoi_diagram/sweepline.hpp"
#include "dijkstra/dijkstra.h"

namespace sdc {

    using VoronoiDiagram = sweepline<std::vector<geom::Point>::const_iterator, geom::Point, double>;
    constexpr double eps = 1e-6;

    std::vector<std::unordered_map<size_t, double>> PrepareVoronoiForDijkstra(const VoronoiDiagram& diagram) {
        std::vector<std::unordered_map<size_t, double>> graph(diagram.vertices_.size());
        for (const auto& edge : diagram.edges_) {
            if (edge.b == -1 || edge.e == -1) {
                continue;
            }
            geom::Point a = diagram.vertices_[edge.b].c;
            geom::Point b = diagram.vertices_[edge.e].c;
            const auto distance = geom::Distance(a, b);
            graph[edge.e][edge.b] = distance;
            graph[edge.b][edge.e] = distance;
        }
        return graph;
    }

    geom::Polyline ConstructPolylineUsingVoronoi(std::vector<geom::Point> obstacles,
                                                 const geom::Point& localization,
                                                 const geom::Point& destination) {
        VoronoiDiagram diagram{eps};
        std::sort(std::begin(obstacles), std::end(obstacles));
        diagram(std::cbegin(obstacles), std::cend(obstacles));

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
        std::vector<size_t> path = dijkstra::FindShortestPath(graph, src_idx, dst_idx);

        geom::Polyline result;
        result.push_back(localization);
        for (size_t index : path) {
            result.push_back(diagram.vertices_[index].c);
        }
        result.push_back(destination);

        return result;
    }
}
