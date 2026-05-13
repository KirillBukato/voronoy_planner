#pragma once

#include <algorithm>
#include <cmath>
#include <deque>
#include <random>
#include <unordered_map>
#include "geometry/geometry.h"
#include "voronoi_diagram/sweepline.hpp"
#include "dijkstra/dijkstra.h"

namespace sdc {

    using VoronoiDiagram = sweepline<std::vector<geom::Point>::const_iterator, geom::Point, double>;
    using VoronoiVertex = VoronoiDiagram::vertex;
    using VoronoiEdge   = VoronoiDiagram::edge;
    constexpr double eps = 1e-5;

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

    // Sample points along all polygon edges at the given step, plus vertices.
    // Also adds start, end, and a frame of points around the scene bbox so the
    // Voronoi diagram has edges in the outer region.
    inline std::vector<geom::Point> CollectSites(
            const std::vector<geom::Polygon>& polygons,
            const geom::Point& start,
            const geom::Point& end,
            double step) {

        std::vector<geom::Point> pts;

        // 1. Sample polygon edges
        for (const auto& poly : polygons) {
            const auto& v = poly.vertices;
            const std::size_t n = v.size();
            for (std::size_t i = 0; i < n; ++i) {
                const geom::Point& a = v[i];
                const geom::Point& b = v[(i + 1) % n];
                pts.push_back(a);
                const double len = geom::Distance(a, b);
                const int steps = std::max(1, static_cast<int>(std::ceil(len / step)));
                for (int k = 1; k < steps; ++k) {
                    const double t = k / static_cast<double>(steps);
                    pts.push_back({a.x + t * (b.x - a.x), a.y + t * (b.y - a.y)});
                }
            }
        }

        // 2. Add start and end as sites so the diagram has vertices near them
        pts.push_back(start);
        pts.push_back(end);

        // 3. Compute scene bbox
        double minX = start.x, maxX = start.x;
        double minY = start.y, maxY = start.y;
        const auto expand = [&](double x, double y) {
            minX = std::min(minX, x); maxX = std::max(maxX, x);
            minY = std::min(minY, y); maxY = std::max(maxY, y);
        };
        expand(end.x, end.y);
        for (const auto& p : pts) { expand(p.x, p.y); }

        // 4. Add corner + midpoint sentinel points just outside the bbox.
        //    These ensure the Voronoi diagram has edges in the outer region
        //    without creating long collinear sequences that cause degeneracies.
        const double pad = std::max(maxX - minX, maxY - minY) * 0.4 + step * 3.0;
        const double x0 = minX - pad, x1 = maxX + pad;
        const double y0 = minY - pad, y1 = maxY + pad;
        const double xm = (x0 + x1) * 0.5 + step * 0.37;  // offset midpoints to avoid symmetry
        const double ym = (y0 + y1) * 0.5 + step * 0.23;
        // 4 corners
        pts.push_back({x0, y0});
        pts.push_back({x1, y0});
        pts.push_back({x0, y1});
        pts.push_back({x1, y1});
        // midpoints of each side (offset to break symmetry)
        pts.push_back({xm, y0});
        pts.push_back({xm, y1});
        pts.push_back({x0, ym});
        pts.push_back({x1, ym});
        // extra points along top/bottom to cover the long road
        {
            const double roadStep = (x1 - x0) / 5.0 + step * 0.17;
            for (double x = x0 + roadStep; x < x1 - 1e-9; x += roadStep) {
                pts.push_back({x, y0 + step * 0.11});
                pts.push_back({x, y1 - step * 0.13});
            }
        }

        // 5. Deduplicate
        std::sort(pts.begin(), pts.end());
        pts.erase(std::unique(pts.begin(), pts.end(),
                              [](const geom::Point& a, const geom::Point& b) {
                                  return geom::Equals(a, b);
                              }),
                  pts.end());

        // 6. Deterministic jitter to break cocircular/collinear degeneracies.
        //    Use 1e-4 — large enough to perturb but negligible vs step size (0.4m).
        std::mt19937_64 rng{12345};
        std::uniform_real_distribution<double> jitter(-1e-4, 1e-4);
        for (auto& p : pts) {
            p.x += jitter(rng);
            p.y += jitter(rng);
        }
        std::sort(pts.begin(), pts.end());

        return pts;
    }

    // Clearance penalty factor: edges near obstacles cost more.
    // weight = dist * (1 + alpha / max(clearance, eps))
    // alpha controls how much clearance matters vs raw distance.
    constexpr double kClearanceAlpha = 1.0;

    inline std::vector<std::unordered_map<size_t, double>> PrepareVoronoiForDijkstra(
            const VoronoiDiagram& diagram,
            const std::vector<geom::Polygon>& polygons,
            double bbMinX, double bbMinY, double bbMaxX, double bbMaxY) {
        std::vector<std::unordered_map<size_t, double>> graph(diagram.vertices_.size());
        const auto inBB = [&](const geom::Point& p) {
            return p.x >= bbMinX && p.x <= bbMaxX && p.y >= bbMinY && p.y <= bbMaxY;
        };
        for (const auto& edge : diagram.edges_) {
            if (edge.b == -1 || edge.e == -1) continue;
            const geom::Point a = diagram.vertices_[edge.b].c;
            const geom::Point b = diagram.vertices_[edge.e].c;
            if (!inBB(a) || !inBB(b)) continue;
            if (geom::EdgePassesThroughAnyPolygon(a, b, polygons)) continue;
            const double dist = geom::Distance(a, b);
            // Clearance-weighted cost: dist * (1 + alpha/clearance)
            // Prefers paths through open space without going wildly out of the way.
            const double clearA = geom::MinClearance(a, polygons);
            const double clearB = geom::MinClearance(b, polygons);
            const double clearance = std::max(std::min(clearA, clearB), 1e-3);
            const double weight = dist * (1.0 + kClearanceAlpha / clearance);
            graph[edge.e][edge.b] = weight;
            graph[edge.b][edge.e] = weight;
        }
        return graph;
    }

    inline PlannerResult ConstructPolylineUsingVoronoi(
            const std::vector<geom::Polygon>& obstacles,
            const geom::Point& localization,
            const geom::Point& destination,
            double sampling_step = 0.5) {

        // Collect sites: polygon edges + start + end + frame
        std::vector<geom::Point> sites = CollectSites(obstacles, localization, destination, sampling_step);

        VoronoiDiagram diagram{eps};
        diagram(std::cbegin(sites), std::cend(sites));

        // Scene bbox (from sites, with padding for the graph filter)
        double bbMinX = localization.x, bbMaxX = localization.x;
        double bbMinY = localization.y, bbMaxY = localization.y;
        const auto expandBB = [&](double x, double y) {
            bbMinX = std::min(bbMinX, x); bbMaxX = std::max(bbMaxX, x);
            bbMinY = std::min(bbMinY, y); bbMaxY = std::max(bbMaxY, y);
        };
        expandBB(destination.x, destination.y);
        for (const auto& p : sites) { expandBB(p.x, p.y); }
        // Tight padding: just enough to include all frame points
        const double bbPad = std::max(bbMaxX - bbMinX, bbMaxY - bbMinY) * 0.3 + 1.0;
        bbMinX -= bbPad; bbMinY -= bbPad; bbMaxX += bbPad; bbMaxY += bbPad;

        // Build graph with blocked edges removed
        std::vector<std::unordered_map<size_t, double>> graph =
            PrepareVoronoiForDijkstra(diagram, obstacles, bbMinX, bbMinY, bbMaxX, bbMaxY);

        // Add virtual nodes for start and end — connect to all reachable in-bounds Voronoi vertices
        const size_t N = diagram.vertices_.size();
        const size_t virtual_src = N;
        const size_t virtual_dst = N + 1;
        graph.resize(N + 2);

        const auto inBB = [&](const geom::Point& p) {
            return p.x >= bbMinX && p.x <= bbMaxX && p.y >= bbMinY && p.y <= bbMaxY;
        };

        const auto clearanceWeight = [&](const geom::Point& a, const geom::Point& b) {
            const double dist = geom::Distance(a, b);
            const double clearA = geom::MinClearance(a, obstacles);
            const double clearB = geom::MinClearance(b, obstacles);
            const double clearance = std::max(std::min(clearA, clearB), 1e-3);
            return dist * (1.0 + kClearanceAlpha / clearance);
        };

        for (size_t i = 0; i < N; i++) {
            const geom::Point vp = diagram.vertices_[i].c;
            if (!inBB(vp)) continue;
            if (!geom::EdgePassesThroughAnyPolygon(localization, vp, obstacles)) {
                const double w = clearanceWeight(localization, vp);
                graph[virtual_src][i] = w;
                graph[i][virtual_src] = w;
            }
            if (!geom::EdgePassesThroughAnyPolygon(destination, vp, obstacles)) {
                const double w = clearanceWeight(destination, vp);
                graph[virtual_dst][i] = w;
                graph[i][virtual_dst] = w;
            }
        }
        // Direct start→end if unobstructed
        if (!geom::EdgePassesThroughAnyPolygon(localization, destination, obstacles)) {
            const double w = clearanceWeight(localization, destination);
            graph[virtual_src][virtual_dst] = w;
            graph[virtual_dst][virtual_src] = w;
        }

        std::vector<size_t> path_indices = dijkstra::FindShortestPath(graph, virtual_src, virtual_dst);

        geom::Polyline path;
        path.push_back(localization);
        for (size_t idx : path_indices) {
            if (idx < N) {
                path.push_back(diagram.vertices_[idx].c);
            }
        }
        path.push_back(destination);

        VoronoiData voronoi{diagram.vertices_, diagram.edges_, diagram.inf};

        return PlannerResult{std::move(path), std::move(voronoi), std::move(sites)};
    }

} // namespace sdc
