#include <iostream>
#include "geometry/geometry.h"
#include "voronoi_planner.h"
#include "visualization/visualization.h"

int main() {
    const geom::Point start{-3.0, 0.0};
    const geom::Point end{5.0, 0.0};

    const std::vector<geom::Polygon> polygons = {
        // Bottom boundary wall
        geom::Polygon{{{-3.0, -3.0}, {5.0, -3.0}, {5.0, -2.8}, {-3.0, -2.8}}},
        // Top boundary wall
        geom::Polygon{{{-3.0, 2.8}, {5.0, 2.8}, {5.0, 3.0}, {-3.0, 3.0}}},
        // Obstacle 1
        geom::Polygon{{{-1.7, -0.3}, {-0.9, -0.7}, {-0.6, 0.4}, {-1.4, 0.8}}},
        // Obstacle 2
        geom::Polygon{{{0.3, -1.8}, {1.4, -1.3}, {1.1, -0.3}, {0.1, -0.9}}},
        // Obstacle 3
        geom::Polygon{{{2.3, -0.8}, {3.7, -0.2}, {3.4, 0.9}, {2.1, 0.3}}},
    };

    const geom::Polyline path = sdc::PlanPath(polygons, start, end);
    viz::SaveSvg({polygons, start, end, path}, "output.svg");

    return 0;
}
