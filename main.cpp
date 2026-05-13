#include <iostream>
#include "geometry/geometry.h"
#include "voronoi_planner.h"
#include "visualization/visualization.h"

int main() {
    // Scene bounds: corridor from x=-3 to x=5, y=-3 to y=3
    // Bottom wall (thin horizontal rectangle)
    // Top wall (thin horizontal rectangle)
    // Three convex polygon obstacles
    std::vector<geom::Polygon> polygons = {
        // Bottom boundary wall
        geom::Polygon{{
            {-3.0, -3.0},
            { 5.0, -3.0},
            { 5.0, -2.8},
            {-3.0, -2.8}
        }},
        // Top boundary wall
        geom::Polygon{{
            {-3.0,  2.8},
            { 5.0,  2.8},
            { 5.0,  3.0},
            {-3.0,  3.0}
        }},
        // Obstacle 1: roughly left-centre
        geom::Polygon{{
            {-1.7, -0.6},
            {-0.6, -0.6},
            {-0.6,  0.6},
            {-1.7,  0.6}
        }},
        // Obstacle 2: bottom-centre
        geom::Polygon{{
            { 0.3, -1.8},
            { 1.4, -1.3},
            { 1.1, -0.3},
            { 0.1, -0.9}
        }},
        // Obstacle 3: right-centre
        geom::Polygon{{
            { 2.3, -0.8},
            { 3.7, -0.2},
            { 3.4,  0.9},
            { 2.1,  0.3}
        }},
    };

    geom::Point localization{-3.0, 0.0};
    geom::Point destination{5.0, 0.0};

    sdc::PlannerResult result = sdc::ConstructPolylineUsingVoronoi(
        polygons, localization, destination);

    std::cout << "Voronoi edges: " << result.voronoi_edges.size() << "\n";
    std::cout << "Path (" << result.path.size() << " points):\n";
    for (const auto& pt : result.path) {
        std::cout << "  " << pt.x << " " << pt.y << "\n";
    }

    viz::VisualizationData visData{
        polygons,
        result.voronoi_edges,
        localization,
        destination,
        result.path
    };
    viz::SaveSvg(visData, "output.svg");
    std::cout << "Visualization saved to output.svg\n";

    return 0;
}
