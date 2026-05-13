#include <iostream>
#include "geometry/geometry.h"
#include "voronoi_planner.h"
#include "visualization/visualization.h"

int main() {
    // Obstacle boundary points (sampled from polygon edges)
    std::vector<geom::Point> points = {
        {-1.7, -0.3}, {-0.9, -0.7}, {-0.6,  0.4}, {-1.4,  0.8},
        { 0.3, -1.8}, { 1.4, -1.3}, { 1.1, -0.3}, { 0.1, -0.9},
        { 0.2,  0.7}, { 1.3,  0.3}, { 1.6,  1.7}, { 0.5,  1.9},
        { 2.3, -0.8}, { 3.7, -0.2}, { 3.4,  0.9}, { 2.1,  0.3},
    };

    geom::Point localization{-3.0, 0.0};
    geom::Point destination{5.0, 0.0};

    sdc::PlannerResult result = sdc::ConstructPolylineUsingVoronoi(
        points, localization, destination);

    std::cout << "Path (" << result.path.size() << " points):\n";
    for (const auto& pt : result.path) {
        std::cout << pt.x << " " << pt.y << "\n";
    }

    viz::VisualizationData visData{
        result.sites,
        result.voronoi,
        localization,
        destination,
        result.path
    };
    viz::SaveSvg(visData, "output.svg");
    std::cout << "Visualization saved to output.svg\n";

    return 0;
}
