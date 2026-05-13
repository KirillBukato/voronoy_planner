#include <iostream>
#include "geometry/geometry.h"
#include "voronoi_planner.h"
#include "visualization/visualization.h"

int main() {
    // Road scene with slightly irregular (non-axis-aligned) polygons
    // to avoid cocircular degeneracies in Fortune's algorithm.
    //
    // Road runs roughly along X axis from x=0 to x=20.
    // Top boundary and bottom boundary are slightly tilted trapezoids.
    // Agents are slightly rotated rectangles.

    std::vector<geom::Polygon> obstacles = {
        // Top wall (slightly tilted)
        geom::Polygon({{-1.1, 1.9}, {21.3, 2.1}, {21.1, 4.2}, {-0.9, 3.8}}),
        // Bottom wall (slightly tilted)
        geom::Polygon({{-0.8, -4.1}, {21.2, -3.9}, {21.4, -1.8}, {-1.2, -2.1}}),

        // Agent 1: slightly rotated rectangle near x=4
        geom::Polygon({{2.9, -0.1}, {5.1, 0.1}, {4.9, 1.1}, {2.7, 0.9}}),
        // Agent 2: slightly rotated rectangle near x=8
        geom::Polygon({{6.8, -1.1}, {9.2, -0.9}, {9.1, 0.1}, {6.9, -0.1}}),
        // Agent 3: slightly rotated rectangle near x=12
        geom::Polygon({{10.9, 0.1}, {13.1, -0.1}, {13.2, 1.0}, {11.0, 1.2}}),
        // Agent 4: slightly rotated rectangle near x=16
        geom::Polygon({{14.8, -1.2}, {17.2, -0.8}, {17.1, 0.2}, {14.9, -0.2}}),
    };

    geom::Point localization{0.3, 0.1};
    geom::Point destination{19.7, -0.1};

    sdc::PlannerResult result = sdc::ConstructPolylineUsingVoronoi(
        obstacles, localization, destination, /*sampling_step=*/0.5);

    std::cout << "Path (" << result.path.size() << " points):\n";
    for (const auto& pt : result.path) {
        std::cout << "  " << pt.x << " " << pt.y << "\n";
    }

    viz::VisualizationData visData{
        result.sites,
        result.voronoi,
        localization,
        destination,
        result.path,
        obstacles
    };
    viz::SaveSvg(visData, "output.svg");
    std::cout << "Visualization saved to output.svg\n";

    return 0;
}
