#include <iostream>
#include "geometry/geometry.h"
#include "voronoi_planner.h"

int main() {
    std::vector<geom::Point> obstacles = {{-1, -1}, {3, -1}, {2, 0}, {3, 1}, {-1, 1}, {1, -1}, {1, 1}, {0, 0}};
    geom::Point localization{-2, 0};
    geom::Point destination{4, 0};

    geom::Polyline polyline = sdc::ConstructPolylineUsingVoronoi(std::move(obstacles), localization, destination);

    for (const auto& el : polyline) {
        std::cout << el.x << " " << el.y << "\n";
    }

    return 0;
}