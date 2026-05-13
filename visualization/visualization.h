#pragma once

#include <string>
#include <vector>
#include "geometry/geometry.h"
#include "voronoi_planner.h"

namespace viz {

    struct VisualizationData {
        const std::vector<geom::Polygon>&       polygons;
        const std::vector<sdc::VoronoiEdgeViz>& voronoi_edges;
        geom::Point                             start;
        geom::Point                             end;
        const geom::Polyline&                   path;
    };

    void SaveSvg(const VisualizationData& data, const std::string& filename);

} // namespace viz
