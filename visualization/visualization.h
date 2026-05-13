#pragma once

#include <string>
#include <vector>
#include "geometry/geometry.h"

namespace viz {

    struct VisualizationData {
        const std::vector<geom::Polygon>& polygons;
        geom::Point                       start;
        geom::Point                       end;
        const geom::Polyline&             path;
    };

    void SaveSvg(const VisualizationData& data, const std::string& filename);

} // namespace viz
