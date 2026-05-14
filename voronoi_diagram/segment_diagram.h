#pragma once

#include <algorithm>
#include <cmath>
#include <limits>
#include <unordered_map>
#include <vector>

#include <boost/polygon/voronoi.hpp>

#include "geometry/geometry.h"

namespace seg_diag_detail {

    using coord_t = int32_t;

    struct IPoint {
        coord_t x, y;
        IPoint() = default;
        IPoint(coord_t x, coord_t y)
            : x(x)
            , y(y) {
        }
    };

    struct ISegment {
        IPoint p0, p1;
        ISegment() = default;
        ISegment(IPoint a, IPoint b)
            : p0(a)
            , p1(b) {
        }
    };

} // namespace seg_diag_detail

namespace boost {
    namespace polygon {

        template <>
        struct geometry_concept<seg_diag_detail::IPoint> {
            typedef point_concept type;
        };

        template <>
        struct point_traits<seg_diag_detail::IPoint> {
            typedef seg_diag_detail::coord_t coordinate_type;
            static inline coordinate_type get(const seg_diag_detail::IPoint& p, orientation_2d o) {
                return (o == HORIZONTAL) ? p.x : p.y;
            }
        };

        template <>
        struct geometry_concept<seg_diag_detail::ISegment> {
            typedef segment_concept type;
        };

        template <>
        struct segment_traits<seg_diag_detail::ISegment> {
            typedef seg_diag_detail::coord_t coordinate_type;
            typedef seg_diag_detail::IPoint point_type;
            static inline point_type get(const seg_diag_detail::ISegment& s, direction_1d d) {
                return (d == LOW) ? s.p0 : s.p1;
            }
        };

    } // namespace polygon
} // namespace boost

namespace seg_diag {

    static constexpr double SCALE = 1e5;

    using Graph = std::vector<std::unordered_map<size_t, double>>;

    struct VoronoiGraph {
        std::vector<geom::Point> vertices;
        Graph adjacency;
    };

    inline VoronoiGraph BuildVoronoiGraph(
        const std::vector<geom::Polygon>& polygons,
        const geom::Point& start,
        const geom::Point& end) {
        using IPoint = seg_diag_detail::IPoint;
        using ISegment = seg_diag_detail::ISegment;
        using coord_t = seg_diag_detail::coord_t;
        using VoronoiDiagram = boost::polygon::voronoi_diagram<double>;

        double x0 = std::min(start.x, end.x), y0 = std::min(start.y, end.y);
        double x1 = std::max(start.x, end.x), y1 = std::max(start.y, end.y);
        for (const auto& poly : polygons) {
            for (const auto& pt : poly.vertices) {
                x0 = std::min(x0, pt.x);
                y0 = std::min(y0, pt.y);
                x1 = std::max(x1, pt.x);
                y1 = std::max(y1, pt.y);
            }
        }
        const double margin = std::max(x1 - x0, y1 - y0) * 2.0;

        const auto to_int = [](double v) -> coord_t {
            return static_cast<coord_t>(std::round(v * SCALE));
        };

        std::vector<ISegment> segments;
        for (const auto& polygon : polygons) {
            for (size_t i = 0; i < polygon.vertices.size(); i++) {
                const auto& a = polygon.vertices[i];
                const auto& b = polygon.vertices[(i + 1) % polygon.vertices.size()];
                IPoint int_a{to_int(a.x), to_int(a.y)};
                IPoint int_b{to_int(b.x), to_int(b.y)};
                if (int_a.x == int_b.x && int_a.y == int_b.y) {
                    continue;
                }
                segments.push_back({int_a, int_b});
            }
        }

        if (segments.empty()) {
            return {};
        }

        VoronoiDiagram voronoi_diagram;
        boost::polygon::construct_voronoi(segments.begin(), segments.end(), &voronoi_diagram);

        if (voronoi_diagram.num_vertices() == 0) {
            return {};
        }

        VoronoiGraph result;
        result.vertices.reserve(voronoi_diagram.num_vertices());
        std::vector<size_t> indexes(voronoi_diagram.num_vertices(), std::numeric_limits<size_t>::max());
        for (size_t i = 0; i < voronoi_diagram.num_vertices(); i++) {
            const auto& v = voronoi_diagram.vertices()[i];
            geom::Point point{v.x() / SCALE, v.y() / SCALE};

            bool is_outside = true;
            for (const auto& polygon : polygons) {
                if (geom::PointInPolygon(point, polygon)) {
                    is_outside = false;
                }
            }

            if (is_outside) {
                indexes[i] = result.vertices.size();
                result.vertices.push_back(point);
            }
        }

        result.adjacency.resize(result.vertices.size());

        std::unordered_map<const VoronoiDiagram::vertex_type*, size_t> vmap;
        vmap.reserve(voronoi_diagram.num_vertices());
        {
            for (size_t i = 0; i < voronoi_diagram.num_vertices(); i++) {
                vmap[&voronoi_diagram.vertices()[i]] = i;
            }
        }

        const auto out_of_bounds = [&](geom::Point point) {
            return point.x < x0 - margin || point.x > x1 + margin ||
                   point.y < y0 - margin || point.y > y1 + margin;
        };

        for (const auto& edge : voronoi_diagram.edges()) {
            if (!edge.is_primary() || !edge.is_finite()) {
                continue;
            }
            const auto* va = edge.vertex0();
            const auto* vb = edge.vertex1();
            if (!va || !vb) {
                continue;
            }

            auto ia = vmap.find(va), ib = vmap.find(vb);
            if (ia == vmap.end() || ib == vmap.end()) {
                continue;
            }
            if (indexes[ia->second] == std::numeric_limits<size_t>::max() || indexes[ib->second] == std::numeric_limits<size_t>::max()) {
                continue;
            }

            const geom::Point a{va->x() / SCALE, va->y() / SCALE};
            const geom::Point b{vb->x() / SCALE, vb->y() / SCALE};
            if (out_of_bounds(a) || out_of_bounds(b)) {
                // continue;
            }

            const double distance = geom::Distance(a, b);

            result.adjacency[indexes[ia->second]][indexes[ib->second]] = distance;
            result.adjacency[indexes[ib->second]][indexes[ia->second]] = distance;
        }

        return result;
    }

} // namespace seg_diag
