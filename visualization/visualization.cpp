#include "visualization/visualization.h"

#include <fstream>
#include <limits>
#include <sstream>
#include <string>

namespace viz {

namespace {

    struct BBox {
        double min_x, min_y, max_x, max_y;
    };

    BBox ComputeBBox(const VisualizationData& data) {
        BBox bb{
            std::numeric_limits<double>::max(),
            std::numeric_limits<double>::max(),
            std::numeric_limits<double>::lowest(),
            std::numeric_limits<double>::lowest()
        };

        const auto expand = [&](double x, double y) {
            bb.min_x = std::min(bb.min_x, x);
            bb.min_y = std::min(bb.min_y, y);
            bb.max_x = std::max(bb.max_x, x);
            bb.max_y = std::max(bb.max_y, y);
        };

        // Use sites, path, start/end, and polygon vertices for bbox.
        // Voronoi circumcenters can be arbitrarily far from the scene.
        for (const auto& pt : data.sites) {
            expand(pt.x, pt.y);
        }
        for (const auto& pt : data.path) {
            expand(pt.x, pt.y);
        }
        expand(data.start.x, data.start.y);
        expand(data.end.x, data.end.y);
        for (const auto& poly : data.polygons) {
            for (const auto& pt : poly.vertices) {
                expand(pt.x, pt.y);
            }
        }

        // 15% padding
        const double pad_x = (bb.max_x - bb.min_x) * 0.15 + 0.5;
        const double pad_y = (bb.max_y - bb.min_y) * 0.15 + 0.5;
        bb.min_x -= pad_x; bb.max_x += pad_x;
        bb.min_y -= pad_y; bb.max_y += pad_y;

        return bb;
    }

    struct Transform {
        double scale;
        double margin;
        double min_x;
        double max_y;
        double canvas_w;
        double canvas_h;

        double tx(double wx) const { return (wx - min_x) * scale + margin; }
        double ty(double wy) const { return (max_y - wy) * scale + margin; }

        bool inView(double sx, double sy) const {
            const double lo = -margin * 2;
            const double hiw = canvas_w + margin * 2;
            const double hih = canvas_h + margin * 2;
            return sx >= lo && sx <= hiw && sy >= lo && sy <= hih;
        }
    };

    Transform MakeTransform(const BBox& bb, double canvas_w = 900.0, double canvas_h = 400.0, double margin = 40.0) {
        const double w = bb.max_x - bb.min_x;
        const double h = bb.max_y - bb.min_y;
        const double scale_x = (w > 1e-9) ? (canvas_w - 2.0 * margin) / w : 1.0;
        const double scale_y = (h > 1e-9) ? (canvas_h - 2.0 * margin) / h : 1.0;
        const double scale = std::min(scale_x, scale_y);
        return Transform{scale, margin, bb.min_x, bb.max_y, canvas_w, canvas_h};
    }

    std::string Pt(const Transform& tr, double wx, double wy) {
        std::ostringstream ss;
        ss << tr.tx(wx) << "," << tr.ty(wy);
        return ss.str();
    }

} // anonymous namespace

void SaveSvg(const VisualizationData& data, const std::string& filename) {
    const BBox bb = ComputeBBox(data);
    const double canvas_w = 900.0;
    const double canvas_h = 400.0;
    const double margin = 40.0;
    const Transform tr = MakeTransform(bb, canvas_w, canvas_h, margin);

    const double total_w = canvas_w + 2.0 * margin;
    const double total_h = canvas_h + 2.0 * margin;

    std::ofstream out(filename);
    if (!out.is_open()) return;

    // SVG header
    out << R"(<?xml version="1.0" encoding="UTF-8"?>)" << "\n";
    out << "<svg xmlns=\"http://www.w3.org/2000/svg\""
        << " width=\"" << total_w << "\" height=\"" << total_h << "\""
        << " viewBox=\"0 0 " << total_w << " " << total_h << "\">\n";

    // Clip path
    out << "  <defs>\n";
    out << "    <clipPath id=\"scene\">\n";
    out << "      <rect x=\"" << margin << "\" y=\"" << margin
        << "\" width=\"" << canvas_w - 2.0 * margin
        << "\" height=\"" << canvas_h - 2.0 * margin << "\"/>\n";
    out << "    </clipPath>\n";
    out << "  </defs>\n";

    // Background
    out << "  <rect width=\"" << total_w << "\" height=\"" << total_h
        << "\" fill=\"#f8f8f8\"/>\n";

    // Scene border
    out << "  <rect x=\"" << margin << "\" y=\"" << margin
        << "\" width=\"" << canvas_w - 2.0 * margin
        << "\" height=\"" << canvas_h - 2.0 * margin
        << "\" fill=\"none\" stroke=\"#dddddd\" stroke-width=\"1\"/>\n";

    // --- Layer 1: Voronoi edges (clipped) ---
    out << "  <!-- Voronoi edges -->\n";
    out << "  <g stroke=\"#cccccc\" stroke-width=\"0.8\" fill=\"none\" clip-path=\"url(#scene)\">\n";
    const auto inf = data.voronoi.inf;
    for (const auto& edge : data.voronoi.edges) {
        if (edge.b == inf || edge.e == inf) continue;
        const auto& vb = data.voronoi.vertices[edge.b].c;
        const auto& ve = data.voronoi.vertices[edge.e].c;
        const double x1 = tr.tx(vb.x), y1 = tr.ty(vb.y);
        const double x2 = tr.tx(ve.x), y2 = tr.ty(ve.y);
        if (!tr.inView(x1, y1) && !tr.inView(x2, y2)) continue;
        out << "    <line"
            << " x1=\"" << x1 << "\" y1=\"" << y1 << "\""
            << " x2=\"" << x2 << "\" y2=\"" << y2 << "\""
            << "/>\n";
    }
    out << "  </g>\n";

    // --- Layer 2: Polygons (filled obstacles) ---
    out << "  <!-- Polygons -->\n";
    out << "  <g fill=\"#c0392b\" fill-opacity=\"0.35\" stroke=\"#c0392b\" stroke-width=\"1.5\" clip-path=\"url(#scene)\">\n";
    for (const auto& poly : data.polygons) {
        if (poly.vertices.empty()) continue;
        out << "    <polygon points=\"";
        for (const auto& pt : poly.vertices) {
            out << Pt(tr, pt.x, pt.y) << " ";
        }
        out << "\"/>\n";
    }
    out << "  </g>\n";

    // --- Layer 3: Obstacle sites (sampled points) ---
    out << "  <!-- Obstacle sites -->\n";
    out << "  <g fill=\"#7f8c8d\" fill-opacity=\"0.6\">\n";
    for (const auto& pt : data.sites) {
        const double cx = tr.tx(pt.x), cy = tr.ty(pt.y);
        if (!tr.inView(cx, cy)) continue;
        out << "    <circle cx=\"" << cx << "\" cy=\"" << cy << "\" r=\"2\"/>\n";
    }
    out << "  </g>\n";

    // --- Layer 4: Path ---
    out << "  <!-- Path -->\n";
    if (data.path.size() >= 2) {
        out << "  <polyline fill=\"none\" stroke=\"#e74c3c\" stroke-width=\"3\""
            << " stroke-linejoin=\"round\" stroke-linecap=\"round\"\n"
            << "    points=\"";
        for (const auto& pt : data.path) {
            out << Pt(tr, pt.x, pt.y) << " ";
        }
        out << "\"/>\n";
    }

    // --- Layer 5: Start point ---
    out << "  <!-- Start -->\n";
    {
        const double cx = tr.tx(data.start.x);
        const double cy = tr.ty(data.start.y);
        out << "  <circle cx=\"" << cx << "\" cy=\"" << cy
            << "\" r=\"9\" fill=\"#2ecc71\" stroke=\"white\" stroke-width=\"2\"/>\n";
        out << "  <text x=\"" << cx << "\" y=\"" << cy + 4.5
            << "\" text-anchor=\"middle\" font-size=\"9\" font-family=\"sans-serif\""
            << " fill=\"white\" font-weight=\"bold\">S</text>\n";
    }

    // --- Layer 6: End point ---
    out << "  <!-- End -->\n";
    {
        const double cx = tr.tx(data.end.x);
        const double cy = tr.ty(data.end.y);
        out << "  <circle cx=\"" << cx << "\" cy=\"" << cy
            << "\" r=\"9\" fill=\"#3498db\" stroke=\"white\" stroke-width=\"2\"/>\n";
        out << "  <text x=\"" << cx << "\" y=\"" << cy + 4.5
            << "\" text-anchor=\"middle\" font-size=\"9\" font-family=\"sans-serif\""
            << " fill=\"white\" font-weight=\"bold\">E</text>\n";
    }

    // Legend
    out << "  <!-- Legend -->\n";
    out << "  <g font-family=\"sans-serif\" font-size=\"12\">\n";
    out << "    <line x1=\"10\" y1=\"18\" x2=\"35\" y2=\"18\" stroke=\"#cccccc\" stroke-width=\"1.5\"/>\n";
    out << "    <text x=\"42\" y=\"22\" fill=\"#555\">Voronoi edges</text>\n";
    out << "    <line x1=\"10\" y1=\"36\" x2=\"35\" y2=\"36\" stroke=\"#e74c3c\" stroke-width=\"3\"/>\n";
    out << "    <text x=\"42\" y=\"40\" fill=\"#555\">Path</text>\n";
    out << "    <rect x=\"12\" y=\"46\" width=\"20\" height=\"12\" fill=\"#c0392b\" fill-opacity=\"0.35\" stroke=\"#c0392b\" stroke-width=\"1\"/>\n";
    out << "    <text x=\"42\" y=\"58\" fill=\"#555\">Obstacles</text>\n";
    out << "    <circle cx=\"22\" cy=\"72\" r=\"7\" fill=\"#2ecc71\"/>\n";
    out << "    <text x=\"42\" y=\"76\" fill=\"#555\">Start</text>\n";
    out << "    <circle cx=\"22\" cy=\"90\" r=\"7\" fill=\"#3498db\"/>\n";
    out << "    <text x=\"42\" y=\"94\" fill=\"#555\">End</text>\n";
    out << "  </g>\n";

    out << "</svg>\n";
}

} // namespace viz
