#include "visualization/visualization.h"

#include <fstream>
#include <limits>
#include <sstream>
#include <string>
#include <cmath>

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

    for (const auto& poly : data.polygons) {
        for (const auto& pt : poly.vertices) {
            expand(pt.x, pt.y);
        }
    }
    for (const auto& pt : data.path) {
        expand(pt.x, pt.y);
    }
    expand(data.start.x, data.start.y);
    expand(data.end.x,   data.end.y);

    // 20% padding
    const double pad_x = (bb.max_x - bb.min_x) * 0.2;
    const double pad_y = (bb.max_y - bb.min_y) * 0.2;
    bb.min_x -= pad_x; bb.max_x += pad_x;
    bb.min_y -= pad_y; bb.max_y += pad_y;

    return bb;
}

struct Transform {
    double scale;
    double margin;
    double min_x;
    double max_y;
    double canvas_size;

    double tx(double wx) const { return (wx - min_x) * scale + margin; }
    double ty(double wy) const { return (max_y - wy) * scale + margin; }

    bool inView(double sx, double sy) const {
        const double lo = -margin * 2.0;
        const double hi = canvas_size + margin * 2.0;
        return sx >= lo && sx <= hi && sy >= lo && sy <= hi;
    }
};

Transform MakeTransform(const BBox& bb, double canvas_size = 800.0, double margin = 40.0) {
    const double w = bb.max_x - bb.min_x;
    const double h = bb.max_y - bb.min_y;
    const double extent = std::max(w, h);
    const double scale = (extent > 1e-9) ? (canvas_size - 2.0 * margin) / extent : 1.0;
    return Transform{scale, margin, bb.min_x, bb.max_y, canvas_size};
}

std::string Pt(const Transform& tr, double wx, double wy) {
    std::ostringstream ss;
    ss << tr.tx(wx) << "," << tr.ty(wy);
    return ss.str();
}

} // anonymous namespace

void SaveSvg(const VisualizationData& data, const std::string& filename) {
    const BBox bb = ComputeBBox(data);
    const Transform tr = MakeTransform(bb);

    const double canvas_size = 800.0;
    const double margin = 40.0;
    const double total = canvas_size + 2.0 * margin;

    std::ofstream out(filename);
    if (!out.is_open()) return;

    // SVG header
    out << R"(<?xml version="1.0" encoding="UTF-8"?>)" << "\n";
    out << "<svg xmlns=\"http://www.w3.org/2000/svg\""
        << " width=\"" << total << "\" height=\"" << total << "\""
        << " viewBox=\"0 0 " << total << " " << total << "\">\n";

    // Clip path
    out << "  <defs>\n";
    out << "    <clipPath id=\"scene\">\n";
    out << "      <rect x=\"" << margin << "\" y=\"" << margin
        << "\" width=\"" << canvas_size - 2.0 * margin
        << "\" height=\"" << canvas_size - 2.0 * margin << "\"/>\n";
    out << "    </clipPath>\n";
    out << "  </defs>\n";

    // Background
    out << "  <rect width=\"" << total << "\" height=\"" << total
        << "\" fill=\"#f8f8f8\"/>\n";

    // Scene border
    out << "  <rect x=\"" << margin << "\" y=\"" << margin
        << "\" width=\"" << canvas_size - 2.0 * margin
        << "\" height=\"" << canvas_size - 2.0 * margin
        << "\" fill=\"none\" stroke=\"#dddddd\" stroke-width=\"1\"/>\n";

    // --- Layer 1: Voronoi edges (clipped) ---
    out << "  <!-- Voronoi edges -->\n";
    out << "  <g stroke=\"#b0c4de\" stroke-width=\"1\" fill=\"none\" clip-path=\"url(#scene)\">\n";
    for (const auto& e : data.voronoi_edges) {
        if (!e.finite) continue;
        const double x1 = tr.tx(e.x1), y1 = tr.ty(e.y1);
        const double x2 = tr.tx(e.x2), y2 = tr.ty(e.y2);
        if (!tr.inView(x1, y1) && !tr.inView(x2, y2)) continue;
        out << "    <line"
            << " x1=\"" << x1 << "\" y1=\"" << y1 << "\""
            << " x2=\"" << x2 << "\" y2=\"" << y2 << "\""
            << "/>\n";
    }
    out << "  </g>\n";

    // --- Layer 2: Obstacle polygons ---
    out << "  <!-- Obstacle polygons -->\n";
    out << "  <g fill=\"#d0d0d0\" fill-opacity=\"0.6\" stroke=\"#555555\" stroke-width=\"1.5\""
        << " clip-path=\"url(#scene)\">\n";
    for (const auto& poly : data.polygons) {
        if (poly.vertices.size() < 2) continue;
        out << "    <polygon points=\"";
        for (const auto& pt : poly.vertices) {
            out << Pt(tr, pt.x, pt.y) << " ";
        }
        out << "\"/>\n";
    }
    out << "  </g>\n";

    // --- Layer 3: Path ---
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

    // --- Layer 4: Start point ---
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

    // --- Layer 5: End point ---
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
    out << "    <line x1=\"10\" y1=\"18\" x2=\"35\" y2=\"18\" stroke=\"#b0c4de\" stroke-width=\"1.5\"/>\n";
    out << "    <text x=\"42\" y=\"22\" fill=\"#555\">Voronoi edges</text>\n";
    out << "    <line x1=\"10\" y1=\"36\" x2=\"35\" y2=\"36\" stroke=\"#e74c3c\" stroke-width=\"3\"/>\n";
    out << "    <text x=\"42\" y=\"40\" fill=\"#555\">Path</text>\n";
    out << "    <rect x=\"10\" y=\"46\" width=\"25\" height=\"14\" fill=\"#d0d0d0\" stroke=\"#555555\" stroke-width=\"1\"/>\n";
    out << "    <text x=\"42\" y=\"58\" fill=\"#555\">Obstacles</text>\n";
    out << "    <circle cx=\"22\" cy=\"72\" r=\"7\" fill=\"#2ecc71\"/>\n";
    out << "    <text x=\"42\" y=\"76\" fill=\"#555\">Start</text>\n";
    out << "    <circle cx=\"22\" cy=\"90\" r=\"7\" fill=\"#3498db\"/>\n";
    out << "    <text x=\"42\" y=\"94\" fill=\"#555\">End</text>\n";
    out << "  </g>\n";

    out << "</svg>\n";
}

} // namespace viz
