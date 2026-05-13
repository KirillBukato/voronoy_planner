#include "visualization/visualization.h"

#include <cmath>
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
                std::numeric_limits<double>::lowest()};
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
            expand(data.end.x, data.end.y);

            const double px = (bb.max_x - bb.min_x) * 0.2;
            const double py = (bb.max_y - bb.min_y) * 0.2;
            bb.min_x -= px;
            bb.max_x += px;
            bb.min_y -= py;
            bb.max_y += py;
            return bb;
        }

        struct Transform {
            double scale, margin, min_x, max_y, canvas_size;
            double tx(double wx) const {
                return (wx - min_x) * scale + margin;
            }
            double ty(double wy) const {
                return (max_y - wy) * scale + margin;
            }
        };

        Transform MakeTransform(const BBox& bb, double canvas = 800.0, double margin = 40.0) {
            const double w = bb.max_x - bb.min_x, h = bb.max_y - bb.min_y;
            const double ext = std::max(w, h);
            const double s = (ext > 1e-9) ? (canvas - 2.0 * margin) / ext : 1.0;
            return {s, margin, bb.min_x, bb.max_y, canvas};
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
        const double canvas = 800.0, margin = 40.0, total = canvas + 2.0 * margin;

        std::ofstream out(filename);
        if (!out.is_open()) {
            return;
        }

        out << R"(<?xml version="1.0" encoding="UTF-8"?>)" << "\n";
        out << "<svg xmlns=\"http://www.w3.org/2000/svg\""
            << " width=\"" << total << "\" height=\"" << total << "\""
            << " viewBox=\"0 0 " << total << " " << total << "\">\n";

        // Background
        out << "  <rect width=\"" << total << "\" height=\"" << total << "\" fill=\"#f8f8f8\"/>\n";

        // Scene border
        out << "  <rect x=\"" << margin << "\" y=\"" << margin
            << "\" width=\"" << canvas - 2.0 * margin
            << "\" height=\"" << canvas - 2.0 * margin
            << "\" fill=\"none\" stroke=\"#dddddd\" stroke-width=\"1\"/>\n";

        // --- Obstacle polygons ---
        out << "  <!-- Obstacles -->\n";
        out << "  <g fill=\"#c0c8d8\" fill-opacity=\"0.85\" stroke=\"#445566\" stroke-width=\"1.5\""
            << " stroke-linejoin=\"round\">\n";
        for (const auto& poly : data.polygons) {
            if (poly.vertices.size() < 2) {
                continue;
            }
            out << "    <polygon points=\"";
            for (const auto& pt : poly.vertices) {
                out << Pt(tr, pt.x, pt.y) << " ";
            }
            out << "\"/>\n";
        }
        out << "  </g>\n";

        // --- Path ---
        out << "  <!-- Path -->\n";
        if (data.path.size() >= 2) {
            // Draw path shadow for readability
            out << "  <polyline fill=\"none\" stroke=\"#ffffff\" stroke-width=\"5\""
                << " stroke-linejoin=\"round\" stroke-linecap=\"round\" points=\"";
            for (const auto& pt : data.path) {
                out << Pt(tr, pt.x, pt.y) << " ";
            }
            out << "\"/>\n";
            // Draw path
            out << "  <polyline fill=\"none\" stroke=\"#e74c3c\" stroke-width=\"2.5\""
                << " stroke-linejoin=\"round\" stroke-linecap=\"round\" points=\"";
            for (const auto& pt : data.path) {
                out << Pt(tr, pt.x, pt.y) << " ";
            }
            out << "\"/>\n";
            // Draw waypoint dots
            out << "  <g fill=\"#e74c3c\">\n";
            for (size_t i = 1; i + 1 < data.path.size(); i++) {
                const auto& pt = data.path[i];
                out << "    <circle cx=\"" << tr.tx(pt.x) << "\" cy=\"" << tr.ty(pt.y) << "\" r=\"2.5\"/>\n";
            }
            out << "  </g>\n";
        }

        // --- Start ---
        {
            const double cx = tr.tx(data.start.x), cy = tr.ty(data.start.y);
            out << "  <circle cx=\"" << cx << "\" cy=\"" << cy
                << "\" r=\"9\" fill=\"#2ecc71\" stroke=\"white\" stroke-width=\"2\"/>\n";
            out << "  <text x=\"" << cx << "\" y=\"" << cy + 4.5
                << "\" text-anchor=\"middle\" font-size=\"9\" font-family=\"sans-serif\""
                << " fill=\"white\" font-weight=\"bold\">S</text>\n";
        }

        // --- End ---
        {
            const double cx = tr.tx(data.end.x), cy = tr.ty(data.end.y);
            out << "  <circle cx=\"" << cx << "\" cy=\"" << cy
                << "\" r=\"9\" fill=\"#3498db\" stroke=\"white\" stroke-width=\"2\"/>\n";
            out << "  <text x=\"" << cx << "\" y=\"" << cy + 4.5
                << "\" text-anchor=\"middle\" font-size=\"9\" font-family=\"sans-serif\""
                << " fill=\"white\" font-weight=\"bold\">E</text>\n";
        }

        // Legend
        out << "  <g font-family=\"sans-serif\" font-size=\"12\">\n";
        out << "    <line x1=\"10\" y1=\"18\" x2=\"35\" y2=\"18\" stroke=\"#e74c3c\" stroke-width=\"2.5\"/>\n";
        out << "    <text x=\"42\" y=\"22\" fill=\"#555\">Path</text>\n";
        out << "    <rect x=\"10\" y=\"30\" width=\"25\" height=\"14\" fill=\"#c0c8d8\" stroke=\"#445566\" stroke-width=\"1\"/>\n";
        out << "    <text x=\"42\" y=\"42\" fill=\"#555\">Obstacles</text>\n";
        out << "    <circle cx=\"22\" cy=\"58\" r=\"7\" fill=\"#2ecc71\"/>\n";
        out << "    <text x=\"42\" y=\"62\" fill=\"#555\">Start</text>\n";
        out << "    <circle cx=\"22\" cy=\"76\" r=\"7\" fill=\"#3498db\"/>\n";
        out << "    <text x=\"42\" y=\"80\" fill=\"#555\">End</text>\n";
        out << "  </g>\n";

        out << "</svg>\n";
    }

} // namespace viz
