#pragma once

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#include "geometry/geometry.h"

namespace geom {

    struct AABB {
        double min_x = std::numeric_limits<double>::max();
        double min_y = std::numeric_limits<double>::max();
        double max_x = std::numeric_limits<double>::lowest();
        double max_y = std::numeric_limits<double>::lowest();

        AABB() = default;

        AABB(double min_x, double min_y, double max_x, double max_y)
            : min_x(min_x)
            , min_y(min_y)
            , max_x(max_x)
            , max_y(max_y) {
        }

        bool Contains(const Point& p) const {
            return p.x >= min_x && p.x <= max_x &&
                   p.y >= min_y && p.y <= max_y;
        }

        bool Intersects(const AABB& other) const {
            return min_x <= other.max_x && max_x >= other.min_x &&
                   min_y <= other.max_y && max_y >= other.min_y;
        }

        void Expand(const AABB& other) {
            min_x = std::min(min_x, other.min_x);
            min_y = std::min(min_y, other.min_y);
            max_x = std::max(max_x, other.max_x);
            max_y = std::max(max_y, other.max_y);
        }

        double Area() const {
            return (max_x - min_x) * (max_y - min_y);
        }
    };

    inline AABB ComputeAABB(const Polygon& poly) {
        AABB bb;
        for (const auto& v : poly.vertices) {
            bb.min_x = std::min(bb.min_x, v.x);
            bb.min_y = std::min(bb.min_y, v.y);
            bb.max_x = std::max(bb.max_x, v.x);
            bb.max_y = std::max(bb.max_y, v.y);
        }
        return bb;
    }

    class RTree {
    public:
        RTree() = default;

        void Build(const std::vector<Polygon>& polygons) {
            entries_.clear();
            nodes_.clear();
            children_.clear();
            if (polygons.empty()) {
                root_ = kNone;
                return;
            }

            const size_t n = polygons.size();
            entries_.reserve(n);
            for (size_t i = 0; i < n; ++i) {
                entries_.push_back({ComputeAABB(polygons[i]), i});
            }

            root_ = BuildSTR(0, n);
        }

        template <typename Callback>
        void Query(const Point& point, Callback&& callback) const {
            if (root_ == kNone) {
                return;
            }
            QueryNode(root_, point, callback);
        }

    private:
        static constexpr size_t kNone = std::numeric_limits<size_t>::max();
        static constexpr size_t kMaxChildren = 8;

        struct Entry {
            AABB bbox;
            size_t polygon_index;
        };

        struct Node {
            AABB bbox;
            bool is_leaf = false;
            size_t begin = 0;
            size_t end = 0;
        };

        std::vector<Entry> entries_;
        std::vector<Node> nodes_;
        std::vector<size_t> children_;
        size_t root_ = kNone;

        size_t BuildSTR(size_t lo, size_t hi) {
            const size_t count = hi - lo;
            if (count <= kMaxChildren) {
                Node node;
                node.is_leaf = true;
                node.begin = lo;
                node.end = hi;
                node.bbox = entries_[lo].bbox;
                for (size_t i = lo + 1; i < hi; ++i) {
                    node.bbox.Expand(entries_[i].bbox);
                }
                nodes_.push_back(node);
                return nodes_.size() - 1;
            }

            std::sort(entries_.begin() + lo, entries_.begin() + hi,
                      [](const Entry& a, const Entry& b) {
                          const double ax = (a.bbox.min_x + a.bbox.max_x) * 0.5;
                          const double bx = (b.bbox.min_x + b.bbox.max_x) * 0.5;
                          return ax < bx;
                      });

            const size_t num_leaves = (count + kMaxChildren - 1) / kMaxChildren;
            const size_t num_slices_x = static_cast<size_t>(std::ceil(std::sqrt(static_cast<double>(num_leaves))));
            const size_t slice_size = num_slices_x * kMaxChildren;

            std::vector<size_t> child_indices;
            for (size_t s = lo; s < hi; s += slice_size) {
                const size_t s_end = std::min(s + slice_size, hi);
                std::sort(entries_.begin() + s, entries_.begin() + s_end,
                          [](const Entry& a, const Entry& b) {
                              const double ay = (a.bbox.min_y + a.bbox.max_y) * 0.5;
                              const double by = (b.bbox.min_y + b.bbox.max_y) * 0.5;
                              return ay < by;
                          });

                for (size_t g = s; g < s_end; g += kMaxChildren) {
                    const size_t g_end = std::min(g + kMaxChildren, s_end);
                    child_indices.push_back(BuildSTR(g, g_end));
                }
            }

            if (child_indices.size() == 1) {
                return child_indices[0];
            }

            const size_t cb = children_.size();
            for (size_t ci : child_indices) {
                children_.push_back(ci);
            }
            const size_t ce = children_.size();

            Node node;
            node.is_leaf = false;
            node.begin = cb;
            node.end = ce;
            node.bbox = nodes_[child_indices[0]].bbox;
            for (size_t i = 1; i < child_indices.size(); ++i) {
                node.bbox.Expand(nodes_[child_indices[i]].bbox);
            }
            nodes_.push_back(node);
            return nodes_.size() - 1;
        }

        template <typename Callback>
        void QueryNode(size_t node_idx, const Point& point, Callback& callback) const {
            const Node& node = nodes_[node_idx];
            if (!node.bbox.Contains(point)) {
                return;
            }
            if (node.is_leaf) {
                for (size_t i = node.begin; i < node.end; ++i) {
                    if (entries_[i].bbox.Contains(point)) {
                        callback(entries_[i].polygon_index);
                    }
                }
            } else {
                for (size_t i = node.begin; i < node.end; ++i) {
                    QueryNode(children_[i], point, callback);
                }
            }
        }
    };

} // namespace geom
