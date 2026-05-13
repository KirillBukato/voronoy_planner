#pragma once
#include <algorithm>
#include <fstream>
#include <vector>
#include <list>
#include <queue>
#include <limits>
#include <unordered_map>
#include <stack>

namespace dijkstra {
    struct State {
        double distance;
        size_t vertex;

        State(double d, size_t v)
            : distance(d)
            , vertex(v) {
        }

        bool operator<(const State& other) const {
            return distance > other.distance;
        }
    };

    inline std::vector<size_t> FindShortestPath(const std::vector<std::unordered_map<size_t, double>>& graph, size_t src, size_t dst) {
        std::vector<double> distances(graph.size(), std::numeric_limits<double>::max());
        std::vector<size_t> prev(graph.size(), -1);
        distances[src] = 0;
        std::priority_queue<State> queue;
        queue.emplace(0, src);
        while (!queue.empty()) {
            State state = queue.top();
            queue.pop();
            if (state.distance > distances[state.vertex]) {
                continue;
            }
            for (auto [neighbour, cost] : graph[state.vertex]) {
                double new_distance = state.distance + cost;
                if (new_distance < distances[neighbour]) {
                    distances[neighbour] = new_distance;
                    prev[neighbour] = state.vertex;
                    queue.emplace(new_distance, neighbour);
                }
            }
        }
        std::vector<size_t> result;
        if (prev[dst] == -1) {
            return result;
        }
        size_t idx = dst;
        while (idx != -1) {
            result.push_back(idx);
            idx = prev[idx];
        }
        std::reverse(result.begin(), result.end());
        return result;
    }

} // namespace dijkstra
