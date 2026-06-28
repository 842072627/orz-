#pragma once

#include "RuntimeConfig.h"
#include "Types.h"

#include <algorithm>
#include <vector>

namespace MazePlan {

struct MazeVertexCostBreakdown {
    int base = 0;
    int over = 0;
    long long extra = 0;
    double factor = 1.0;
    long long decayed_extra = 0;
    int cost = 0;
};

struct MazeVertexCostParams {
    int mode = 0;
    int threshold = 0;
    double weight = 0.0;
    int cap = 1000000000;
    float exp_base = 2.0f;
    int linear_end = 0;
    float sens_decay = 1.0f;
    int base_weight = 100;
};

inline MazeVertexCostParams maze_flow_vertex_cost_params_from_cfg() {
    const auto& c = optsvr::cfg();
    MazeVertexCostParams p;
    p.mode = c.flow_vertex_cost_mode;
    p.threshold = c.flow_vertex_cost_threshold;
    p.weight = c.flow_vertex_cost_weight;
    p.cap = c.flow_vertex_cost_cap;
    p.exp_base = c.flow_vertex_cost_exp_base;
    p.linear_end = c.flow_vertex_cost_linear_end;
    p.sens_decay = c.flow_vertex_sens_decay;
    p.base_weight = c.flow_vertex_base_weight;
    return p;
}

inline int maze_unscale_flow(int raw) {
    const int s = optsvr::cfg().flow_decay_scale;
    if (s <= 1) return raw;
    return (raw + s / 2) / s;
}

inline int maze_original_vertex_flow_cost(int temp_vertex) {
    return std::max(0, (temp_vertex - 1) / 2);
}

inline double maze_vertex_sens_factor(int depth_offset,
                                      std::vector<double>& decay_table,
                                      float decay_value) {
    if (depth_offset <= 0) return 1.0;

    const double decay = decay_value;
    if (decay < 0.0 || decay >= 1.0) return 1.0;

    while (static_cast<int>(decay_table.size()) <= depth_offset) {
        decay_table.push_back(decay_table.back() * decay);
    }
    return decay_table[depth_offset];
}

inline double maze_repeated_power(double base, int steps) {
    double out = 1.0;
    for (int i = 0; i < steps; ++i) out *= base;
    return out;
}

inline MazeVertexCostBreakdown maze_p1_vertex_flow_cost(
    int temp_vertex,
    int depth_offset,
    std::vector<double>& decay_table,
    const MazeVertexCostParams& params) {
    MazeVertexCostBreakdown out;
    if (params.weight <= 0.0) return out;

    const int raw_base = maze_original_vertex_flow_cost(temp_vertex);
    out.base = static_cast<int>(static_cast<long long>(raw_base) *
                                std::max(0, params.base_weight) / 100);
    out.cost = out.base;
    if (params.mode <= 0) return out;

    const int threshold = std::max(0, params.threshold);
    out.over = std::max(0, temp_vertex - threshold);
    if (out.over == 0) return out;

    const long long weight = static_cast<long long>(std::max(0.0, params.weight));
    const int cap = (params.cap > 0) ? params.cap : 1000000000;
    if (params.mode == 1) {
        out.extra = static_cast<long long>(out.over) * out.over * weight / 100;
    } else if (params.mode == 2) {
        const int exp_steps = std::min(out.over, 20);
        if (params.exp_base == 2.0f) {
            out.extra = (((1LL << exp_steps) - 1LL) * weight) / 100;
        } else {
            const double exp_base = std::max(1.0f, params.exp_base);
            const double shaped = maze_repeated_power(exp_base, exp_steps);
            out.extra = static_cast<long long>((shaped - 1.0) *
                                               static_cast<double>(weight) / 100.0);
        }
    } else if (params.mode == 3) {
        const int linear_end = std::min(std::max(0, params.linear_end), 20);
        if (out.over <= linear_end) {
            out.extra = static_cast<long long>(out.over) * weight / 100;
        } else {
            const int exp_steps = std::min(out.over, 20);
            const double exp_base = std::max(1.0f, params.exp_base);
            const double shaped =
                static_cast<double>(linear_end) +
                maze_repeated_power(exp_base, exp_steps) -
                maze_repeated_power(exp_base, linear_end);
            out.extra = static_cast<long long>(shaped *
                                               static_cast<double>(weight) / 100.0);
        }
    } else if (params.mode == 4) {
        const int linear_end = std::min(std::max(0, params.linear_end), 20);
        const int tail = std::max(0, out.over - linear_end);
        const double quad_scale = std::max(0.0f, params.exp_base);
        const double shaped =
            static_cast<double>(std::min(out.over, linear_end)) +
            static_cast<double>(tail) +
            quad_scale * static_cast<double>(tail) * static_cast<double>(tail);
        out.extra = static_cast<long long>(shaped *
                                           static_cast<double>(weight) / 100.0);
    } else {
        out.extra = static_cast<long long>(out.over) * weight;
    }

    out.factor = maze_vertex_sens_factor(depth_offset, decay_table, params.sens_decay);
    out.decayed_extra =
        static_cast<long long>(static_cast<double>(out.extra) * out.factor + 0.5);
    const long long cost = static_cast<long long>(out.base) + out.decayed_extra;
    out.cost = static_cast<int>(std::min<long long>(cost, cap));
    return out;
}

inline int maze_vertex_flow_count_at(const std::vector<Int4>& flow, int loc) {
    if (loc < 0 || loc >= static_cast<int>(flow.size())) return 0;
    int temp_vertex = 1;
    for (int j = 0; j < 4; ++j) {
        temp_vertex += maze_unscale_flow(flow[loc].d[j]);
    }
    return temp_vertex;
}

inline int maze_vertex_flow_cost_for_search(int temp_vertex,
                                            int depth_offset,
                                            std::vector<double>& decay_table) {
    if (!optsvr::cfg().p1_flow_vertex_penalty_enabled) {
        return maze_original_vertex_flow_cost(temp_vertex);
    }
    return maze_p1_vertex_flow_cost(temp_vertex,
                                    depth_offset,
                                    decay_table,
                                    maze_flow_vertex_cost_params_from_cfg()).cost;
}

} // namespace MazePlan
