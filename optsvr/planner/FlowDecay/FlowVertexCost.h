#pragma once

#include "RuntimeConfig.h"
#include "Types.h"

#include <algorithm>
#include <functional>
#include <vector>

namespace OptPlan {

struct VertexCostBreakdown {
    int base = 0;
    int over = 0;
    long long extra = 0;
    double factor = 1.0;
    long long decayed_extra = 0;
    int cost = 0;
};

struct VertexCostParams {
    int mode = 0;
    int threshold = 0;
    double weight = 0.0;
    int cap = 1000000000;
    float exp_base = 2.0f;
    int linear_end = 0;
    float sens_decay = 1.0f;
    int base_weight = 100;
};

struct VertexFlowHotStats {
    int active = 0;
    int max = 0;
    double hot1_avg = 0.0;
    double hot10_avg = 0.0;
    double hot50_avg = 0.0;
};

inline VertexCostParams flow_vertex_cost_params_from_cfg() {
    const auto& c = optsvr::cfg();
    VertexCostParams p;
    p.mode = c.flow_vertex_cost_mode;
    p.threshold = c.flow_vertex_cost_threshold;
    p.weight = c.flow_vertex_cost_weight;
    p.cap = c.flow_vertex_cost_cap;
    p.exp_base = c.flow_vertex_cost_exp_base;
    p.linear_end = c.flow_vertex_cost_linear_end;
    p.sens_decay = c.flow_vertex_sens_decay;
    p.base_weight = c.flow_vertex_base_weight;
    if (c.planner_mode == 1 && !c.p1_flow_vertex_penalty_enabled) {
        p.weight = 0.0;
        p.base_weight = 0;
    }
    return p;
}

inline int unscale_flow(int raw) {
    int s = optsvr::cfg().flow_decay_scale;
    if (s <= 1) return raw;
    return (raw + s / 2) / s;
}

inline double vertex_sens_factor(int depth_offset,
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

inline double repeated_power(double base, int steps) {
    double out = 1.0;
    for (int i = 0; i < steps; ++i) out *= base;
    return out;
}

inline VertexCostBreakdown vertex_flow_cost(int temp_vertex, int depth_offset,
                                            std::vector<double>& decay_table,
                                            const VertexCostParams& params) {
    VertexCostBreakdown out;
    if (params.weight <= 0.0) return out;

    const int raw_base = std::max(0, (temp_vertex - 1) / 2);
    out.base =
        static_cast<int>(static_cast<long long>(raw_base) *
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
            const double shaped = repeated_power(exp_base, exp_steps);
            out.extra =
                static_cast<long long>((shaped - 1.0) * static_cast<double>(weight) / 100.0);
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
                repeated_power(exp_base, exp_steps) - repeated_power(exp_base, linear_end);
            out.extra =
                static_cast<long long>(shaped * static_cast<double>(weight) / 100.0);
        }
    } else if (params.mode == 4) {
        const int linear_end = std::min(std::max(0, params.linear_end), 20);
        const int tail = std::max(0, out.over - linear_end);
        const double quad_scale = std::max(0.0f, params.exp_base);
        const double shaped =
            static_cast<double>(std::min(out.over, linear_end)) +
            static_cast<double>(tail) +
            quad_scale * static_cast<double>(tail) * static_cast<double>(tail);
        out.extra =
            static_cast<long long>(shaped * static_cast<double>(weight) / 100.0);
    } else {
        out.extra = static_cast<long long>(out.over) * weight;
    }

    out.factor = vertex_sens_factor(depth_offset, decay_table, params.sens_decay);
    out.decayed_extra =
        static_cast<long long>(static_cast<double>(out.extra) * out.factor + 0.5);
    const long long cost = static_cast<long long>(out.base) + out.decayed_extra;
    out.cost = static_cast<int>(std::min<long long>(cost, cap));
    return out;
}

inline VertexCostBreakdown vertex_flow_cost(int temp_vertex, int depth_offset,
                                            std::vector<double>& decay_table) {
    return vertex_flow_cost(temp_vertex, depth_offset, decay_table,
                            flow_vertex_cost_params_from_cfg());
}

inline double vertex_flow_cost_double(int temp_vertex, int depth_offset,
                                      std::vector<double>& decay_table,
                                      const VertexCostParams& params) {
    if (params.weight <= 0.0) return 0.0;

    const int raw_base = std::max(0, (temp_vertex - 1) / 2);
    const double base =
        static_cast<double>(raw_base) * static_cast<double>(std::max(0, params.base_weight)) / 100.0;
    if (params.mode <= 0) return base;

    const int threshold = std::max(0, params.threshold);
    const int over = std::max(0, temp_vertex - threshold);
    if (over == 0) return base;

    const double weight = std::max(0.0, params.weight);
    double extra = 0.0;
    if (params.mode == 1) {
        extra = static_cast<double>(over) * static_cast<double>(over) * weight / 100.0;
    } else if (params.mode == 2) {
        const int exp_steps = std::min(over, 20);
        const double exp_base = std::max(1.0f, params.exp_base);
        extra = (repeated_power(exp_base, exp_steps) - 1.0) * weight / 100.0;
    } else if (params.mode == 3) {
        const int linear_end = std::min(std::max(0, params.linear_end), 20);
        if (over <= linear_end) {
            extra = static_cast<double>(over) * weight / 100.0;
        } else {
            const int exp_steps = std::min(over, 20);
            const double exp_base = std::max(1.0f, params.exp_base);
            const double shaped =
                static_cast<double>(linear_end) +
                repeated_power(exp_base, exp_steps) - repeated_power(exp_base, linear_end);
            extra = shaped * weight / 100.0;
        }
    } else if (params.mode == 4) {
        const int linear_end = std::min(std::max(0, params.linear_end), 20);
        const int tail = std::max(0, over - linear_end);
        const double quad_scale = std::max(0.0f, params.exp_base);
        const double shaped =
            static_cast<double>(std::min(over, linear_end)) +
            static_cast<double>(tail) +
            quad_scale * static_cast<double>(tail) * static_cast<double>(tail);
        extra = shaped * weight / 100.0;
    } else {
        extra = static_cast<double>(over) * weight;
    }

    const double factor = vertex_sens_factor(depth_offset, decay_table, params.sens_decay);
    const double cost = base + extra * factor;
    const double cap = (params.cap > 0) ? static_cast<double>(params.cap) : 1000000000.0;
    return std::min(cost, cap);
}

inline int vertex_flow_count_at(const std::vector<Int4>& flow, int loc) {
    if (loc < 0 || loc >= static_cast<int>(flow.size())) return 0;
    int temp_vertex = 1;
    for (int j = 0; j < 4; ++j) {
        temp_vertex += unscale_flow(flow[loc].d[j]);
    }
    return temp_vertex;
}

inline int vertex_flow_hot_count(int active, int percent) {
    if (active <= 0) return 0;
    const int count = (active * percent + 99) / 100;
    return std::min(active, std::max(1, count));
}

inline double vertex_flow_average_first(const std::vector<int>& values, int count) {
    if (count <= 0) return 0.0;
    long long sum = 0;
    for (int i = 0; i < count; ++i) sum += values[i];
    return static_cast<double>(sum) / static_cast<double>(count);
}

inline void vertex_flow_partition_top(std::vector<int>& values, int count, int range_end) {
    if (count <= 0 || count >= range_end) return;
    std::nth_element(values.begin(), values.begin() + count, values.begin() + range_end,
                     std::greater<int>());
}

inline VertexFlowHotStats compute_vertex_flow_hot_stats(const std::vector<Int4>& flow) {
    VertexFlowHotStats stats;
    std::vector<int> active_values;
    for (int loc = 0; loc < static_cast<int>(flow.size()); ++loc) {
        const int value = vertex_flow_count_at(flow, loc);
        if (value <= 1) continue;
        active_values.push_back(value);
        stats.max = std::max(stats.max, value);
    }

    stats.active = static_cast<int>(active_values.size());
    if (stats.active <= 0) return stats;

    const int hot50_count = vertex_flow_hot_count(stats.active, 50);
    const int hot10_count = vertex_flow_hot_count(stats.active, 10);
    const int hot1_count = vertex_flow_hot_count(stats.active, 1);

    vertex_flow_partition_top(active_values, hot50_count, stats.active);
    stats.hot50_avg = vertex_flow_average_first(active_values, hot50_count);

    vertex_flow_partition_top(active_values, hot10_count, hot50_count);
    stats.hot10_avg = vertex_flow_average_first(active_values, hot10_count);

    vertex_flow_partition_top(active_values, hot1_count, hot10_count);
    stats.hot1_avg = vertex_flow_average_first(active_values, hot1_count);

    return stats;
}

}  // namespace OptPlan
