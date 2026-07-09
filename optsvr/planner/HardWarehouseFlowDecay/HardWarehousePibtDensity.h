// optsvr/planner/HardWarehouseFlowDecay/HardWarehousePibtDensity.h
// Cell-level historical occupancy penalty for PIBT candidate ordering.
#pragma once

#include "SharedEnv.h"
#include <cstdint>
#include <vector>

namespace optsvr {

class HardWarehousePibtDensity {
public:
    ~HardWarehousePibtDensity();

    void initialize(SharedEnvironment* env);

    // Record one simulated/real tick of cell occupancy. The penalty is inactive
    // until the configured sliding window is fully populated.
    void record_occupancy(const std::vector<State>& agent_states, int num_agents);

    bool active() const;
    double penalty(int loc) const;
    double apply_penalty(int loc, double base_heuristic) const;
    double candidate_penalty(int loc, double base_heuristic, bool is_wait) const;
    void record_ordering(bool changed, bool base_first_wait, bool ranked_first_wait) const;

    int sort_mode() const { return sort_mode_; }
    double tie_base_eps() const { return tie_base_eps_; }

    int sample_count() const { return history_samples_; }
    int window() const { return window_; }

private:
    bool enabled_ = false;
    int map_size_ = 0;
    int window_ = 0;
    int cursor_ = 0;
    int history_samples_ = 0;

    float free_threshold_ = 0.0f;
    float quad_threshold_ = 0.0f;
    double linear_weight_ = 0.0;
    double quad_weight_ = 0.0;
    double cap_ = 0.0;
    double quad_power_ = 2.0;
    int sort_mode_ = 0;
    double tie_base_eps_ = 0.0;
    bool apply_to_wait_ = true;
    int goal_exempt_dist_ = -1;
    bool stats_enabled_ = false;
    int stats_interval_ = 0;

    std::vector<uint16_t> history_counts_;
    std::vector<std::vector<int>> history_ring_;

    std::uint64_t total_samples_ = 0;
    mutable std::uint64_t stats_candidates_ = 0;
    mutable std::uint64_t stats_positive_candidates_ = 0;
    mutable long double stats_base_sum_ = 0.0;
    mutable long double stats_penalty_sum_ = 0.0;
    mutable long double stats_positive_penalty_sum_ = 0.0;
    mutable long double stats_ratio_sum_ = 0.0;
    mutable long double stats_positive_ratio_sum_ = 0.0;
    mutable std::uint64_t stats_quad_candidates_ = 0;
    mutable std::uint64_t stats_order_samples_ = 0;
    mutable std::uint64_t stats_order_changed_ = 0;
    mutable std::uint64_t stats_base_first_wait_ = 0;
    mutable std::uint64_t stats_ranked_first_wait_ = 0;
    mutable double stats_max_penalty_ = 0.0;
    mutable std::uint64_t stats_last_print_sample_ = 0;
    mutable bool stats_final_printed_ = false;

    float occupancy_ratio(int loc) const;
    double penalty_from_ratio(float ratio) const;
    void print_stats(bool force) const;
};

} // namespace optsvr
