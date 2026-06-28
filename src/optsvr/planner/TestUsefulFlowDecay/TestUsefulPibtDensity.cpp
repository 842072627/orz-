// optsvr/planner/TestUsefulFlowDecay/TestUsefulPibtDensity.cpp

#include "TestUsefulPibtDensity.h"
#include "RuntimeConfig.h"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>

namespace optsvr {

TestUsefulPibtDensity::~TestUsefulPibtDensity() {
    print_stats(true);
}

void TestUsefulPibtDensity::initialize(SharedEnvironment* env) {
    enabled_ = cfg().pibt_history_congestion_enabled;
    map_size_ = env ? static_cast<int>(env->map.size()) : 0;
    window_ = std::max(1, cfg().pibt_history_window);
    cursor_ = 0;
    history_samples_ = 0;
    total_samples_ = 0;

    free_threshold_ = std::clamp(cfg().pibt_history_free_threshold, 0.0f, 1.0f);
    quad_threshold_ = std::clamp(cfg().pibt_history_quad_threshold, 0.0f, 1.0f);
    if (quad_threshold_ <= free_threshold_) {
        quad_threshold_ = std::min(1.0f, free_threshold_ + 0.01f);
    }

    linear_weight_ = std::max(0.0f, cfg().pibt_history_linear_weight);
    quad_weight_ = std::max(0.0f, cfg().pibt_history_quad_weight);
    cap_ = std::max(0.0f, cfg().pibt_history_cap);
    quad_power_ = std::max(0.25f, cfg().pibt_history_quad_power);
    sort_mode_ = std::clamp(cfg().pibt_history_sort_mode, 0, 2);
    tie_base_eps_ = std::max(0.0f, cfg().pibt_history_tie_base_eps);
    apply_to_wait_ = cfg().pibt_history_apply_to_wait;
    goal_exempt_dist_ = cfg().pibt_history_goal_exempt_dist;
    stats_enabled_ = cfg().pibt_history_stats_enabled;
    stats_interval_ = std::max(0, cfg().pibt_history_stats_interval);

    history_counts_.assign(map_size_, 0);
    history_ring_.assign(window_, {});
    stats_candidates_ = 0;
    stats_positive_candidates_ = 0;
    stats_base_sum_ = 0.0;
    stats_penalty_sum_ = 0.0;
    stats_positive_penalty_sum_ = 0.0;
    stats_ratio_sum_ = 0.0;
    stats_positive_ratio_sum_ = 0.0;
    stats_quad_candidates_ = 0;
    stats_order_samples_ = 0;
    stats_order_changed_ = 0;
    stats_base_first_wait_ = 0;
    stats_ranked_first_wait_ = 0;
    stats_max_penalty_ = 0;
    stats_last_print_sample_ = 0;
    stats_final_printed_ = false;

    if (enabled_ && cfg().debug_level >= 1) {
        std::cerr << "[PibtHistoryCongestion] enabled window=" << window_
                  << " free=" << free_threshold_
                  << " quad=" << quad_threshold_
                  << " linear_w=" << linear_weight_
                  << " quad_w=" << quad_weight_
                  << " cap=" << cap_
                  << " quad_power=" << quad_power_
                  << " sort_mode=" << sort_mode_
                  << " tie_eps=" << tie_base_eps_
                  << " apply_wait=" << (apply_to_wait_ ? "on" : "off")
                  << " goal_exempt=" << goal_exempt_dist_
                  << " stats=" << (stats_enabled_ ? "on" : "off")
                  << " stats_interval=" << stats_interval_ << "\n";
    }
}

void TestUsefulPibtDensity::record_occupancy(
    const std::vector<State>& agent_states,
    int num_agents)
{
    if (!enabled_ || map_size_ <= 0 || window_ <= 0) return;

    auto& expired = history_ring_[cursor_];
    for (int loc : expired) {
        if (loc >= 0 && loc < map_size_ && history_counts_[loc] > 0) {
            history_counts_[loc]--;
        }
    }
    expired.clear();

    const int n = std::min(num_agents, static_cast<int>(agent_states.size()));
    expired.reserve(n);
    for (int i = 0; i < n; i++) {
        const int loc = agent_states[i].location;
        if (loc < 0 || loc >= map_size_) continue;
        if (history_counts_[loc] < std::numeric_limits<uint16_t>::max()) {
            history_counts_[loc]++;
        }
        expired.push_back(loc);
    }

    cursor_ = (cursor_ + 1) % window_;
    if (history_samples_ < window_) {
        history_samples_++;
    }
    total_samples_++;
    print_stats(false);
}

bool TestUsefulPibtDensity::active() const {
    return enabled_ && map_size_ > 0 && window_ > 0 && history_samples_ >= window_;
}

float TestUsefulPibtDensity::occupancy_ratio(int loc) const {
    if (!active() || loc < 0 || loc >= map_size_) return 0.0f;
    return std::min(1.0f, static_cast<float>(history_counts_[loc]) / static_cast<float>(window_));
}

double TestUsefulPibtDensity::penalty(int loc) const {
    const float ratio = occupancy_ratio(loc);
    return penalty_from_ratio(ratio);
}

double TestUsefulPibtDensity::penalty_from_ratio(float ratio) const {
    if (ratio < free_threshold_) return 0.0;

    double raw = 0.0;
    if (ratio < quad_threshold_) {
        const float denom = std::max(0.0001f, quad_threshold_ - free_threshold_);
        const float x = (ratio - free_threshold_) / denom;
        raw = linear_weight_ * static_cast<double>(x);
    } else {
        const float denom = std::max(0.0001f, 1.0f - quad_threshold_);
        const float x = (ratio - quad_threshold_) / denom;
        raw = linear_weight_ + quad_weight_ * std::pow(static_cast<double>(x), quad_power_);
    }

    if (cap_ > 0.0) raw = std::min(raw, cap_);
    return std::max(0.0, raw);
}

double TestUsefulPibtDensity::apply_penalty(int loc, double base_heuristic) const {
    return base_heuristic + candidate_penalty(loc, base_heuristic, false);
}

double TestUsefulPibtDensity::candidate_penalty(int loc, double base_heuristic, bool is_wait) const {
    const float ratio = occupancy_ratio(loc);
    double value = 0.0;
    if ((!is_wait || apply_to_wait_) &&
        (goal_exempt_dist_ < 0 || base_heuristic > static_cast<double>(goal_exempt_dist_))) {
        value = penalty_from_ratio(ratio);
    }
    if (stats_enabled_ && active()) {
        stats_candidates_++;
        stats_base_sum_ += static_cast<long double>(std::max(0.0, base_heuristic));
        stats_penalty_sum_ += static_cast<long double>(value);
        stats_ratio_sum_ += static_cast<long double>(ratio);
        if (ratio >= quad_threshold_) {
            stats_quad_candidates_++;
        }
        if (value > 0.0) {
            stats_positive_candidates_++;
            stats_positive_penalty_sum_ += static_cast<long double>(value);
            stats_positive_ratio_sum_ += static_cast<long double>(ratio);
            stats_max_penalty_ = std::max(stats_max_penalty_, value);
        }
    }
    return value;
}

void TestUsefulPibtDensity::record_ordering(
    bool changed,
    bool base_first_wait,
    bool ranked_first_wait) const
{
    if (!stats_enabled_ || !active()) return;
    stats_order_samples_++;
    if (changed) stats_order_changed_++;
    if (base_first_wait) stats_base_first_wait_++;
    if (ranked_first_wait) stats_ranked_first_wait_++;
}

void TestUsefulPibtDensity::print_stats(bool force) const {
    if (!enabled_ || !stats_enabled_ || stats_candidates_ == 0) return;
    if (force && stats_final_printed_) return;

    if (!force && stats_interval_ > 0 &&
        total_samples_ - stats_last_print_sample_ < static_cast<std::uint64_t>(stats_interval_)) {
        return;
    }
    if (!force && stats_interval_ == 0) return;

    stats_last_print_sample_ = total_samples_;
    if (force) stats_final_printed_ = true;

    const long double candidates = static_cast<long double>(stats_candidates_);
    const long double positive = static_cast<long double>(stats_positive_candidates_);
    const long double positive_ratio = candidates > 0.0 ? positive / candidates : 0.0;
    const long double avg_base = candidates > 0.0 ? stats_base_sum_ / candidates : 0.0;
    const long double avg_penalty = candidates > 0.0 ? stats_penalty_sum_ / candidates : 0.0;
    const long double avg_positive_penalty =
        positive > 0.0 ? stats_positive_penalty_sum_ / positive : 0.0;
    const long double avg_ratio = candidates > 0.0 ? stats_ratio_sum_ / candidates : 0.0;
    const long double avg_positive_ratio =
        positive > 0.0 ? stats_positive_ratio_sum_ / positive : 0.0;
    const long double quad_ratio =
        candidates > 0.0 ? static_cast<long double>(stats_quad_candidates_) / candidates : 0.0;
    const long double penalty_base_ratio =
        stats_base_sum_ > 0.0 ? stats_penalty_sum_ / stats_base_sum_ : 0.0;
    const long double order_samples = static_cast<long double>(stats_order_samples_);
    const long double order_change_ratio =
        order_samples > 0.0 ? static_cast<long double>(stats_order_changed_) / order_samples : 0.0;
    const long double base_wait_ratio =
        order_samples > 0.0 ? static_cast<long double>(stats_base_first_wait_) / order_samples : 0.0;
    const long double ranked_wait_ratio =
        order_samples > 0.0 ? static_cast<long double>(stats_ranked_first_wait_) / order_samples : 0.0;

    std::ostringstream oss;
    oss << std::fixed << std::setprecision(4)
        << "[PibtHistoryCongestionStats]"
        << (force ? " final" : "")
        << " samples=" << total_samples_
        << " window=" << window_
        << " candidates=" << stats_candidates_
        << " positive=" << stats_positive_candidates_
        << " positive_ratio=" << static_cast<double>(positive_ratio)
        << " avg_base_h=" << static_cast<double>(avg_base)
        << " avg_penalty=" << static_cast<double>(avg_penalty)
        << " avg_positive_penalty=" << static_cast<double>(avg_positive_penalty)
        << " penalty_base_ratio=" << static_cast<double>(penalty_base_ratio)
        << " avg_occ_ratio=" << static_cast<double>(avg_ratio)
        << " avg_positive_occ_ratio=" << static_cast<double>(avg_positive_ratio)
        << " quad_ratio=" << static_cast<double>(quad_ratio)
        << " order_change_ratio=" << static_cast<double>(order_change_ratio)
        << " base_wait_ratio=" << static_cast<double>(base_wait_ratio)
        << " ranked_wait_ratio=" << static_cast<double>(ranked_wait_ratio)
        << " max_penalty=" << stats_max_penalty_;
    std::cerr << oss.str() << "\n";
}

} // namespace optsvr
