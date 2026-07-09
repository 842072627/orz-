#include "IdleParking.h"
#include "RuntimeConfig.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <numeric>
#include <random>

namespace nz {

IdleParking& idle_parking() {
    static IdleParking parking;
    return parking;
}

void IdleParking::reset() {
    initialized_ = false;
    rows_ = 0;
    cols_ = 0;
    n_agents_ = 0;
    ratio_ = 0.0f;
    parked_.clear();
    parked_goals_.clear();
    parked_agents_.clear();
}

bool IdleParking::is_parked(int agent_id) const {
    return agent_id >= 0 && agent_id < static_cast<int>(parked_.size()) &&
           parked_[agent_id] != 0;
}

std::vector<int> IdleParking::collect_outer_goals(const SharedEnvironment* env) const {
    std::vector<int> goals;
    if (env == nullptr || env->rows <= 0 || env->cols <= 0) return goals;

    const int rows = env->rows;
    const int cols = env->cols;
    const int map_size = rows * cols;
    if (static_cast<int>(env->map.size()) < map_size) return goals;

    int best_border_dist = std::numeric_limits<int>::max();
    for (int loc = 0; loc < map_size; ++loc) {
        if (env->map[loc] != 0) continue;
        const int r = loc / cols;
        const int c = loc % cols;
        const int border_dist = std::min(std::min(r, rows - 1 - r),
                                         std::min(c, cols - 1 - c));
        best_border_dist = std::min(best_border_dist, border_dist);
    }
    if (best_border_dist == std::numeric_limits<int>::max()) return goals;

    const int requested_ring = std::max(0, cfg().heap_idle_parking_ring);
    const int max_border_dist = std::max(best_border_dist, requested_ring);
    for (int loc = 0; loc < map_size; ++loc) {
        if (env->map[loc] != 0) continue;
        const int r = loc / cols;
        const int c = loc % cols;
        const int border_dist = std::min(std::min(r, rows - 1 - r),
                                         std::min(c, cols - 1 - c));
        if (border_dist <= max_border_dist) goals.push_back(loc);
    }
    return goals;
}

void IdleParking::initialize(SharedEnvironment* env) {
    reset();
    initialized_ = true;

    if (env == nullptr || cfg().schedule_mode != 1) return;

    rows_ = env->rows;
    cols_ = env->cols;
    n_agents_ = env->num_of_agents;
    ratio_ = std::max(0.0f, std::min(1.0f, cfg().heap_idle_parking_ratio));
    if (ratio_ <= 0.0f || n_agents_ <= 0) return;

    std::vector<int> outer_goals = collect_outer_goals(env);
    if (outer_goals.empty()) {
        if (cfg().debug_level >= 1) {
            std::cerr << "[IdleParking] no outer free goals found, disabled\n";
        }
        return;
    }

    int park_count = static_cast<int>(std::floor(ratio_ * static_cast<float>(n_agents_)));
    if (park_count <= 0) park_count = 1;
    park_count = std::min(park_count, n_agents_);

    std::mt19937 rng(static_cast<unsigned int>(cfg().heap_idle_parking_seed));
    std::vector<int> agents(n_agents_);
    std::iota(agents.begin(), agents.end(), 0);
    std::shuffle(agents.begin(), agents.end(), rng);
    std::shuffle(outer_goals.begin(), outer_goals.end(), rng);

    parked_.assign(n_agents_, 0);
    parked_goals_.assign(n_agents_, -1);
    parked_agents_.reserve(park_count);

    for (int i = 0; i < park_count; ++i) {
        const int agent = agents[i];
        const int goal = outer_goals[i % outer_goals.size()];
        parked_[agent] = 1;
        parked_goals_[agent] = goal;
        parked_agents_.push_back(agent);
    }

    if (cfg().debug_level >= 1) {
        std::cerr << "[IdleParking] parked=" << parked_agents_.size()
                  << "/" << n_agents_
                  << " ratio=" << ratio_
                  << " outer_goals=" << outer_goals.size()
                  << " seed=" << cfg().heap_idle_parking_seed
                  << " ring=" << cfg().heap_idle_parking_ring
                  << "\n";
    }
}

void IdleParking::apply_schedule_filter(std::vector<int>& schedule) const {
    if (!enabled()) return;
    for (int agent : parked_agents_) {
        if (agent >= 0 && agent < static_cast<int>(schedule.size())) {
            schedule[agent] = -1;
        }
    }
}

void IdleParking::apply_goal_locations(SharedEnvironment* env) const {
    if (!enabled() || env == nullptr) return;
    if (static_cast<int>(env->goal_locations.size()) < n_agents_) return;

    const int reveal_time = std::max(env->curr_timestep, env->system_timestep);
    for (int agent : parked_agents_) {
        if (agent < 0 || agent >= static_cast<int>(env->goal_locations.size())) continue;
        const int goal = parked_goals_[agent];
        if (goal < 0) continue;
        env->goal_locations[agent].clear();
        env->goal_locations[agent].push_back({goal, reveal_time});
    }
}

}  // namespace nz
