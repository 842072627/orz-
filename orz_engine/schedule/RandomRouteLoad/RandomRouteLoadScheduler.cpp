// optsvr/schedule/RandomRouteLoad/RandomRouteLoadScheduler.cpp
#include "RandomRouteLoadScheduler.h"
#include "HeuristicMatrix.h"

#include <algorithm>
#include <cmath>
#include <functional>
#include <iostream>
#include <limits>
#include <queue>
#include <unordered_map>
#include <unordered_set>

namespace orz_engine {

namespace {
constexpr int H_CAP = 100000;
}

void RandomRouteLoadScheduler::initialize(SharedEnvironment* env,
                                          int /*preprocess_time_limit_ms*/) {
    env_ = env;
    if (!env_) return;

    rows_ = env_->rows;
    cols_ = env_->cols;
    map_size_ = rows_ * cols_;

    build_route_grid();

    heap_matcher_.set_env(env_);
    heap_matcher_.set_congestion_func(nullptr);

    if (cfg().debug_level >= 1) {
        std::cerr << "[RandomRouteLoad] initialized: zones="
                  << route_grid_rows_ << "x" << route_grid_cols_
                  << " cell=" << route_cell_size_
                  << " weight=" << cfg().random_route_load_weight
                  << " batch=" << cfg().random_route_load_batch_size
                  << " decay=" << cfg().random_route_load_decay
                  << " len_write=" << cfg().random_route_load_length_weighted_write
                  << " len_scale=" << cfg().random_route_load_write_length_scale
                  << " dyn_sp=" << cfg().random_route_load_dynamic_shortest
                  << " dyn_w=" << cfg().random_route_load_dynamic_path_weight
                  << " cap_norm=" << cfg().random_route_load_capacity_norm
                  << " min_valid=" << cfg().random_route_load_min_valid_ratio
                  << "\n";
    }
}

bool RandomRouteLoadScheduler::is_free_loc(int loc) const {
    return env_ && loc >= 0 && loc < static_cast<int>(env_->map.size()) &&
           env_->map[loc] == 0;
}

int RandomRouteLoadScheduler::zone_of_loc(int loc) const {
    if (loc < 0 || loc >= map_size_ || route_cell_size_ <= 0 ||
        route_grid_cols_ <= 0) {
        return -1;
    }
    int r = loc / cols_;
    int c = loc % cols_;
    int zr = std::min(route_grid_rows_ - 1, r / route_cell_size_);
    int zc = std::min(route_grid_cols_ - 1, c / route_cell_size_);
    if (zr < 0 || zc < 0) return -1;
    return zr * route_grid_cols_ + zc;
}

int RandomRouteLoadScheduler::dist(int source, int target) const {
    if (!env_ || source < 0 || target < 0 ||
        source >= map_size_ || target >= map_size_) {
        return H_CAP;
    }
    int d = get_h_fast(env_, source, target);
    if (d < 0 || d >= H_CAP) return H_CAP;
    return d;
}

void RandomRouteLoadScheduler::build_route_grid() {
    route_cell_size_ = cfg().random_route_load_grid_cell;
    if (route_cell_size_ <= 0) route_cell_size_ = 4;

    route_grid_rows_ = (rows_ + route_cell_size_ - 1) / route_cell_size_;
    route_grid_cols_ = (cols_ + route_cell_size_ - 1) / route_cell_size_;
    route_grid_count_ = route_grid_rows_ * route_grid_cols_;
    route_zone_rep_locs_.assign(route_grid_count_, -1);
    route_zone_free_cells_.assign(route_grid_count_, 0);
    route_zone_total_cells_.assign(route_grid_count_, 0);
    route_zone_valid_ratio_.assign(route_grid_count_, 0.0f);
    route_zone_edges_.clear();
    future_load_.assign(route_grid_count_, 0.0f);
    dynamic_zone_pair_load_cache_.clear();
    dynamic_zone_pair_len_cache_.clear();
    dynamic_zone_pair_next_.clear();
    dynamic_zone_pair_cache_valid_ = false;

    if (!env_ || route_grid_count_ <= 0) return;

    std::vector<int> first_free(route_grid_count_, -1);
    for (int loc = 0; loc < map_size_; loc++) {
        int z = zone_of_loc(loc);
        if (z < 0 || z >= route_grid_count_) continue;
        route_zone_total_cells_[z]++;
        if (!is_free_loc(loc)) continue;
        route_zone_free_cells_[z]++;
        if (first_free[z] < 0) first_free[z] = loc;
    }

    for (int z = 0; z < route_grid_count_; z++) {
        if (route_zone_total_cells_[z] > 0) {
            route_zone_valid_ratio_[z] =
                static_cast<float>(route_zone_free_cells_[z]) /
                static_cast<float>(route_zone_total_cells_[z]);
        }
    }

    for (int zr = 0; zr < route_grid_rows_; zr++) {
        for (int zc = 0; zc < route_grid_cols_; zc++) {
            int z = zr * route_grid_cols_ + zc;
            int r0 = zr * route_cell_size_;
            int c0 = zc * route_cell_size_;
            int r1 = std::min(rows_ - 1, r0 + route_cell_size_ - 1);
            int c1 = std::min(cols_ - 1, c0 + route_cell_size_ - 1);
            int center = ((r0 + r1) / 2) * cols_ + ((c0 + c1) / 2);
            route_zone_rep_locs_[z] = is_free_loc(center) ? center : first_free[z];
        }
    }

    build_route_zone_graph();
    build_zone_pair_corridors();
    zone_pair_load_cache_.assign(
        static_cast<size_t>(route_grid_count_) * route_grid_count_, 0.0f);
}

float RandomRouteLoadScheduler::zone_capacity_adjusted_load(int zone) const {
    if (zone < 0 || zone >= route_grid_count_ ||
        zone >= static_cast<int>(future_load_.size())) {
        return 0.0f;
    }

    float load = future_load_[zone];
    if (!cfg().random_route_load_capacity_norm) return load;

    float valid_ratio = 1.0f;
    if (zone < static_cast<int>(route_zone_valid_ratio_.size())) {
        valid_ratio = route_zone_valid_ratio_[zone];
    }
    float min_ratio = std::max(1e-3f, cfg().random_route_load_min_valid_ratio);
    valid_ratio = std::max(valid_ratio, min_ratio);
    return load / valid_ratio;
}

void RandomRouteLoadScheduler::build_route_zone_graph() {
    route_zone_edges_.assign(route_grid_count_, {});
    if (!env_ || route_grid_count_ <= 0 || route_grid_cols_ <= 0) return;

    const int dr[4] = {-1, 1, 0, 0};
    const int dc[4] = {0, 0, -1, 1};
    for (int z = 0; z < route_grid_count_; z++) {
        int from = route_zone_rep_locs_[z];
        if (from < 0) continue;
        int zr = z / route_grid_cols_;
        int zc = z % route_grid_cols_;
        for (int k = 0; k < 4; k++) {
            int nr = zr + dr[k];
            int nc = zc + dc[k];
            if (nr < 0 || nr >= route_grid_rows_ ||
                nc < 0 || nc >= route_grid_cols_) {
                continue;
            }
            int nz = nr * route_grid_cols_ + nc;
            int to = route_zone_rep_locs_[nz];
            if (to < 0) continue;
            int d = dist(from, to);
            if (d >= H_CAP) continue;
            route_zone_edges_[z].push_back(
                RouteZoneEdge{nz, static_cast<float>(std::max(1, d))});
        }
    }
}

void RandomRouteLoadScheduler::build_zone_pair_corridors() {
    zone_pair_corridor_weights_.clear();
    if (route_grid_count_ <= 0) return;

    zone_pair_corridor_weights_.resize(
        static_cast<size_t>(route_grid_count_) * route_grid_count_);

    int tight = std::max(0, cfg().random_route_load_tight_slack);
    int loose = std::max(tight, cfg().random_route_load_loose_slack);

    for (int za = 0; za < route_grid_count_; za++) {
        int from = route_zone_rep_locs_[za];
        if (from < 0) continue;
        for (int zb = 0; zb < route_grid_count_; zb++) {
            int to = route_zone_rep_locs_[zb];
            if (to < 0) continue;

            auto& weights =
                zone_pair_corridor_weights_[static_cast<size_t>(za) *
                                            route_grid_count_ + zb];
            if (za == zb || from == to) {
                weights.push_back({za, 1.0f});
                continue;
            }

            int base = dist(from, to);
            if (base >= H_CAP) continue;

            float total_w = 0.0f;
            for (int z = 0; z < route_grid_count_; z++) {
                int rep = route_zone_rep_locs_[z];
                if (rep < 0) continue;
                int d1 = dist(from, rep);
                int d2 = dist(rep, to);
                if (d1 >= H_CAP || d2 >= H_CAP) continue;
                int extra = d1 + d2 - base;
                if (extra < 0) extra = 0;

                float w = 0.0f;
                if (extra <= tight) {
                    w = 1.0f;
                } else if (extra <= loose && loose > tight) {
                    w = 1.0f - static_cast<float>(extra - tight) /
                                 static_cast<float>(loose - tight);
                }
                if (w > 0.0f) {
                    weights.push_back({z, w});
                    total_w += w;
                }
            }

            if (total_w <= 0.0f) {
                weights.clear();
                weights.push_back({za, 0.5f});
                weights.push_back({zb, 0.5f});
                continue;
            }

            for (auto& zw : weights) {
                zw.second /= total_w;
            }
        }
    }
}

void RandomRouteLoadScheduler::refresh_zone_pair_load_cache() {
    if (route_grid_count_ <= 0) return;
    const size_t n_pairs = static_cast<size_t>(route_grid_count_) * route_grid_count_;
    if (zone_pair_load_cache_.size() != n_pairs) {
        zone_pair_load_cache_.assign(n_pairs, 0.0f);
    }
    if (zone_pair_corridor_weights_.size() != n_pairs) {
        build_zone_pair_corridors();
    }

    for (size_t i = 0; i < n_pairs; i++) {
        float load = 0.0f;
        for (const auto& zw : zone_pair_corridor_weights_[i]) {
            int z = zw.first;
            if (z >= 0 && z < route_grid_count_) {
                load += zw.second * zone_capacity_adjusted_load(z);
            }
        }
        zone_pair_load_cache_[i] = load;
    }
}

void RandomRouteLoadScheduler::refresh_dynamic_zone_pair_cache(float route_weight) {
    dynamic_zone_pair_cache_valid_ = false;
    if (!cfg().random_route_load_dynamic_shortest ||
        route_grid_count_ <= 0 ||
        route_zone_edges_.empty()) {
        return;
    }

    const int n = route_grid_count_;
    const size_t n_pairs = static_cast<size_t>(n) * n;
    if (zone_pair_load_cache_.size() != n_pairs) {
        refresh_zone_pair_load_cache();
    }

    dynamic_zone_pair_load_cache_.assign(n_pairs, 0.0f);
    dynamic_zone_pair_len_cache_.assign(n_pairs, 0.0f);
    dynamic_zone_pair_next_.assign(n_pairs, -1);

    const float inf = std::numeric_limits<float>::infinity();
    const float path_weight =
        std::max(0.0f, route_weight * cfg().random_route_load_dynamic_path_weight);

    struct Node {
        float cost;
        int zone;
        bool operator>(const Node& other) const {
            return cost > other.cost;
        }
    };

    std::vector<float> best_cost(n);
    std::vector<float> best_len(n);
    std::vector<float> best_load_sum(n);
    std::vector<int> first_hop(n);

    for (int src = 0; src < n; src++) {
        if (route_zone_rep_locs_[src] < 0) continue;

        std::fill(best_cost.begin(), best_cost.end(), inf);
        std::fill(best_len.begin(), best_len.end(), inf);
        std::fill(best_load_sum.begin(), best_load_sum.end(), 0.0f);
        std::fill(first_hop.begin(), first_hop.end(), -1);

        std::priority_queue<Node, std::vector<Node>, std::greater<Node>> pq;
        best_cost[src] = 0.0f;
        best_len[src] = 0.0f;
        first_hop[src] = src;
        pq.push(Node{0.0f, src});

        while (!pq.empty()) {
            Node cur = pq.top();
            pq.pop();
            int z = cur.zone;
            if (cur.cost > best_cost[z] + 1e-5f) continue;

            for (const RouteZoneEdge& e : route_zone_edges_[z]) {
                if (e.to < 0 || e.to >= n || e.length <= 0.0f) continue;
                size_t edge_idx = static_cast<size_t>(z) * n + e.to;
                float edge_load = 0.0f;
                if (edge_idx < zone_pair_load_cache_.size()) {
                    edge_load = std::max(0.0f, zone_pair_load_cache_[edge_idx]);
                }
                float edge_cost = e.length * (1.0f + path_weight * edge_load);
                float next_cost = best_cost[z] + edge_cost;
                float next_len = best_len[z] + e.length;
                bool better = next_cost + 1e-5f < best_cost[e.to];
                if (!better && std::fabs(next_cost - best_cost[e.to]) <= 1e-5f) {
                    better = next_len < best_len[e.to];
                }
                if (!better) continue;

                best_cost[e.to] = next_cost;
                best_len[e.to] = next_len;
                best_load_sum[e.to] =
                    best_load_sum[z] + edge_load * e.length;
                first_hop[e.to] = (z == src) ? e.to : first_hop[z];
                pq.push(Node{next_cost, e.to});
            }
        }

        for (int dst = 0; dst < n; dst++) {
            size_t idx = static_cast<size_t>(src) * n + dst;
            if (dst == src) {
                dynamic_zone_pair_load_cache_[idx] =
                    zone_capacity_adjusted_load(src);
                dynamic_zone_pair_len_cache_[idx] = 0.0f;
                dynamic_zone_pair_next_[idx] = src;
            } else if (std::isfinite(best_cost[dst]) && best_len[dst] > 0.0f) {
                dynamic_zone_pair_load_cache_[idx] =
                    best_load_sum[dst] / best_len[dst];
                dynamic_zone_pair_len_cache_[idx] = best_len[dst];
                dynamic_zone_pair_next_[idx] = first_hop[dst];
            }
        }
    }

    dynamic_zone_pair_cache_valid_ = true;
}

AgentInfo RandomRouteLoadScheduler::make_agent_info_from_snapshot(int agent_id) const {
    int loc = -1;
    if (agent_id >= 0 && agent_id < static_cast<int>(curr_locations_snapshot_.size())) {
        loc = curr_locations_snapshot_[agent_id];
    }
    return AgentInfo{agent_id, loc};
}

TaskInfo RandomRouteLoadScheduler::make_task_info_from_snapshot(int task_id) const {
    TaskInfo info;
    info.id = task_id;
    info.start_location = -1;

    auto it = task_pool_snapshot_.find(task_id);
    if (it == task_pool_snapshot_.end()) return info;

    const Task& task = it->second;
    if (task.locations.empty()) return info;

    int start_idx = task.idx_next_loc;
    if (start_idx < 0) start_idx = 0;
    if (start_idx >= static_cast<int>(task.locations.size())) return info;

    info.start_location = task.locations[start_idx];
    info.t_revealed = task.t_revealed;
    info.errand_locations.reserve(task.locations.size() - start_idx);
    for (size_t i = static_cast<size_t>(start_idx); i < task.locations.size(); i++) {
        info.errand_locations.push_back(task.locations[i]);
    }
    return info;
}

void RandomRouteLoadScheduler::reset_future_load(const std::vector<int>& local_schedule) {
    std::fill(future_load_.begin(), future_load_.end(), 0.0f);

    for (int a = 0; a < static_cast<int>(local_schedule.size()); a++) {
        int tid = local_schedule[a];
        if (tid == -1) continue;
        add_agent_task_route_to_load(a, tid, 1.0f);
    }
}

void RandomRouteLoadScheduler::add_agent_task_route_to_load(int agent_id,
                                                            int task_id,
                                                            float amount) {
    AgentInfo agent = make_agent_info_from_snapshot(agent_id);
    TaskInfo task = make_task_info_from_snapshot(task_id);
    add_assigned_route_to_load(agent, task, amount);
}

void RandomRouteLoadScheduler::add_assigned_route_to_load(const AgentInfo& agent,
                                                          const TaskInfo& task,
                                                          float amount) {
    if (amount <= 0.0f || task.start_location < 0) return;

    float cumulative = 0.0f;
    if (agent.location >= 0 && agent.location < map_size_ &&
        agent.location != task.start_location) {
        int d = std::max(1, dist(agent.location, task.start_location));
        float step_weight = segment_future_load_amount(
            amount, d, 0.5f * static_cast<float>(d));
        add_segment_corridor_load(agent.location, task.start_location, step_weight);
        cumulative += static_cast<float>(d);
    }
    add_task_internal_route_to_load(task, amount, cumulative);
}

void RandomRouteLoadScheduler::add_task_internal_route_to_load(const TaskInfo& task,
                                                               float amount,
                                                               float initial_cumulative) {
    if (amount <= 0.0f || task.errand_locations.size() < 2) {
        return;
    }

    float cumulative = std::max(0.0f, initial_cumulative);
    for (size_t i = 1; i < task.errand_locations.size(); i++) {
        int from = task.errand_locations[i - 1];
        int to = task.errand_locations[i];
        int d = std::max(1, dist(from, to));
        float mid_dist = cumulative + 0.5f * static_cast<float>(d);
        float step_weight = segment_future_load_amount(amount, d, mid_dist);
        add_segment_corridor_load(from, to, step_weight);
        cumulative += static_cast<float>(d);
    }
}

float RandomRouteLoadScheduler::route_load_decay_factor(float cumulative_dist) const {
    const float decay = std::max(0.0f, cfg().random_route_load_decay);
    if (decay <= 0.0f) return 1.0f;
    return 1.0f / (1.0f + std::max(0.0f, cumulative_dist) * decay);
}

float RandomRouteLoadScheduler::segment_future_load_amount(
    float amount,
    int segment_dist,
    float midpoint_dist) const {
    if (amount <= 0.0f) return 0.0f;

    float write = amount * route_load_decay_factor(midpoint_dist);
    if (cfg().random_route_load_length_weighted_write) {
        const float scale = std::max(0.0f, cfg().random_route_load_write_length_scale);
        write *= static_cast<float>(std::max(1, segment_dist)) * scale;
    }
    return write;
}

void RandomRouteLoadScheduler::add_segment_corridor_load(int from,
                                                         int to,
                                                         float amount) {
    if (amount <= 0.0f || route_grid_count_ <= 0 || from < 0 || to < 0) {
        return;
    }

    int z_from = zone_of_loc(from);
    int z_to = zone_of_loc(to);
    if (z_from < 0 || z_to < 0 ||
        z_from >= route_grid_count_ || z_to >= route_grid_count_) {
        return;
    }

    if (cfg().random_route_load_dynamic_shortest &&
        add_zone_pair_dynamic_corridor_load(z_from, z_to, amount)) {
        return;
    }
    add_zone_pair_static_corridor_load(z_from, z_to, amount);
}

void RandomRouteLoadScheduler::add_zone_pair_static_corridor_load(
    int z_from,
    int z_to,
    float amount) {
    if (amount <= 0.0f || route_grid_count_ <= 0 ||
        z_from < 0 || z_to < 0 ||
        z_from >= route_grid_count_ || z_to >= route_grid_count_) {
        return;
    }

    const size_t n_pairs = static_cast<size_t>(route_grid_count_) * route_grid_count_;
    if (zone_pair_corridor_weights_.size() != n_pairs) {
        build_zone_pair_corridors();
    }

    size_t idx = static_cast<size_t>(z_from) * route_grid_count_ + z_to;
    if (idx >= zone_pair_corridor_weights_.size() ||
        zone_pair_corridor_weights_[idx].empty()) {
        if (z_from >= 0) future_load_[z_from] += amount * 0.5f;
        if (z_to >= 0) future_load_[z_to] += amount * 0.5f;
        return;
    }

    for (const auto& [z, w] : zone_pair_corridor_weights_[idx]) {
        if (z >= 0 && z < static_cast<int>(future_load_.size())) {
            future_load_[z] += amount * w;
        }
    }
}

bool RandomRouteLoadScheduler::add_zone_pair_dynamic_corridor_load(
    int z_from,
    int z_to,
    float amount) {
    if (!dynamic_zone_pair_cache_valid_ ||
        amount <= 0.0f ||
        route_grid_count_ <= 0 ||
        z_from < 0 || z_to < 0 ||
        z_from >= route_grid_count_ || z_to >= route_grid_count_) {
        return false;
    }

    if (z_from == z_to) {
        add_zone_pair_static_corridor_load(z_from, z_to, amount);
        return true;
    }

    const int n = route_grid_count_;
    size_t idx = static_cast<size_t>(z_from) * n + z_to;
    if (idx >= dynamic_zone_pair_next_.size() ||
        idx >= dynamic_zone_pair_len_cache_.size()) {
        return false;
    }

    float total_len = dynamic_zone_pair_len_cache_[idx];
    if (total_len <= 0.0f || !std::isfinite(total_len)) {
        return false;
    }

    int cur = z_from;
    std::vector<std::pair<int, float>> path_edges;
    path_edges.reserve(n);
    for (int guard = 0; guard < n && cur != z_to; guard++) {
        size_t next_idx = static_cast<size_t>(cur) * n + z_to;
        if (next_idx >= dynamic_zone_pair_next_.size()) return false;
        int next = dynamic_zone_pair_next_[next_idx];
        if (next < 0 || next >= n || next == cur) return false;

        float edge_len = 0.0f;
        for (const RouteZoneEdge& e : route_zone_edges_[cur]) {
            if (e.to == next) {
                edge_len = e.length;
                break;
            }
        }
        if (edge_len <= 0.0f) {
            int from_loc = route_zone_rep_locs_[cur];
            int to_loc = route_zone_rep_locs_[next];
            if (from_loc < 0 || to_loc < 0) return false;
            int d = dist(from_loc, to_loc);
            if (d >= H_CAP) return false;
            edge_len = static_cast<float>(std::max(1, d));
        }

        path_edges.push_back({next, edge_len});
        cur = next;
    }

    if (cur != z_to || path_edges.empty()) return false;

    cur = z_from;
    for (const auto& [next, edge_len] : path_edges) {
        float edge_amount = amount * edge_len / total_len;
        add_zone_pair_static_corridor_load(cur, next, edge_amount);
        cur = next;
    }
    return true;
}

float RandomRouteLoadScheduler::point_zone_load(int loc) const {
    int z = zone_of_loc(loc);
    if (z < 0 || z >= route_grid_count_ ||
        z >= static_cast<int>(future_load_.size())) {
        return 0.0f;
    }
    return zone_capacity_adjusted_load(z);
}

float RandomRouteLoadScheduler::segment_corridor_load(int from, int to) const {
    if (route_grid_count_ <= 0 || from < 0 || to < 0) return 0.0f;
    return zone_pair_route_load(zone_of_loc(from), zone_of_loc(to));
}

float RandomRouteLoadScheduler::zone_pair_corridor_load(int from_zone,
                                                        int to_zone) const {
    if (from_zone < 0 || to_zone < 0 ||
        from_zone >= route_grid_count_ || to_zone >= route_grid_count_) {
        return 0.0f;
    }
    size_t idx = static_cast<size_t>(from_zone) * route_grid_count_ + to_zone;
    if (idx >= zone_pair_load_cache_.size()) return 0.0f;
    return zone_pair_load_cache_[idx];
}

float RandomRouteLoadScheduler::zone_pair_route_load(int from_zone,
                                                     int to_zone) const {
    if (from_zone < 0 || to_zone < 0 ||
        from_zone >= route_grid_count_ || to_zone >= route_grid_count_) {
        return 0.0f;
    }

    size_t idx = static_cast<size_t>(from_zone) * route_grid_count_ + to_zone;
    if (cfg().random_route_load_dynamic_shortest &&
        dynamic_zone_pair_cache_valid_ &&
        idx < dynamic_zone_pair_load_cache_.size()) {
        return dynamic_zone_pair_load_cache_[idx];
    }
    return zone_pair_corridor_load(from_zone, to_zone);
}

RandomRouteLoadScheduler::TaskRouteLoadInfo
RandomRouteLoadScheduler::compute_task_route_load_info(const TaskInfo& task) const {
    TaskRouteLoadInfo info;
    if (task.start_location < 0) return info;

    info.start_load = point_zone_load(task.start_location);
    if (task.errand_locations.size() < 2) {
        info.avg_load = info.start_load;
        return info;
    }

    float weighted = 0.0f;
    float length_sum = 0.0f;
    for (size_t i = 1; i < task.errand_locations.size(); i++) {
        int from = task.errand_locations[i - 1];
        int to = task.errand_locations[i];
        int d = std::max(1, dist(from, to));
        float load = segment_corridor_load(from, to);
        info.segments.push_back(
            TaskSegmentLoad{load, static_cast<float>(d)});
        weighted += load * static_cast<float>(d);
        length_sum += static_cast<float>(d);
    }

    info.length = length_sum;
    info.avg_load = length_sum > 0.0f ? weighted / length_sum : info.start_load;
    return info;
}

std::vector<RandomRouteLoadScheduler::TaskRouteLoadInfo>
RandomRouteLoadScheduler::compute_task_route_load_infos(
    const std::vector<TaskInfo>& tasks) const {
    std::vector<TaskRouteLoadInfo> infos(tasks.size());
    for (int i = 0; i < static_cast<int>(tasks.size()); i++) {
        infos[i] = compute_task_route_load_info(tasks[i]);
    }
    return infos;
}

float RandomRouteLoadScheduler::task_route_load(const TaskInfo& task) const {
    TaskRouteLoadInfo info = compute_task_route_load_info(task);
    return info.avg_load + 0.25f * info.start_load;
}

float RandomRouteLoadScheduler::pair_route_load(
    int agent_loc,
    const TaskInfo& task,
    const TaskRouteLoadInfo& task_load) const {
    if (agent_loc < 0 || task.start_location < 0) {
        return task_load.avg_load + 0.25f * task_load.start_load;
    }

    int agent_zone = zone_of_loc(agent_loc);
    int task_zone = zone_of_loc(task.start_location);
    float agent_start_load = zone_pair_route_load(agent_zone, task_zone);
    int agent_start_dist = 0;
    if (agent_loc != task.start_location) {
        agent_start_dist = std::max(1, dist(agent_loc, task.start_location));
    }

    float total_len = static_cast<float>(agent_start_dist) + task_load.length;
    if (total_len <= 0.0f) {
        return point_zone_load(task.start_location);
    }

    float weighted = 0.0f;
    float cumulative = 0.0f;
    if (agent_start_dist > 0) {
        float d = static_cast<float>(agent_start_dist);
        weighted += agent_start_load * d * route_load_decay_factor(0.5f * d);
        cumulative += d;
    }
    for (const TaskSegmentLoad& seg : task_load.segments) {
        if (seg.length <= 0.0f) continue;
        float mid_dist = cumulative + 0.5f * seg.length;
        weighted += seg.load * seg.length * route_load_decay_factor(mid_dist);
        cumulative += seg.length;
    }

    return weighted / total_len + 0.25f * task_load.start_load;
}

void RandomRouteLoadScheduler::schedule(int time_limit,
                                        std::vector<int>& proposed_schedule,
                                        SharedEnvironment* env) {
    auto sched_entry_time = std::chrono::steady_clock::now();
    auto deadline = sched_entry_time + std::chrono::milliseconds(time_limit);

    env_ = env;
    if (!env_) return;

    int n_agents = env_->num_of_agents;
    if (n_agents <= 0 || env_->curr_states.empty() ||
        static_cast<int>(env_->curr_states.size()) < n_agents ||
        static_cast<int>(proposed_schedule.size()) < n_agents) {
        std::cerr << "[RandomRouteLoad] invalid state, skipping\n";
        return;
    }

    if (rows_ != env_->rows || cols_ != env_->cols ||
        map_size_ != env_->rows * env_->cols) {
        rows_ = env_->rows;
        cols_ = env_->cols;
        map_size_ = rows_ * cols_;
        build_route_grid();
    }

    try {
        task_pool_snapshot_ = env_->task_pool;

        std::vector<State> states_copy = env_->curr_states;
        curr_locations_snapshot_.resize(n_agents);
        for (int i = 0; i < n_agents; i++) {
            curr_locations_snapshot_[i] =
                (i < static_cast<int>(states_copy.size())) ? states_copy[i].location : -1;
        }

        schedule_snapshot_.assign(proposed_schedule.begin(), proposed_schedule.end());
    } catch (...) {
        std::cerr << "[RandomRouteLoad] snapshot copy failed, skipping\n";
        return;
    }

    std::vector<int> local_schedule = schedule_snapshot_;
    const int schedule_timestep = std::max(env_->system_timestep, env_->curr_timestep);

    std::vector<int> free_agents;
    std::unordered_set<int> assigned_tasks;
    std::unordered_map<int, int> old_assignment;
    const bool do_reassign = cfg().reassign_enabled;
    const int reassign_min_dist = cfg().reassign_min_dist;

    for (int a = 0; a < n_agents; a++) {
        int tid = schedule_snapshot_[a];
        if (tid == -1) {
            free_agents.push_back(a);
            continue;
        }

        auto it = task_pool_snapshot_.find(tid);
        if (it == task_pool_snapshot_.end() || it->second.locations.empty()) {
            local_schedule[a] = -1;
            free_agents.push_back(a);
            continue;
        }

        const Task& task = it->second;
        bool released_for_reassign = false;
        if (do_reassign && task.idx_next_loc == 0) {
            bool too_close = false;
            if (reassign_min_dist > 0) {
                int agent_loc = curr_locations_snapshot_[a];
                int task_start = task.locations.front();
                if (agent_loc >= 0 && task_start >= 0) {
                    too_close = dist(agent_loc, task_start) < reassign_min_dist;
                }
            }
            if (!too_close) {
                old_assignment[a] = tid;
                local_schedule[a] = -1;
                free_agents.push_back(a);
                released_for_reassign = true;
            }
        }

        if (!released_for_reassign) {
            assigned_tasks.insert(tid);
        }
    }

    std::vector<int> free_tasks;
    free_tasks.reserve(task_pool_snapshot_.size());
    for (const auto& [tid, task] : task_pool_snapshot_) {
        if (assigned_tasks.find(tid) != assigned_tasks.end()) continue;
        if (task.locations.empty()) continue;
        if (task.idx_next_loc < 0 ||
            task.idx_next_loc >= static_cast<int>(task.locations.size())) {
            continue;
        }
        free_tasks.push_back(tid);
    }

    dynamic_zone_pair_cache_valid_ = false;
    reset_future_load(local_schedule);

    if (free_agents.empty() || free_tasks.empty()) {
        return;
    }

    std::vector<AgentInfo> remaining_agents;
    remaining_agents.reserve(free_agents.size());
    for (int a : free_agents) {
        AgentInfo ai = make_agent_info_from_snapshot(a);
        if (ai.location < 0 || ai.location >= map_size_) continue;
        remaining_agents.push_back(ai);
    }

    std::vector<TaskInfo> remaining_tasks;
    remaining_tasks.reserve(free_tasks.size());
    for (int tid : free_tasks) {
        TaskInfo ti = make_task_info_from_snapshot(tid);
        if (ti.start_location < 0) continue;
        remaining_tasks.push_back(std::move(ti));
    }

    if (remaining_agents.empty() || remaining_tasks.empty()) return;

    heap_matcher_.set_timestep_info(schedule_timestep, cfg().game_total_steps);

    std::unordered_map<int, int> old_task_by_agent_loc;
    if (!old_assignment.empty() && cfg().reassign_keep_bias > 0.0f) {
        old_task_by_agent_loc.reserve(old_assignment.size());
        for (const AgentInfo& agent : remaining_agents) {
            auto it = old_assignment.find(agent.id);
            if (it != old_assignment.end()) {
                old_task_by_agent_loc[agent.location] = it->second;
            }
        }
        if (cfg().debug_level >= 1 && !old_task_by_agent_loc.empty()) {
            std::cerr << "[RandomRouteLoad] reassign released="
                      << old_task_by_agent_loc.size()
                      << " keep_bias=" << cfg().reassign_keep_bias << "\n";
        }
    }

    int batch_size = cfg().random_route_load_batch_size;
    if (batch_size <= 0) {
        batch_size = static_cast<int>(remaining_agents.size());
    }

    std::unordered_set<int> matched_tasks;
    int batch_count = 0;
    int matched_count = 0;

    while (!remaining_agents.empty() && !remaining_tasks.empty()) {
        auto now = std::chrono::steady_clock::now();
        if (now >= deadline) break;

        int max_matches = std::min<int>(
            batch_size,
            std::min(remaining_agents.size(), remaining_tasks.size()));
        int batches_left = (static_cast<int>(remaining_agents.size()) + batch_size - 1) /
                           std::max(1, batch_size);
        int remaining_ms = static_cast<int>(
            std::chrono::duration_cast<std::chrono::milliseconds>(deadline - now).count());
        if (remaining_ms <= 0) break;

        int batch_ms = std::max(5, remaining_ms / std::max(1, batches_left));
        auto batch_deadline = std::min(deadline, now + std::chrono::milliseconds(batch_ms));

        const float route_weight = cfg().random_route_load_weight;
        const float keep_bias =
            old_task_by_agent_loc.empty() ? 0.0f : cfg().reassign_keep_bias;
        std::vector<TaskRouteLoadInfo> task_load_infos;
        if (route_weight > 0.0f) {
            refresh_zone_pair_load_cache();
            refresh_dynamic_zone_pair_cache(route_weight);
            task_load_infos = compute_task_route_load_infos(remaining_tasks);
            if (cfg().debug_level >= 2) {
                int probe_bucket = schedule_timestep / 1000;
                if (probe_bucket != last_route_load_probe_bucket_) {
                    last_route_load_probe_bucket_ = probe_bucket;
                    std::vector<float> loads;
                    std::vector<float> penalties;
                    std::vector<float> bases;
                    std::vector<float> ratios;
                    int na = std::min<int>(48, remaining_agents.size());
                    int nt = std::min<int>(128, remaining_tasks.size());
                    int a_stride = std::max<int>(1, remaining_agents.size() / std::max(1, na));
                    int t_stride = std::max<int>(1, remaining_tasks.size() / std::max(1, nt));
                    loads.reserve(static_cast<size_t>(na) * nt);
                    penalties.reserve(loads.capacity());
                    bases.reserve(loads.capacity());
                    ratios.reserve(loads.capacity());
                    for (int ai = 0, ac = 0; ai < static_cast<int>(remaining_agents.size()) && ac < na;
                         ai += a_stride, ac++) {
                        const AgentInfo& agent = remaining_agents[ai];
                        for (int ti = 0, tc = 0; ti < static_cast<int>(remaining_tasks.size()) && tc < nt;
                             ti += t_stride, tc++) {
                            const TaskInfo& task = remaining_tasks[ti];
                            float load = pair_route_load(agent.location, task, task_load_infos[ti]);
                            float penalty = route_weight * load;
                            int d = std::max(0, dist(agent.location, task.start_location));
                            float dist_score = cfg().heap_dist_weight *
                                std::pow(static_cast<float>(d), cfg().game_dist_pow);
                            float base = dist_score + task_load_infos[ti].length;
                            loads.push_back(load);
                            penalties.push_back(penalty);
                            bases.push_back(base);
                            ratios.push_back(base > 1e-3f ? penalty / base : 0.0f);
                        }
                    }
                    auto pct = [](std::vector<float> v, double p) -> float {
                        if (v.empty()) return 0.0f;
                        size_t k = static_cast<size_t>((v.size() - 1) * p);
                        std::nth_element(v.begin(), v.begin() + k, v.end());
                        return v[k];
                    };
                    std::cerr << "[RandomRouteLoadProbe] tick=" << schedule_timestep
                              << " pairs=" << loads.size()
                              << " weight=" << route_weight
                              << " keep_bias=" << keep_bias
                              << " decay=" << cfg().random_route_load_decay
                              << " len_write=" << cfg().random_route_load_length_weighted_write
                              << " len_scale=" << cfg().random_route_load_write_length_scale
                              << " dyn_sp=" << cfg().random_route_load_dynamic_shortest
                              << " dyn_w=" << cfg().random_route_load_dynamic_path_weight
                              << " cap_norm=" << cfg().random_route_load_capacity_norm
                              << " min_valid=" << cfg().random_route_load_min_valid_ratio
                              << " load(p50/p90/p99)=" << pct(loads, 0.50) << "/"
                              << pct(loads, 0.90) << "/" << pct(loads, 0.99)
                              << " penalty(p50/p90/p99)=" << pct(penalties, 0.50) << "/"
                              << pct(penalties, 0.90) << "/" << pct(penalties, 0.99)
                              << " base(p50/p90/p99)=" << pct(bases, 0.50) << "/"
                              << pct(bases, 0.90) << "/" << pct(bases, 0.99)
                              << " pen/base(p50/p90/p99)=" << pct(ratios, 0.50) << "/"
                              << pct(ratios, 0.90) << "/" << pct(ratios, 0.99)
                              << "\n";
                }
            }
        }

        if (route_weight > 0.0f || keep_bias > 0.0f) {
            heap_matcher_.set_pair_extra_cost_func(
                [this, &remaining_tasks,
                 task_load_infos = std::move(task_load_infos),
                 &old_task_by_agent_loc,
                 route_weight,
                 keep_bias](int agent_loc, int task_idx) -> float {
                    if (task_idx < 0 ||
                        task_idx >= static_cast<int>(remaining_tasks.size())) {
                        return 0.0f;
                    }
                    float extra = 0.0f;
                    if (route_weight > 0.0f &&
                        task_idx < static_cast<int>(task_load_infos.size())) {
                        extra += route_weight *
                                 pair_route_load(agent_loc, remaining_tasks[task_idx],
                                                 task_load_infos[task_idx]);
                    }
                    if (keep_bias > 0.0f) {
                        auto old_it = old_task_by_agent_loc.find(agent_loc);
                        if (old_it != old_task_by_agent_loc.end() &&
                            old_it->second == remaining_tasks[task_idx].id) {
                            extra -= keep_bias;
                        }
                    }
                    return extra;
                });
        } else {
            heap_matcher_.set_pair_extra_cost_func(nullptr);
        }

        MatchResult matches = heap_matcher_.match(
            remaining_agents, remaining_tasks, batch_deadline,
            -1, old_assignment, max_matches);
        heap_matcher_.set_pair_extra_cost_func(nullptr);

        if (matches.empty()) break;

        std::unordered_set<int> matched_agents_this_batch;
        std::unordered_set<int> matched_tasks_this_batch;
        std::unordered_map<int, AgentInfo> agent_by_id;
        std::unordered_map<int, TaskInfo> task_by_id;
        agent_by_id.reserve(remaining_agents.size());
        task_by_id.reserve(remaining_tasks.size());
        for (const auto& a : remaining_agents) agent_by_id[a.id] = a;
        for (const auto& t : remaining_tasks) task_by_id[t.id] = t;

        for (const auto& [aid, tid] : matches) {
            if (aid < 0 || aid >= static_cast<int>(local_schedule.size())) continue;
            auto ait = agent_by_id.find(aid);
            auto tit = task_by_id.find(tid);
            if (ait == agent_by_id.end() || tit == task_by_id.end()) continue;

            local_schedule[aid] = tid;
            matched_agents_this_batch.insert(aid);
            matched_tasks_this_batch.insert(tid);
            matched_tasks.insert(tid);
            add_assigned_route_to_load(ait->second, tit->second, 1.0f);
            matched_count++;
        }

        if (matched_agents_this_batch.empty()) break;

        remaining_agents.erase(
            std::remove_if(remaining_agents.begin(), remaining_agents.end(),
                [&](const AgentInfo& a) {
                    return matched_agents_this_batch.count(a.id) > 0;
                }),
            remaining_agents.end());
        remaining_tasks.erase(
            std::remove_if(remaining_tasks.begin(), remaining_tasks.end(),
                [&](const TaskInfo& t) {
                    return matched_tasks_this_batch.count(t.id) > 0;
                }),
            remaining_tasks.end());

        batch_count++;
    }

    auto task_it = remaining_tasks.begin();
    int fallback_count = 0;
    if (cfg().schedule_fallback_enabled) {
        for (const AgentInfo& agent : remaining_agents) {
            if (agent.id < 0 || agent.id >= static_cast<int>(local_schedule.size())) continue;
            if (local_schedule[agent.id] != -1) continue;
            while (task_it != remaining_tasks.end() &&
                   matched_tasks.count(task_it->id) > 0) {
                ++task_it;
            }
            if (task_it == remaining_tasks.end()) break;
            local_schedule[agent.id] = task_it->id;
            matched_tasks.insert(task_it->id);
            add_assigned_route_to_load(agent, *task_it, 1.0f);
            ++task_it;
            fallback_count++;
        }
    } else if (cfg().debug_level >= 1 && !remaining_agents.empty()) {
        std::cerr << "[RandomRouteLoad] fallback disabled: "
                  << remaining_agents.size()
                  << " remaining agents not sequentially assigned\n";
    }

    for (int a : free_agents) {
        if (a >= 0 && a < static_cast<int>(proposed_schedule.size())) {
            proposed_schedule[a] = local_schedule[a];
        }
    }

    if (cfg().debug_level >= 1) {
        std::cerr << "[RandomRouteLoad] free_agents=" << free_agents.size()
                  << " free_tasks=" << free_tasks.size()
                  << " batches=" << batch_count
                  << " matched=" << matched_count
                  << " fallback=" << fallback_count << "\n";
    }
}

} // namespace orz_engine
