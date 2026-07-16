// optsvr/schedule/GameBalanceOpt/GameBalanceOptScheduler.cpp
// 中心线zone调度器实现 — BFS 中心线 + 24×24 zone 密度 + 方向锁 + 三层豁免
#include "GameBalanceOptScheduler.h"
#include "HeuristicMatrix.h"
#include "orz_highway.h"
#include "ScheduleStatsExport.h"
#include <atomic>
#include <climits>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <thread>

namespace optsvr {

// ================================================================
// 常量（对应 IDEA/GameOpt 的 config.h 宏 + RuntimeConfig 参数）
// ================================================================
static constexpr int    CONTAIN_EXPAND   = 1;
static constexpr int    ROUTE_THRESHOLD  = 20;
static constexpr int    GAME_H_CAP       = 100000;
static constexpr float  INVALID_SCORE    = -1e9f;
static constexpr float  DIVISOR_TABLE[21] = {
    1.0f, 1.1f, 1.2f, 1.3f, 1.4f,
    1.5f, 1.6f, 1.7f, 1.8f, 1.9f,
    2.0f, 2.1f, 2.2f, 2.3f, 2.4f,
    2.5f, 2.6f, 2.7f, 2.8f, 2.9f,
    3.0f
};
static inline bool use_direct_distance_score() {
    return cfg().game_distance_score_mode == 1 || cfg().game_distance_score_mode == 3;
}

static inline bool is_valid_score(float score) {
    return score > INVALID_SCORE * 0.5f;
}

static inline float distance_score(float dist) {
    return use_direct_distance_score() ? -dist : 1.0f / (dist + 1.0f);
}

static inline float apply_route_density_penalty(float score, float divisor) {
    return use_direct_distance_score() ? score : score / divisor;
}

static inline float route_density_penalty_cost(int density, int threshold) {
    if (density <= threshold) return 0.0f;
    const int exponent = std::max(0, density - threshold - 1);
    return 10.0f * std::pow(1.1f, static_cast<float>(exponent));
}

static inline float future_flow_soft_penalty_from_ratio(float ratio) {
    constexpr float kStartRatio = 0.70f;
    constexpr float kAtCapMultiplier = 5.0f;
    if (ratio < kStartRatio) return 0.0f;
    const float base = std::max(0.0f, cfg().game_flow_soft_penalty_base);
    const float at_cap = base * kAtCapMultiplier;
    if (ratio <= 1.0f) {
        const float progress = (ratio - kStartRatio) / (1.0f - kStartRatio);
        return base + (at_cap - base) * progress;
    }
    float growth = cfg().game_flow_soft_over_cap_growth;
    if (!std::isfinite(growth)) growth = GAME_FLOW_SOFT_OVER_CAP_GROWTH;
    growth = std::max(0.0f, growth);
    return at_cap + base * 100.0f * growth * (ratio - 1.0f);
}

// ================================================================
// compute_dtw — BFS 到墙距离
// ================================================================
void GameBalanceOptScheduler::compute_dtw(const SharedEnvironment* env, std::vector<int>& dtw) {
    dtw.assign(total_, INT_MAX / 2);
    std::queue<int> q;
    for (int i = 0; i < total_; i++)
        if (env->map[i] != 0) { dtw[i] = 0; q.push(i); }
    while (!q.empty()) {
        int cur = q.front(); q.pop();
        int nd = dtw[cur] + 1;
        int neigh[4] = {cur - cols_, cur + cols_, cur - 1, cur + 1};
        for (int n : neigh)
            if (n >= 0 && n < total_ && env->map[n] == 0 && nd < dtw[n])
                { dtw[n] = nd; q.push(n); }
    }
}

// ================================================================
// project — 投影 loc 到 (main_dist, branch_dist)
// ================================================================
void GameBalanceOptScheduler::project(int loc, int& m, int& b) const {
    if (!ready_ || loc < 0 || loc >= (int)branch_dist_.size()) {
        m = -1; b = -1; return;
    }
    b = branch_dist_[loc];
    m = (b >= 0) ? main_dist_[loc] : -1;
}

float GameBalanceOptScheduler::projected_distance_between(int from, int to) const {
    int fm, fb, tm, tb;
    project(from, fm, fb);
    project(to, tm, tb);
    if (fm < 0 || tm < 0) return -1.0f;
    return static_cast<float>(std::abs(fm - tm) + std::abs(fb - tb));
}

float GameBalanceOptScheduler::score_distance_between(int from, int to) const {
    if (cfg().game_use_bfs_distance && cfg().game_require_bfs_matrix &&
        env_ != nullptr && !heuristic_matrix().is_ready()) {
        std::cerr << "[FATAL] BFS matrix not ready but GAME_REQUIRE_BFS_MATRIX=true\n"
                  << "  Preprocess time or memory may be insufficient for 17GB matrix.\n"
                  << "  Set GAME_REQUIRE_BFS_MATRIX=0 to fall back to projected distance.\n";
        std::cerr << "[FATAL] cfg_check: use_bfs=" << cfg().game_use_bfs_distance
                  << " require=" << cfg().game_require_bfs_matrix
                  << " env=" << (env_!=nullptr?"ok":"null")
                  << " ready=" << heuristic_matrix().is_ready() << "\n";
        volatile int* ptr = nullptr;
        *ptr = 0;  // 故意空指针崩溃
    }
    if (cfg().game_use_bfs_distance && env_ != nullptr && heuristic_matrix().is_ready()) {
        if (from < 0 || to < 0 || from >= total_ || to >= total_) return -1.0f;
        int h = get_h_fast(env_, from, to);
        if (h < 0 || h >= GAME_H_CAP) return -1.0f;
        return static_cast<float>(h);
    }
    return projected_distance_between(from, to);
}

bool GameBalanceOptScheduler::game_directional_flow_enabled() const {
    const int planner_mode = cfg().planner_mode;
    return (planner_mode == 16 || planner_mode == 14) &&
           cfg().game_highway_enabled && cfg().game_dir_flow_enabled &&
           cfg().game_dir_flow_cap > 0;
}

bool GameBalanceOptScheduler::task_has_channel_errand(const std::vector<int>& locs) const {
    const int planner_mode = cfg().planner_mode;
    if ((planner_mode != 16 && planner_mode != 14) ||
        !cfg().game_highway_enabled || !cfg().game_skip_channel_errands) return false;
    for (int loc : locs) {
        if (ORZ_Highway::is_highway_cell(loc)) return true;
    }
    return false;
}

int GameBalanceOptScheduler::zone24_of_loc(int loc) const {
    if (loc < 0 || loc >= total_ || cols_ <= 0 || n24c_ <= 0) return -1;
    const int r = loc / cols_;
    const int c = loc % cols_;
    const int zid = (r / 24) * n24c_ + (c / 24);
    const int nz = n24c_ * n24r_;
    return (zid >= 0 && zid < nz) ? zid : -1;
}

void GameBalanceOptScheduler::build_zone24_distances() {
    const int nz = n24c_ * n24r_;
    constexpr uint16_t kInf = 65535;
    zone24_dist_.assign(std::max(0, nz * nz), kInf);
    zone24_neighbors_.assign(std::max(0, nz), {});
    if (nz <= 0 || (int)zone24_has_free_.size() != nz) return;

    // Two geometric neighbor zones are connected only when at least one
    // traversable cell edge actually crosses their shared boundary.
    for (int loc = 0; loc < total_; ++loc) {
        if (env_ == nullptr || env_->map[loc] != 0) continue;
        const int zid = zone24_of_loc(loc);
        if (zid < 0) continue;
        const int r = loc / cols_;
        const int c = loc % cols_;
        const int next[2] = {
            (c + 1 < cols_) ? loc + 1 : -1,
            (r + 1 < rows_) ? loc + cols_ : -1,
        };
        for (int nb_loc : next) {
            if (nb_loc < 0 || env_->map[nb_loc] != 0) continue;
            const int nb_zid = zone24_of_loc(nb_loc);
            if (nb_zid < 0 || nb_zid == zid) continue;
            zone24_neighbors_[zid].push_back(nb_zid);
            zone24_neighbors_[nb_zid].push_back(zid);
        }
    }
    for (auto& neighbors : zone24_neighbors_) {
        std::sort(neighbors.begin(), neighbors.end());
        neighbors.erase(std::unique(neighbors.begin(), neighbors.end()),
                        neighbors.end());
    }

    std::queue<int> q;
    for (int src = 0; src < nz; ++src) {
        if (!zone24_has_free_[src]) continue;
        uint16_t* row = zone24_dist_.data() + src * nz;
        row[src] = 0;
        q.push(src);
        while (!q.empty()) {
            const int cur = q.front();
            q.pop();
            for (int nb : zone24_neighbors_[cur]) {
                if (nb < 0 || nb >= nz || !zone24_has_free_[nb]) continue;
                if (row[nb] != kInf) continue;
                row[nb] = static_cast<uint16_t>(row[cur] + 1);
                q.push(nb);
            }
        }
    }
}

std::vector<int> GameBalanceOptScheduler::shortest_zone_path(
    int from_zid, int to_zid, bool randomize) const {
    const int nz = n24c_ * n24r_;
    constexpr uint16_t kInf = 65535;
    if (from_zid < 0 || to_zid < 0 || from_zid >= nz || to_zid >= nz)
        return {};
    if (from_zid == to_zid) return {from_zid};
    if ((int)zone24_has_free_.size() != nz ||
        !zone24_has_free_[from_zid] || !zone24_has_free_[to_zid] ||
        (int)zone24_dist_.size() != nz * nz)
        return {};
    const uint16_t* dist = zone24_dist_.data() + from_zid * nz;
    if (dist[to_zid] == kInf) return {};

    std::vector<int> path;
    path.reserve(static_cast<size_t>(dist[to_zid]) + 1);
    int cur = from_zid;
    path.push_back(cur);
    while (cur != to_zid) {
        std::vector<int> nexts;
        for (int nb : zone24_neighbors_[cur]) {
            const uint16_t nb_to_target = zone24_dist_[nb * nz + to_zid];
            if (nb >= 0 && nb < nz && dist[nb] == dist[cur] + 1 &&
                nb_to_target != kInf &&
                dist[nb] + nb_to_target == dist[to_zid])
                nexts.push_back(nb);
        }
        if (nexts.empty()) return {};
        if (randomize && nexts.size() > 1) {
            std::uniform_int_distribution<int> pick(0, (int)nexts.size() - 1);
            cur = nexts[pick(ff_rng_)];
        } else {
            std::sort(nexts.begin(), nexts.end());
            cur = nexts.front();
        }
        path.push_back(cur);
    }
    return path;
}

bool GameBalanceOptScheduler::build_task_zone_cache(
    int tid, TaskStaticCache& info) const {
    if (env_ == nullptr) return false;
    auto tit = env_->task_pool.find(tid);
    if (tit == env_->task_pool.end() || tit->second.locations.empty()) return false;

    const auto& locs = tit->second.locations;
    info.internal_zone_paths.resize(locs.size());
    info.coarse_tail_distance.assign(locs.size(), 0.0f);
    info.coarse_intrinsic_distance = 0.0f;
    info.coarse_distance_valid = true;
    std::vector<float> coarse_leg_distance(locs.size(), 0.0f);
    for (size_t i = 1; i < locs.size(); ++i) {
        const float dist = projected_distance_between(locs[i - 1], locs[i]);
        if (dist < 0.0f) {
            info.coarse_distance_valid = false;
        } else {
            coarse_leg_distance[i] = dist;
            info.coarse_intrinsic_distance += dist;
        }
        info.internal_zone_paths[i] = shortest_zone_path(
            zone24_of_loc(locs[i - 1]), zone24_of_loc(locs[i]), false);
    }
    for (int i = static_cast<int>(locs.size()) - 2; i >= 0; --i) {
        info.coarse_tail_distance[i] =
            info.coarse_tail_distance[i + 1] + coarse_leg_distance[i + 1];
    }
    info.zone_paths_ready = true;
    return true;
}

bool GameBalanceOptScheduler::build_task_distance_cache(
    int tid, TaskStaticCache& info) const {
    if (env_ == nullptr) return false;
    auto tit = env_->task_pool.find(tid);
    if (tit == env_->task_pool.end() || tit->second.locations.empty()) return false;

    const auto& locs = tit->second.locations;
    info.intrinsic_distance = 0.0f;
    info.distance_valid = true;
    info.tail_distance.assign(locs.size(), 0.0f);
    std::vector<float> leg_distance(locs.size(), 0.0f);
    for (size_t i = 1; i < locs.size(); ++i) {
        const float dist = score_distance_between(locs[i - 1], locs[i]);
        if (dist < 0.0f) {
            info.distance_valid = false;
        } else {
            leg_distance[i] = dist;
            info.intrinsic_distance += dist;
        }
    }
    for (int i = static_cast<int>(locs.size()) - 2; i >= 0; --i)
        info.tail_distance[i] = info.tail_distance[i + 1] + leg_distance[i + 1];
    info.distance_ready = true;
    return true;
}

bool GameBalanceOptScheduler::build_task_static_cache(
    int tid, TaskStaticCache& info) const {
    info = TaskStaticCache{};
    return build_task_zone_cache(tid, info) &&
           build_task_distance_cache(tid, info);
}

std::size_t GameBalanceOptScheduler::prewarm_task_static_cache(
    const std::vector<int>& tids,
    std::chrono::steady_clock::time_point deadline,
    int max_threads) {
    std::vector<int> zone_pending;
    std::vector<int> distance_pending;
    zone_pending.reserve(tids.size());
    distance_pending.reserve(tids.size());
    for (int tid : tids) {
        auto cached = task_static_cache_.find(tid);
        if (cached == task_static_cache_.end() || !cached->second.zone_paths_ready)
            zone_pending.push_back(tid);
        if (cached == task_static_cache_.end() || !cached->second.distance_ready)
            distance_pending.push_back(tid);
    }
    if ((zone_pending.empty() && distance_pending.empty()) ||
        std::chrono::steady_clock::now() >= deadline)
        return 0;

    auto deduplicate = [](std::vector<int>& pending) {
        std::sort(pending.begin(), pending.end());
        pending.erase(std::unique(pending.begin(), pending.end()), pending.end());
    };
    deduplicate(zone_pending);
    deduplicate(distance_pending);

    int cache_threads = cfg().num_threads;
    if (cache_threads <= 0) {
        cache_threads = static_cast<int>(std::thread::hardware_concurrency());
        if (cache_threads <= 0) cache_threads = 4;
    }
    const int zone_threads = std::max(1,
        std::min<int>(cache_threads, std::max<std::size_t>(1, zone_pending.size())));
    const auto zone_start = std::chrono::steady_clock::now();
    std::vector<TaskStaticCache> zone_built(zone_pending.size());
    std::vector<unsigned char> zone_ok(zone_pending.size(), 0);
    std::atomic<std::size_t> next_zone{0};
    std::vector<std::thread> workers;
    workers.reserve(zone_threads);
    for (int t = 0; t < zone_threads; ++t) {
        workers.emplace_back([&]() {
            while (std::chrono::steady_clock::now() < deadline) {
                const std::size_t i = next_zone.fetch_add(1, std::memory_order_relaxed);
                if (i >= zone_pending.size()) break;
                if (build_task_zone_cache(zone_pending[i], zone_built[i]))
                    zone_ok[i] = 1;
            }
        });
    }
    for (auto& worker : workers) worker.join();
    workers.clear();

    std::size_t zone_added = 0;
    for (std::size_t i = 0; i < zone_pending.size(); ++i) {
        if (!zone_ok[i]) continue;
        auto [it, inserted] = task_static_cache_.try_emplace(zone_pending[i]);
        it->second.coarse_intrinsic_distance =
            zone_built[i].coarse_intrinsic_distance;
        it->second.coarse_distance_valid = zone_built[i].coarse_distance_valid;
        it->second.coarse_tail_distance =
            std::move(zone_built[i].coarse_tail_distance);
        it->second.internal_zone_paths =
            std::move(zone_built[i].internal_zone_paths);
        it->second.zone_paths_ready = true;
        if (inserted) ++zone_added;
    }
    const auto zone_end = std::chrono::steady_clock::now();

    if (max_threads > 0) cache_threads = std::min(cache_threads, max_threads);
    const int distance_threads = std::max(1,
        std::min<int>(cache_threads, std::max<std::size_t>(1, distance_pending.size())));
    std::vector<TaskStaticCache> distance_built(distance_pending.size());
    std::vector<unsigned char> distance_ok(distance_pending.size(), 0);
    std::atomic<std::size_t> next_distance{0};
    workers.reserve(distance_threads);
    for (int t = 0; t < distance_threads; ++t) {
        workers.emplace_back([&]() {
            while (std::chrono::steady_clock::now() < deadline) {
                const std::size_t i =
                    next_distance.fetch_add(1, std::memory_order_relaxed);
                if (i >= distance_pending.size()) break;
                if (build_task_distance_cache(
                        distance_pending[i], distance_built[i]))
                    distance_ok[i] = 1;
            }
        });
    }
    for (auto& worker : workers) worker.join();

    std::size_t distance_added = 0;
    for (std::size_t i = 0; i < distance_pending.size(); ++i) {
        if (!distance_ok[i]) continue;
        auto [it, inserted] = task_static_cache_.try_emplace(distance_pending[i]);
        (void)inserted;
        it->second.intrinsic_distance = distance_built[i].intrinsic_distance;
        it->second.distance_valid = distance_built[i].distance_valid;
        it->second.tail_distance = std::move(distance_built[i].tail_distance);
        it->second.distance_ready = true;
        ++distance_added;
    }
    const auto distance_end = std::chrono::steady_clock::now();

    if (cfg().debug_level >= 1 && tids.size() >= 100) {
        const auto zone_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            zone_end - zone_start).count();
        const auto distance_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            distance_end - zone_end).count();
        std::cerr << "[GameTaskCache] tasks=" << tids.size()
                  << " zone=" << zone_added << "@" << zone_ms << "ms"
                  << " dist=" << distance_added << "@" << distance_ms << "ms\n";
    }
    return distance_added;
}

const GameBalanceOptScheduler::TaskStaticCache*
GameBalanceOptScheduler::ensure_task_static_cache(int tid) {
    auto cached = task_static_cache_.find(tid);
    if (cached != task_static_cache_.end() && cached->second.distance_ready &&
        cached->second.zone_paths_ready)
        return &cached->second;
    TaskStaticCache info;
    if (!build_task_static_cache(tid, info)) return nullptr;
    auto [it, inserted] = task_static_cache_.insert_or_assign(tid, std::move(info));
    (void)inserted;
    return &it->second;
}

const std::vector<int>* GameBalanceOptScheduler::cached_internal_zone_path(
    int tid, int edge_idx, int from_loc, const std::vector<int>& locs) const {
    if (tid < 0 || edge_idx <= 0 || edge_idx >= static_cast<int>(locs.size()) ||
        from_loc != locs[edge_idx - 1])
        return nullptr;
    auto cached = task_static_cache_.find(tid);
    if (cached == task_static_cache_.end() ||
        !cached->second.zone_paths_ready ||
        edge_idx >= static_cast<int>(cached->second.internal_zone_paths.size()))
        return nullptr;
    return &cached->second.internal_zone_paths[edge_idx];
}

void GameBalanceOptScheduler::reset_directional_future_flow() {
    const int nz = n24c_ * n24r_;
    zone_future_pos_.assign(std::max(0, nz), 0.0f);
    zone_future_neg_.assign(std::max(0, nz), 0.0f);
    zone_future_exempt_pos_.assign(std::max(0, nz), 0.0f);
    zone_future_exempt_neg_.assign(std::max(0, nz), 0.0f);
}

float GameBalanceOptScheduler::directional_zone_cap(int zid, int intent) const {
    (void)intent;
    const float base_cap = static_cast<float>(std::max(1, cfg().game_dir_flow_cap));
    if (!cfg().game_dir_flow_dynamic_cap || zid < 0 ||
        zid >= static_cast<int>(zone_dynamic_cap_.size()))
        return base_cap;
    const float cap = zone_dynamic_cap_[zid];
    return std::isfinite(cap) && cap > 0.0f ? cap : base_cap;
}

void GameBalanceOptScheduler::reset_dynamic_zone_cap(const SharedEnvironment* env) {
    const int nz = n24c_ * n24r_;
    const float base_cap = static_cast<float>(std::max(1, cfg().game_dir_flow_cap));
    zone_stall_samples_.clear();
    zone_stall_agent_ticks_.assign(std::max(0, nz), 0.0);
    zone_stall_busy_agent_ticks_.assign(std::max(0, nz), 0.0);
    zone_stall_density_.assign(std::max(0, nz), 0.0f);
    zone_stall_busy_density_.assign(std::max(0, nz), 0.0f);
    zone_dynamic_cap_.assign(std::max(0, nz), base_cap);
    dynamic_cap_prev_agent_loc_.assign(env != nullptr ? env->num_of_agents : 0, -1);
    if (env != nullptr) {
        const int n = std::min<int>(env->num_of_agents, env->curr_states.size());
        for (int a = 0; a < n; ++a)
            dynamic_cap_prev_agent_loc_[a] = env->curr_states[a].location;
        dynamic_cap_prev_tick_ = std::max(0,
            std::max(env->system_timestep, env->curr_timestep));
    } else {
        dynamic_cap_prev_tick_ = -1;
    }
    dynamic_cap_last_update_tick_ = dynamic_cap_prev_tick_;
    dynamic_cap_last_window_ticks_ = 0;
    dynamic_cap_last_up_ = 0;
    dynamic_cap_last_down_ = 0;
    dynamic_cap_last_avg_ = base_cap;
    dynamic_cap_last_min_ = base_cap;
    dynamic_cap_last_max_ = base_cap;
    dynamic_cap_last_stall_avg_ = 0.0f;
    dynamic_cap_last_stall_max_ = 0.0f;
    dynamic_cap_last_busy_stall_avg_ = 0.0f;
    dynamic_cap_last_free_stall_avg_ = 0.0f;
    dynamic_cap_last_active_ = 0;
    dynamic_cap_last_below_base_ = 0;
    dynamic_cap_last_above_base_ = 0;
    dynamic_cap_last_at_min_ = 0;
}

void GameBalanceOptScheduler::update_dynamic_zone_cap(const SharedEnvironment* env) {
    if (env == nullptr || !ready_) return;
    const int nz = n24c_ * n24r_;
    if (nz <= 0) return;
    if ((int)zone_dynamic_cap_.size() != nz ||
        (int)zone_stall_agent_ticks_.size() != nz ||
        (int)zone_stall_busy_agent_ticks_.size() != nz ||
        (int)zone_stall_density_.size() != nz ||
        (int)zone_stall_busy_density_.size() != nz ||
        (int)dynamic_cap_prev_agent_loc_.size() != env->num_of_agents) {
        reset_dynamic_zone_cap(env);
        return;
    }
    if (!cfg().game_dir_flow_dynamic_cap) return;

    const int curr_tick = std::max(0,
        std::max(env->system_timestep, env->curr_timestep));
    if (dynamic_cap_prev_tick_ < 0 || curr_tick <= dynamic_cap_prev_tick_) {
        const int n = std::min<int>(env->num_of_agents, env->curr_states.size());
        for (int a = 0; a < n; ++a)
            dynamic_cap_prev_agent_loc_[a] = env->curr_states[a].location;
        dynamic_cap_prev_tick_ = curr_tick;
        return;
    }

    const int window = std::max(1, cfg().game_dir_flow_dynamic_cap_window);
    const int segment_start = std::max(dynamic_cap_prev_tick_, curr_tick - window);
    const int segment_ticks = curr_tick - segment_start;
    ZoneStallSample sample;
    sample.start_tick = segment_start;
    sample.end_tick = curr_tick;
    sample.stationary_agents.assign(nz, 0);
    sample.stationary_busy_agents.assign(nz, 0);

    const int n = std::min<int>(env->num_of_agents, env->curr_states.size());
    for (int a = 0; a < n; ++a) {
        const int loc = env->curr_states[a].location;
        if (loc >= 0 && loc == dynamic_cap_prev_agent_loc_[a]) {
            const int zid = zone24_of_loc(loc);
            if (zid >= 0 && zid < nz && sample.stationary_agents[zid] < UINT16_MAX) {
                ++sample.stationary_agents[zid];
                const bool busy = a < static_cast<int>(env->curr_task_schedule.size()) &&
                                  env->curr_task_schedule[a] >= 0;
                if (busy && sample.stationary_busy_agents[zid] < UINT16_MAX)
                    ++sample.stationary_busy_agents[zid];
            }
        }
        dynamic_cap_prev_agent_loc_[a] = loc;
    }
    dynamic_cap_prev_tick_ = curr_tick;

    if (segment_ticks > 0) {
        for (int zid = 0; zid < nz; ++zid) {
            zone_stall_agent_ticks_[zid] +=
                static_cast<double>(sample.stationary_agents[zid]) * segment_ticks;
            zone_stall_busy_agent_ticks_[zid] +=
                static_cast<double>(sample.stationary_busy_agents[zid]) * segment_ticks;
        }
        zone_stall_samples_.push_back(std::move(sample));
    }

    const int cutoff = curr_tick - window;
    while (!zone_stall_samples_.empty() &&
           zone_stall_samples_.front().end_tick <= cutoff) {
        const auto& old = zone_stall_samples_.front();
        const int ticks = old.end_tick - old.start_tick;
        for (int zid = 0; zid < nz; ++zid) {
            zone_stall_agent_ticks_[zid] -=
                static_cast<double>(old.stationary_agents[zid]) * ticks;
            zone_stall_busy_agent_ticks_[zid] -=
                static_cast<double>(old.stationary_busy_agents[zid]) * ticks;
        }
        zone_stall_samples_.pop_front();
    }
    if (!zone_stall_samples_.empty() &&
        zone_stall_samples_.front().start_tick < cutoff) {
        auto& old = zone_stall_samples_.front();
        const int trimmed_ticks = cutoff - old.start_tick;
        for (int zid = 0; zid < nz; ++zid) {
            zone_stall_agent_ticks_[zid] -=
                static_cast<double>(old.stationary_agents[zid]) * trimmed_ticks;
            zone_stall_busy_agent_ticks_[zid] -=
                static_cast<double>(old.stationary_busy_agents[zid]) * trimmed_ticks;
        }
        old.start_tick = cutoff;
    }

    const int covered_ticks = zone_stall_samples_.empty()
        ? 0 : curr_tick - zone_stall_samples_.front().start_tick;
    dynamic_cap_last_window_ticks_ = covered_ticks;
    for (int zid = 0; zid < nz; ++zid) {
        const int valid_cells = zid < (int)zone24_valid_cells_.size()
            ? zone24_valid_cells_[zid] : 0;
        if (valid_cells <= 0 || covered_ticks <= 0) {
            zone_stall_density_[zid] = 0.0f;
            zone_stall_busy_density_[zid] = 0.0f;
            continue;
        }
        const double denom = static_cast<double>(valid_cells) * covered_ticks;
        zone_stall_density_[zid] = static_cast<float>(
            std::max(0.0, zone_stall_agent_ticks_[zid]) / denom);
        zone_stall_busy_density_[zid] = static_cast<float>(
            std::max(0.0, zone_stall_busy_agent_ticks_[zid]) / denom);
    }

    const int interval = std::max(1, cfg().game_dir_flow_dynamic_cap_interval);
    if (dynamic_cap_last_update_tick_ >= 0 &&
        curr_tick - dynamic_cap_last_update_tick_ < interval)
        return;
    dynamic_cap_last_update_tick_ = curr_tick;
    dynamic_cap_last_up_ = 0;
    dynamic_cap_last_down_ = 0;
    if (covered_ticks < window) return;

    const float base_cap = static_cast<float>(std::max(1, cfg().game_dir_flow_cap));
    float low = cfg().game_dir_flow_dynamic_cap_low;
    float high = cfg().game_dir_flow_dynamic_cap_high;
    if (!std::isfinite(low)) low = GAME_DIR_FLOW_DYNAMIC_CAP_LOW;
    if (!std::isfinite(high)) high = GAME_DIR_FLOW_DYNAMIC_CAP_HIGH;
    low = std::max(0.0f, low);
    high = std::max(low, high);
    float inc = cfg().game_dir_flow_dynamic_cap_inc;
    float dec = cfg().game_dir_flow_dynamic_cap_dec;
    float min_ratio = cfg().game_dir_flow_dynamic_cap_min_ratio;
    float max_ratio = cfg().game_dir_flow_dynamic_cap_max_ratio;
    if (!std::isfinite(inc)) inc = GAME_DIR_FLOW_DYNAMIC_CAP_INC;
    if (!std::isfinite(dec)) dec = GAME_DIR_FLOW_DYNAMIC_CAP_DEC;
    if (!std::isfinite(min_ratio)) min_ratio = GAME_DIR_FLOW_DYNAMIC_CAP_MIN_RATIO;
    if (!std::isfinite(max_ratio)) max_ratio = GAME_DIR_FLOW_DYNAMIC_CAP_MAX_RATIO;
    inc = std::max(0.0f, inc);
    dec = std::max(0.1f, std::min(1.0f, dec));
    min_ratio = std::max(0.05f, min_ratio);
    max_ratio = std::max(min_ratio, max_ratio);
    const float min_cap = base_cap * min_ratio;
    const float max_cap = base_cap * max_ratio;
    std::vector<int> decreased_zones;

    for (int zid = 0; zid < nz; ++zid) {
        if (zid >= (int)zone24_valid_cells_.size() || zone24_valid_cells_[zid] <= 0)
            continue;
        const float normal_flow =
            (zid < (int)zone_future_pos_.size() && zid < (int)zone_future_neg_.size())
            ? std::max(zone_future_pos_[zid], zone_future_neg_[zid]) : 0.0f;
        const float exempt_flow =
            (zid < (int)zone_future_exempt_pos_.size() &&
             zid < (int)zone_future_exempt_neg_.size())
            ? std::max(zone_future_exempt_pos_[zid], zone_future_exempt_neg_[zid]) : 0.0f;
        const bool has_route_flow = normal_flow > 0.0f || exempt_flow > 0.0f;
        if (!has_route_flow) continue;

        float cap = std::max(min_cap, std::min(max_cap, zone_dynamic_cap_[zid]));
        const float density = zone_stall_busy_density_[zid];
        if (density > high) {
            const float next = std::max(min_cap, cap * dec);
            if (next + 1e-4f < cap) {
                ++dynamic_cap_last_down_;
                decreased_zones.push_back(zid);
            }
            cap = next;
        } else if (density < low && normal_flow >= 0.70f * cap) {
            const float next = std::min(max_cap, cap + inc);
            if (next > cap + 1e-4f) ++dynamic_cap_last_up_;
            cap = next;
        }
        zone_dynamic_cap_[zid] = cap;
    }

    float cap_sum = 0.0f;
    float cap_min = max_cap;
    float cap_max = min_cap;
    float stall_sum = 0.0f;
    float free_stall_sum = 0.0f;
    float stall_max = 0.0f;
    int active_zones = 0;
    int below_base = 0;
    int above_base = 0;
    int at_min = 0;
    std::vector<int> saturated_zones;
    saturated_zones.reserve(nz);
    for (int zid = 0; zid < nz; ++zid) {
        const float pos_flow = zid < (int)zone_future_pos_.size()
            ? zone_future_pos_[zid] : 0.0f;
        const float neg_flow = zid < (int)zone_future_neg_.size()
            ? zone_future_neg_[zid] : 0.0f;
        const float flow = pos_flow + neg_flow;
        if (zid < (int)zone24_valid_cells_.size() && zone24_valid_cells_[zid] > 0) {
            if (zone_dynamic_cap_[zid] < base_cap - 1e-4f) ++below_base;
            if (zone_dynamic_cap_[zid] > base_cap + 1e-4f) ++above_base;
            if (zone_dynamic_cap_[zid] <= min_cap + 1e-4f) ++at_min;
        }
        if (flow > 0.0f) saturated_zones.push_back(zid);
        if (flow <= 0.0f && std::abs(zone_dynamic_cap_[zid] - base_cap) < 1e-4f)
            continue;
        cap_sum += zone_dynamic_cap_[zid];
        cap_min = std::min(cap_min, zone_dynamic_cap_[zid]);
        cap_max = std::max(cap_max, zone_dynamic_cap_[zid]);
        const float busy_stall = zone_stall_busy_density_[zid];
        const float free_stall = std::max(
            0.0f, zone_stall_density_[zid] - busy_stall);
        stall_sum += busy_stall;
        free_stall_sum += free_stall;
        stall_max = std::max(stall_max, busy_stall);
        ++active_zones;
    }
    dynamic_cap_last_avg_ = active_zones > 0 ? cap_sum / active_zones : base_cap;
    dynamic_cap_last_min_ = active_zones > 0 ? cap_min : base_cap;
    dynamic_cap_last_max_ = active_zones > 0 ? cap_max : base_cap;
    dynamic_cap_last_stall_avg_ = active_zones > 0 ? stall_sum / active_zones : 0.0f;
    dynamic_cap_last_stall_max_ = stall_max;
    dynamic_cap_last_busy_stall_avg_ = dynamic_cap_last_stall_avg_;
    dynamic_cap_last_free_stall_avg_ = active_zones > 0
        ? free_stall_sum / active_zones : 0.0f;
    dynamic_cap_last_active_ = active_zones;
    dynamic_cap_last_below_base_ = below_base;
    dynamic_cap_last_above_base_ = above_base;
    dynamic_cap_last_at_min_ = at_min;

    auto saturation = [&](int zid) {
        const float cap = directional_zone_cap(zid, +1);
        const float flow = std::max(zone_future_pos_[zid], zone_future_neg_[zid]);
        return cap > 0.0f ? flow / cap : 0.0f;
    };
    const int top_count = std::min<int>(8, saturated_zones.size());
    std::partial_sort(
        saturated_zones.begin(), saturated_zones.begin() + top_count,
        saturated_zones.end(), [&](int lhs, int rhs) {
            return saturation(lhs) > saturation(rhs);
        });

    std::sort(decreased_zones.begin(), decreased_zones.end(), [&](int lhs, int rhs) {
        return zone_stall_busy_density_[lhs] > zone_stall_busy_density_[rhs];
    });
    std::vector<int> low_cap_zones;
    for (int zid : saturated_zones) {
        if (zone_dynamic_cap_[zid] < base_cap - 1e-4f)
            low_cap_zones.push_back(zid);
    }
    std::sort(low_cap_zones.begin(), low_cap_zones.end(), [&](int lhs, int rhs) {
        if (std::abs(zone_dynamic_cap_[lhs] - zone_dynamic_cap_[rhs]) > 1e-4f)
            return zone_dynamic_cap_[lhs] < zone_dynamic_cap_[rhs];
        return saturation(lhs) > saturation(rhs);
    });

    if (cfg().debug_level >= 1) {
        auto format_zones = [&](const std::vector<int>& zones, int limit) {
            std::ostringstream out;
            out << std::fixed;
            const int count = std::min<int>(limit, zones.size());
            for (int i = 0; i < count; ++i) {
                const int zid = zones[i];
                const float busy_stall = zone_stall_busy_density_[zid];
                const float free_stall = std::max(
                    0.0f, zone_stall_density_[zid] - busy_stall);
                if (i > 0) out << ",";
                out << zid
                    << ":c" << std::setprecision(0) << zone_dynamic_cap_[zid]
                    << ":s" << std::setprecision(3) << zone_stall_density_[zid]
                    << "(" << busy_stall << "/" << free_stall << ")"
                    << ":f" << std::setprecision(1) << zone_future_pos_[zid]
                    << "/" << zone_future_neg_[zid]
                    << ":r" << std::setprecision(2) << saturation(zid);
            }
            return out.str();
        };
        std::cerr << "[GameDynCap] tick=" << curr_tick
                  << " window=" << covered_ticks
                  << " active=" << active_zones
                  << " cap=" << std::fixed << std::setprecision(1)
                  << dynamic_cap_last_avg_ << "/" << dynamic_cap_last_min_
                  << "/" << dynamic_cap_last_max_
                  << " zones=" << below_base << "/" << above_base
                  << " at_min=" << at_min
                  << " stall=" << std::setprecision(3)
                  << dynamic_cap_last_stall_avg_ << "/"
                  << dynamic_cap_last_stall_max_
                  << " busy_free=" << dynamic_cap_last_busy_stall_avg_
                  << "/" << dynamic_cap_last_free_stall_avg_
                  << " adjust=" << dynamic_cap_last_up_
                  << "/" << dynamic_cap_last_down_
                  << " top=" << format_zones(saturated_zones, 8)
                  << " down=" << format_zones(decreased_zones, 8)
                  << " low=" << format_zones(low_cap_zones, 8)
                  << "\n";
    }
}

void GameBalanceOptScheduler::add_directional_future_flow_route(
    int agent_loc, const std::vector<int>& locs, int start_idx, int weight, int tid) {
    if (!game_directional_flow_enabled() || !ready_ || locs.empty() || weight == 0) return;
    const int nz = n24c_ * n24r_;
    if ((int)zone_future_pos_.size() != nz ||
        (int)zone_future_neg_.size() != nz ||
        (int)zone_future_exempt_pos_.size() != nz ||
        (int)zone_future_exempt_neg_.size() != nz)
        reset_directional_future_flow();

    int prev_m = -1, prev_b = -1;
    project(agent_loc, prev_m, prev_b);
    if (prev_m < 0) return;

    float decay = cfg().game_dir_flow_decay;
    if (!std::isfinite(decay)) decay = GAME_DIR_FLOW_DECAY;
    decay = std::max(0.0f, std::min(1.0f, decay));
    float cur_weight = static_cast<float>(weight);
    const int horizon = std::max(0, cfg().game_dir_flow_zone_horizon);
    int zone_steps = 0;
    int last_zid = -1;
    int prev_loc = agent_loc;

    auto add_zid_flow = [&](int zid, int intent) -> bool {
        if (zid < 0 || zid >= nz || zid == last_zid) return true;
        const bool beyond_horizon = horizon > 0 && zone_steps >= horizon;
        if (beyond_horizon) {
            if (intent > 0) zone_future_exempt_pos_[zid] += cur_weight;
            else zone_future_exempt_neg_[zid] += cur_weight;
        } else {
            if (intent > 0) zone_future_pos_[zid] += cur_weight;
            else zone_future_neg_[zid] += cur_weight;
        }
        ++zone_steps;
        last_zid = zid;
        cur_weight *= decay;
        return true;
    };

    for (int i = std::max(0, start_idx); i < static_cast<int>(locs.size()); ++i) {
        int tm = -1, tb = -1;
        project(locs[i], tm, tb);
        if (tm < 0) continue;
        if (tm == prev_m) {
            prev_loc = locs[i];
            prev_m = tm;
            prev_b = tb;
            continue;
        }

        const int intent = (tm > prev_m) ? +1 : -1;
        const int lo = std::min(prev_m, tm);
        const int hi = std::max(prev_m, tm);

        const std::vector<int>* cached_path =
            cached_internal_zone_path(tid, i, prev_loc, locs);
        std::vector<int> generated_path;
        if (cached_path == nullptr) {
            generated_path = shortest_zone_path(
                zone24_of_loc(prev_loc), zone24_of_loc(locs[i]),
                cfg().game_dir_flow_random_shortest);
            cached_path = &generated_path;
        }
        const auto& zpath = *cached_path;
        if (!zpath.empty()) {
            for (int zid : zpath) {
                if (!add_zid_flow(zid, intent)) return;
            }
        } else {
            if (intent > 0) {
                for (const auto& zr : zone_ranges_) {
                    if (zr.min_m <= hi && zr.max_m >= lo &&
                        !add_zid_flow(zr.zone_id, intent)) return;
                }
            } else {
                for (auto it = zone_ranges_.rbegin(); it != zone_ranges_.rend(); ++it) {
                    if (it->min_m <= hi && it->max_m >= lo &&
                        !add_zid_flow(it->zone_id, intent)) return;
                }
            }
        }

        prev_loc = locs[i];
        prev_m = tm;
        prev_b = tb;
    }
}

void GameBalanceOptScheduler::add_directional_future_flow_for_task(
    int agent_loc, int tid, int start_idx) {
    adjust_directional_future_flow_for_task(agent_loc, tid, start_idx, 1);
}

void GameBalanceOptScheduler::adjust_directional_future_flow_for_task(
    int agent_loc, int tid, int start_idx, int weight) {
    if (!game_directional_flow_enabled() || env_ == nullptr) return;
    auto it = env_->task_pool.find(tid);
    if (it == env_->task_pool.end() || it->second.locations.empty()) return;
    const auto& locs = it->second.locations;
    int idx = start_idx;
    if (idx < 0) idx = it->second.idx_next_loc;
    idx = std::max(0, std::min(idx, static_cast<int>(locs.size()) - 1));
    add_directional_future_flow_route(agent_loc, locs, idx, weight, tid);
}

void GameBalanceOptScheduler::seed_directional_future_flow(const SharedEnvironment* env) {
    reset_directional_future_flow();
    if (!game_directional_flow_enabled() || env == nullptr) return;

    const int n = std::min<int>(env->num_of_agents, env->curr_task_schedule.size());
    for (int a = 0; a < n; ++a) {
        const int tid = env->curr_task_schedule[a];
        if (tid < 0 || a >= static_cast<int>(env->curr_states.size())) continue;
        add_directional_future_flow_for_task(env->curr_states[a].location, tid);
    }
}

bool GameBalanceOptScheduler::directional_future_flow_allows(
    int agent_loc, const std::vector<int>& locs, int start_idx, int tid) const {
    if (!cfg().game_dir_flow_hard_filter) return true;
    if (!game_directional_flow_enabled() || !ready_ || locs.empty()) return true;
    const int nz = n24c_ * n24r_;
    if ((int)zone_future_pos_.size() != nz ||
        (int)zone_future_neg_.size() != nz ||
        (int)zone_future_exempt_pos_.size() != nz ||
        (int)zone_future_exempt_neg_.size() != nz)
        return true;

    const int gap = std::max(0, cfg().game_dir_flow_gap);
    int prev_m = -1, prev_b = -1;
    project(agent_loc, prev_m, prev_b);
    if (prev_m < 0) return true;

    float decay = cfg().game_dir_flow_decay;
    if (!std::isfinite(decay)) decay = GAME_DIR_FLOW_DECAY;
    decay = std::max(0.0f, std::min(1.0f, decay));
    float cur_weight = 1.0f;
    const int horizon = std::max(0, cfg().game_dir_flow_zone_horizon);
    float exempt_ratio = cfg().game_dir_flow_horizon_exempt_ratio;
    if (!std::isfinite(exempt_ratio))
        exempt_ratio = GAME_DIR_FLOW_HORIZON_EXEMPT_RATIO;
    exempt_ratio = std::max(0.0f, exempt_ratio);
    const float exempt_gap = static_cast<float>(gap) * exempt_ratio;
    int zone_steps = 0;
    int last_zid = -1;
    int prev_loc = agent_loc;

    auto zid_allows = [&](int zid, int intent) -> bool {
        if (zid < 0 || zid >= nz || zid == last_zid) return true;
        const bool beyond_horizon = horizon > 0 && zone_steps >= horizon;
        const float local_cap = directional_zone_cap(zid, intent);
        const float local_exempt_cap = local_cap * exempt_ratio;
        const float same = beyond_horizon
            ? ((intent > 0) ? zone_future_exempt_pos_[zid] : zone_future_exempt_neg_[zid])
            : ((intent > 0) ? zone_future_pos_[zid] : zone_future_neg_[zid]);
        const float opp = beyond_horizon
            ? ((intent > 0) ? zone_future_exempt_neg_[zid] : zone_future_exempt_pos_[zid])
            : ((intent > 0) ? zone_future_neg_[zid] : zone_future_pos_[zid]);
        const float next_same = same + cur_weight;
        ++zone_steps;
        last_zid = zid;
        cur_weight *= decay;
        if (beyond_horizon) {
            if (next_same > local_exempt_cap) return false;
            if (gap > 0 && next_same > opp + exempt_gap) return false;
        } else {
            if (next_same >= local_cap) return false;
            if (gap > 0 && next_same > opp + static_cast<float>(gap)) return false;
        }
        return true;
    };

    for (int i = std::max(0, start_idx); i < static_cast<int>(locs.size()); ++i) {
        int tm = -1, tb = -1;
        project(locs[i], tm, tb);
        if (tm < 0) continue;
        if (tm == prev_m) {
            prev_loc = locs[i];
            prev_m = tm;
            prev_b = tb;
            continue;
        }

        const int intent = (tm > prev_m) ? +1 : -1;
        const int lo = std::min(prev_m, tm);
        const int hi = std::max(prev_m, tm);

        const std::vector<int>* cached_path =
            cached_internal_zone_path(tid, i, prev_loc, locs);
        std::vector<int> generated_path;
        if (cached_path == nullptr) {
            generated_path = shortest_zone_path(
                zone24_of_loc(prev_loc), zone24_of_loc(locs[i]), false);
            cached_path = &generated_path;
        }
        const auto& zpath = *cached_path;
        if (!zpath.empty()) {
            for (int zid : zpath) {
                if (!zid_allows(zid, intent)) return false;
            }
        } else {
            if (intent > 0) {
                for (const auto& zr : zone_ranges_) {
                    if (zr.min_m > hi || zr.max_m < lo) continue;
                    if (!zid_allows(zr.zone_id, intent)) return false;
                }
            } else {
                for (auto it = zone_ranges_.rbegin(); it != zone_ranges_.rend(); ++it) {
                    if (it->min_m > hi || it->max_m < lo) continue;
                    if (!zid_allows(it->zone_id, intent)) return false;
                }
            }
        }

        prev_loc = locs[i];
        prev_m = tm;
        prev_b = tb;
    }
    return true;
}

float GameBalanceOptScheduler::directional_future_flow_soft_penalty(
    int agent_loc, const std::vector<int>& locs, int start_idx, int tid) const {
    if (!game_directional_flow_enabled() || !ready_ || locs.empty()) return 0.0f;
    const int nz = n24c_ * n24r_;
    if ((int)zone_future_pos_.size() != nz || (int)zone_future_neg_.size() != nz)
        return 0.0f;

    int prev_m = -1, prev_b = -1;
    project(agent_loc, prev_m, prev_b);
    if (prev_m < 0) return 0.0f;

    float decay = cfg().game_dir_flow_decay;
    if (!std::isfinite(decay)) decay = GAME_DIR_FLOW_DECAY;
    decay = std::max(0.0f, std::min(1.0f, decay));
    float cur_weight = 1.0f;
    const int horizon = std::max(0, cfg().game_dir_flow_zone_horizon);
    int zone_steps = 0;
    int last_zid = -1;
    float penalty = 0.0f;
    int prev_loc = agent_loc;

    auto add_zid_penalty = [&](int zid, int intent) -> bool {
        if (zid < 0 || zid >= nz || zid == last_zid) return true;
        if (horizon > 0 && zone_steps >= horizon) return false;
        const float same = (intent > 0) ? zone_future_pos_[zid] : zone_future_neg_[zid];
        const float local_cap = directional_zone_cap(zid, intent);
        if (local_cap <= 0.0f) return false;
        const float ratio = (same + cur_weight) / local_cap;
        penalty += future_flow_soft_penalty_from_ratio(ratio);
        ++zone_steps;
        last_zid = zid;
        cur_weight *= decay;
        return true;
    };

    for (int i = std::max(0, start_idx); i < static_cast<int>(locs.size()); ++i) {
        int tm = -1, tb = -1;
        project(locs[i], tm, tb);
        if (tm < 0) continue;
        if (tm == prev_m) {
            prev_loc = locs[i];
            prev_m = tm;
            prev_b = tb;
            continue;
        }

        const int intent = (tm > prev_m) ? +1 : -1;
        const int lo = std::min(prev_m, tm);
        const int hi = std::max(prev_m, tm);

        const std::vector<int>* cached_path =
            cached_internal_zone_path(tid, i, prev_loc, locs);
        std::vector<int> generated_path;
        if (cached_path == nullptr) {
            generated_path = shortest_zone_path(
                zone24_of_loc(prev_loc), zone24_of_loc(locs[i]), false);
            cached_path = &generated_path;
        }
        const auto& zpath = *cached_path;
        if (!zpath.empty()) {
            for (int zid : zpath) {
                if (!add_zid_penalty(zid, intent)) return penalty;
            }
        } else {
            if (intent > 0) {
                for (const auto& zr : zone_ranges_) {
                    if (zr.min_m <= hi && zr.max_m >= lo)
                        if (!add_zid_penalty(zr.zone_id, intent)) return penalty;
                }
            } else {
                for (auto it = zone_ranges_.rbegin(); it != zone_ranges_.rend(); ++it) {
                    if (it->min_m <= hi && it->max_m >= lo)
                        if (!add_zid_penalty(it->zone_id, intent)) return penalty;
                }
            }
        }

        prev_loc = locs[i];
        prev_m = tm;
        prev_b = tb;
    }
    return penalty;
}

// ================================================================
// build_zone_ranges — 构建 zone→main_dist 范围映射
// ================================================================
void GameBalanceOptScheduler::build_zone_ranges() {
    std::map<int, std::pair<int,int>> zm_range;
    for (int loc : centerline_cells_) {
        if (loc < 0) continue;
        int m = main_dist_[loc];
        int r = loc / cols_, c = loc % cols_;
        int zid = (r / 24) * n24c_ + (c / 24);
        auto& p = zm_range[zid];
        // 与 GameOptAll 对齐: 用 p.first==0 判断首次赋值
        if (p.first == 0 || m < p.first) p.first = m;
        if (m > p.second) p.second = m;
    }

    zone_ranges_.clear();
    for (auto& [zid, range] : zm_range) {
        int lock = (zid >= 0 && zid < (int)zone_direction_.size()) ? zone_direction_[zid] : 0;
        zone_ranges_.push_back({zid, range.first, range.second, lock});
    }
    std::sort(zone_ranges_.begin(), zone_ranges_.end(),
        [](const ZoneMainRange& a, const ZoneMainRange& b) { return a.min_m < b.min_m; });
}

// ================================================================
// build_centerline — BFS 路径 + 墙修正 + main/branch BFS
// ================================================================
void GameBalanceOptScheduler::build_centerline(const SharedEnvironment* env) {
    rows_ = env->rows; cols_ = env->cols;
    total_ = rows_ * cols_;
    n24c_ = cols_ / 24 + 1;
    n24r_ = rows_ / 24 + 1;

    // DTW
    std::vector<int> dtw;
    compute_dtw(env, dtw);

    // 找最左/最右可行走格子
    int start_loc = -1, goal_loc = -1, best_sc = -1, best_gc = -1;
    int min_x = cols_, max_x = 0;
    for (int i = 0; i < total_; i++) {
        if (env->map[i] != 0) continue;
        int c = i % cols_;
        if (c < min_x || (c == min_x && dtw[i] > best_sc))
            { min_x = c; start_loc = i; best_sc = dtw[i]; }
        if (c > max_x || (c == max_x && dtw[i] > best_gc))
            { max_x = c; goal_loc = i; best_gc = dtw[i]; }
    }
    if (start_loc < 0 || goal_loc < 0) return;

    // BFS 最短路
    std::vector<int> came_from(total_, -1);
    std::queue<int> bfs_q;
    bfs_q.push(start_loc);
    came_from[start_loc] = start_loc;
    int found_goal = -1;
    while (!bfs_q.empty()) {
        int cur = bfs_q.front(); bfs_q.pop();
        if (cur == goal_loc) { found_goal = cur; break; }
        int neigh[4] = {cur - cols_, cur + cols_, cur - 1, cur + 1};
        for (int n : neigh)
            if (n >= 0 && n < total_ && env->map[n] == 0 && came_from[n] == -1)
                { came_from[n] = cur; bfs_q.push(n); }
    }

    std::vector<int> base_path;
    if (found_goal < 0) {
        // Fallback: 直接连起止点（极端情况）
        base_path.push_back(start_loc);
        base_path.push_back(goal_loc);
    } else {
        int cur = goal_loc;
        while (cur != start_loc) { base_path.push_back(cur); cur = came_from[cur]; }
        base_path.push_back(start_loc);
        std::reverse(base_path.begin(), base_path.end());
    }

    // 墙修正: DTW<=2 的格子 snap 到更好的 zone 中心
    std::unordered_map<int, int> corrected;
    for (size_t i = 0; i < base_path.size(); i++) {
        int loc = base_path[i];
        if (dtw[loc] > 2) continue;
        int pr = loc / cols_, pc = loc % cols_;
        std::unordered_map<int, int> zone_hits;
        for (int dr = -5; dr <= 5; dr++)
            for (int dc = -5; dc <= 5; dc++) {
                int d = std::abs(dr) + std::abs(dc);
                if (d < 1 || d > 5) continue;
                int nr = pr + dr, nc = pc + dc;
                if (nr >= 0 && nr < rows_ && nc >= 0 && nc < cols_ && env->map[nr*cols_+nc] == 0) {
                    int zid = (nr / 24) * n24c_ + (nc / 24);
                    zone_hits[zid]++;
                }
            }
        int best_zone = -1, best_cnt = -1;
        for (auto& [z, cnt] : zone_hits)
            if (cnt > best_cnt) { best_cnt = cnt; best_zone = z; }
        if (best_zone >= 0) {
            int replacement = loc, best_d = 999999;
            for (size_t j = 0; j < base_path.size(); j++) {
                int pz = (base_path[j] / cols_ / 24) * n24c_ + (base_path[j] % cols_ / 24);
                if (pz == best_zone) {
                    int d2 = std::abs(base_path[j] / cols_ - pr) + std::abs(base_path[j] % cols_ - pc);
                    if (d2 < best_d) { best_d = d2; replacement = base_path[j]; }
                }
            }
            corrected[(int)i] = replacement;
        }
    }

    // 最终中心线
    centerline_cells_.clear();
    for (size_t i = 0; i < base_path.size(); i++) {
        int cell = base_path[i];
        if (dtw[cell] <= 2 && corrected.count((int)i))
            cell = corrected[(int)i];
        if (centerline_cells_.empty() || centerline_cells_.back() != cell)
            centerline_cells_.push_back(cell);
    }

    // 沿中心线累积 main_dist
    std::unordered_map<int, int> cl_main;
    int cumul = 0;
    cl_main[centerline_cells_[0]] = cumul;
    for (size_t i = 1; i < centerline_cells_.size(); i++) {
        int r1 = centerline_cells_[i-1]/cols_, c1 = centerline_cells_[i-1]%cols_;
        int r2 = centerline_cells_[i]/cols_,   c2 = centerline_cells_[i]%cols_;
        cumul += (int)std::round(std::sqrt((double)((c2-c1)*(c2-c1) + (r2-r1)*(r2-r1))));
        cl_main[centerline_cells_[i]] = cumul;
    }
    total_len_ = cumul;

    // BFS 从中心线向外扩散 → branch_dist + main_dist
    branch_dist_.assign(total_, -1);
    main_dist_.assign(total_, -1);
    std::queue<int> seed_q;
    for (size_t i = 0; i < centerline_cells_.size(); i++) {
        int loc = centerline_cells_[i];
        branch_dist_[loc] = 0;
        main_dist_[loc] = cl_main[loc];
        seed_q.push(loc);
        // 中心线相邻点之间插值
        if (i + 1 < centerline_cells_.size()) {
            int nloc = centerline_cells_[i+1];
            int r1 = loc / cols_, c1 = loc % cols_;
            int r2 = nloc / cols_, c2 = nloc % cols_;
            int m1 = cl_main[loc], m2 = cl_main[nloc];
            for (int k = 1; k < 20; k++) {
                float t = (float)k / 20.0f;
                int ir = (int)std::round(r1 + (r2 - r1) * t);
                int ic = (int)std::round(c1 + (c2 - c1) * t);
                int iloc = ir * cols_ + ic;
                if (iloc >= 0 && iloc < total_ && env->map[iloc] == 0 && branch_dist_[iloc] == -1) {
                    branch_dist_[iloc] = 0;
                    main_dist_[iloc] = (int)std::round(m1 + (m2 - m1) * t);
                    seed_q.push(iloc);
                }
            }
        }
    }
    while (!seed_q.empty()) {
        int cur = seed_q.front(); seed_q.pop();
        int nd = branch_dist_[cur] + 1;
        int nmain = main_dist_[cur];
        int neigh[4] = {cur - cols_, cur + cols_, cur - 1, cur + 1};
        for (int n : neigh)
            if (n >= 0 && n < total_ && env->map[n] == 0 && branch_dist_[n] == -1)
                { branch_dist_[n] = nd; main_dist_[n] = nmain; seed_q.push(n); }
    }

    // zone 密度 + main 标记
    int nz = n24c_ * n24r_;
    zone24_density_.assign(nz, 0);
    zone24_has_free_.assign(nz, 0);
    zone24_valid_cells_.assign(nz, 0);
    for (int loc = 0; loc < total_; ++loc) {
        if (env->map[loc] != 0) continue;
        int zid = zone24_of_loc(loc);
        if (zid >= 0 && zid < nz) {
            zone24_has_free_[zid] = 1;
            ++zone24_valid_cells_[zid];
        }
    }
    build_zone24_distances();
    zone_is_main_.assign(nz, 0);
    for (int clloc : centerline_cells_) {
        int r = clloc / cols_, c = clloc % cols_;
        int zid = (r / 24) * n24c_ + (c / 24);
        if (zid >= 0 && zid < nz) zone_is_main_[zid] = 1;
    }

    zone_direction_.assign(nz, 0);
    build_zone_ranges();
    ready_ = true;
}

// ================================================================
// initialize
// ================================================================
void GameBalanceOptScheduler::initialize(SharedEnvironment* env, int /*preprocess_time_limit_ms*/) {
    env_ = env;
    task_static_cache_.clear();
    short_task_length_cuts_ready_ = false;
    short_task_length_cuts_.fill(0);
    short_task_length_means_.fill(1.0f);
    build_centerline(env);
    pibt_lock().reset();

    // Sync zone_n24c to PibtLock for PIBT counter-flow
    pibt_lock().zone_n24c = n24c_;

    // agent空闲计数器（IDEA4: idle exempt）
    agent_idle_ticks_.assign(env->num_of_agents, 0);
    last_idle_update_timestep_ = -1;
    reset_dynamic_zone_cap(env);
}

// ================================================================
// update_zone_density — 每 tick 更新 24×24 zone 密度，只统计有任务的 agent
// ================================================================
void GameBalanceOptScheduler::update_zone_density(const SharedEnvironment* env) {
    int nz = n24c_ * n24r_;
    if ((int)zone24_density_.size() != nz)
        zone24_density_.assign(nz, 0);
    else
        std::fill(zone24_density_.begin(), zone24_density_.end(), 0);

    for (int a = 0; a < env->num_of_agents; a++) {
        if (a >= (int)env->curr_task_schedule.size() || env->curr_task_schedule[a] < 0)
            continue;
        int loc = env->curr_states[a].location;
        if (loc < 0) continue;
        int r = loc / cols_, c = loc % cols_;
        int zi = (r / 24) * n24c_ + (c / 24);
        if (zi >= 0 && zi < nz) zone24_density_[zi]++;
    }
}

// ================================================================
// update_zone_direction — 方向流量 + 方向锁 + PIBT 同步
// ================================================================
void GameBalanceOptScheduler::update_zone_direction(const SharedEnvironment* env) {
    int nz = n24c_ * n24r_;
    if ((int)zone_direction_.size() != nz) zone_direction_.assign(nz, 0);
    if ((int)agent_last_main_.size() != env->num_of_agents) agent_last_main_.assign(env->num_of_agents, -1);

    if ((cfg().planner_mode == 16 || cfg().planner_mode == 14) &&
        cfg().game_highway_enabled) {
        std::fill(zone_direction_.begin(), zone_direction_.end(), 0);
        for (auto& zr : zone_ranges_) zr.lock_dir = 0;
        auto& lock = pibt_lock();
        std::fill(lock.seg_lock_dir, lock.seg_lock_dir + PibtLock::MAX_SEGMENTS, 0);
        std::fill(lock.zone_dir, lock.zone_dir + PibtLock::MAX_ZONES, 0);
        return;
    }

    int dir_threshold = cfg().game_dir_threshold;
    int zone_hard_cap = cfg().game_zone_hard_cap;

    std::vector<int> flow_right(nz, 0), flow_left(nz, 0);
    for (int a = 0; a < env->num_of_agents; a++) {
        if (a >= (int)env->curr_task_schedule.size() || env->curr_task_schedule[a] < 0) {
            if (a < (int)agent_last_main_.size()) agent_last_main_[a] = -1;
            continue;
        }
        int loc = env->curr_states[a].location;
        if (loc < 0) continue;
        int r = loc / cols_, c = loc % cols_;
        int zi = (r / 24) * n24c_ + (c / 24);
        if (zi < 0 || zi >= nz) continue;
        int m = -1, b;
        project(loc, m, b);
        int prev_m = agent_last_main_[a];
        agent_last_main_[a] = m;
        if (prev_m >= 0 && m >= 0) {
            if (m - prev_m >= 1)      flow_right[zi]++;
            else if (m - prev_m <= -1) flow_left[zi]++;
        }
    }

    // 多数方向
    std::fill(zone_direction_.begin(), zone_direction_.end(), 0);
    for (int zi = 0; zi < nz; zi++) {
        if (zone24_density_[zi] < dir_threshold) continue;
        zone_direction_[zi] = (flow_right[zi] > flow_left[zi]) ? +1 :
                              (flow_left[zi] > flow_right[zi]) ? -1 : 0;
    }

    // Phase 1: 热区设置方向（从邻居继承）
    for (int i = 0; i < (int)zone_ranges_.size(); i++) {
        int zid = zone_ranges_[i].zone_id;
        if (zid < 0 || zid >= nz || zone24_density_[zid] < zone_hard_cap)
            continue;
        if (zone_direction_[zid] != 0) continue;
        int neighbor_dir = 0;
        for (int d = -CONTAIN_EXPAND; d <= CONTAIN_EXPAND; d++) {
            int ni = i + d;
            if (ni < 0 || ni >= (int)zone_ranges_.size()) continue;
            int nzid = zone_ranges_[ni].zone_id;
            if (nzid >= 0 && zone_direction_[nzid] != 0) {
                neighbor_dir = zone_direction_[nzid];
                break;
            }
        }
        if (neighbor_dir != 0) {
            zone_direction_[zid] = neighbor_dir;
        } else {
            zone_direction_[zid] = (zid * 1103515245 + 12345) % 2 ? +1 : -1;
        }
    }

    // Phase 2: 热区锁定自身 + 邻居（宽度感知容量控制）
    for (int i = 0; i < (int)zone_ranges_.size(); i++) {
        int zid = zone_ranges_[i].zone_id;
        if (zid < 0 || zid >= nz || zone24_density_[zid] < zone_hard_cap) continue;

        int dir = zone_direction_[zid];
        for (int d = -CONTAIN_EXPAND; d <= CONTAIN_EXPAND; d++) {
            int ni = i + d;
            if (ni < 0 || ni >= (int)zone_ranges_.size()) continue;
            int nzid = zone_ranges_[ni].zone_id;
            if (nzid >= 0) zone_direction_[nzid] = dir;
        }
    }

    // Phase 3: 冷区解锁
    for (int i = 0; i < (int)zone_ranges_.size(); i++) {
        int zid = zone_ranges_[i].zone_id;
        if (zid < 0 || zid >= nz) continue;
        if (zone24_density_[zid] >= zone_hard_cap) continue;
        bool nearby_hot = false;
        for (int d = -CONTAIN_EXPAND; d <= CONTAIN_EXPAND; d++) {
            int ni = i + d;
            if (ni < 0 || ni >= (int)zone_ranges_.size()) continue;
            int nzid = zone_ranges_[ni].zone_id;
            if (nzid >= 0 && zone24_density_[nzid] >= zone_hard_cap) {
                nearby_hot = true; break;
            }
        }
        if (!nearby_hot) zone_direction_[zid] = 0;
    }

    // 同步到 zone_ranges_
    for (auto& zr : zone_ranges_) {
        if (zr.zone_id >= 0 && zr.zone_id < nz)
            zr.lock_dir = zone_direction_[zr.zone_id];
    }

    // 同步到 PibtLock
    auto& lock = pibt_lock();
    for (size_t si = 0; si < zone_ranges_.size() && si < (size_t)PibtLock::MAX_SEGMENTS; si++)
        lock.seg_lock_dir[si] = zone_ranges_[si].lock_dir;

    // PIBT counter-flow: sync zone directions
    if (cfg().game_pibt_counter_flow) {
        for (int zi = 0; zi < nz && zi < PibtLock::MAX_ZONES; zi++)
            lock.zone_dir[zi] = zone_direction_[zi];
    }
}

// ================================================================
// connection_exempt_score — Layer3 逃逸豁免: Bresenham 路径检查
// ================================================================
float GameBalanceOptScheduler::connection_exempt_score(int agent_loc,
        const std::vector<int>& task_locs) const {
    int max_z = (int)zone24_density_.size();
    int ax = agent_loc % cols_, ay = agent_loc / cols_;
    int az = (ay / 24) * n24c_ + (ax / 24);
    if (az < 0 || az >= max_z || task_locs.empty()) return INVALID_SCORE;

    int ex_main_cap    = cfg().game_ex_main_cap;
    int ex_nonmain_cap = cfg().game_ex_nonmain_cap;

    if (zone24_density_[az] >= ex_main_cap) return INVALID_SCORE;

    float total_score = 0.0f;
    for (int loc : task_locs) {
        int tx = loc % cols_, ty = loc / cols_;
        int cx = ax, cy = ay;
        int dx = std::abs(tx - ax), sx = ax < tx ? 1 : -1;
        int dy = -std::abs(ty - ay), sy = ay < ty ? 1 : -1;
        int err = dx + dy;
        while (true) {
            if (env_->map[cy * cols_ + cx] != 0) return INVALID_SCORE;
            int zc = (cy / 24) * n24c_ + (cx / 24);
            if (zc != az && zc >= 0 && zc < max_z) {
                int d = zone24_density_[zc];
                if (zone_is_main_[zc]) {
                    if (d >= ex_main_cap) return INVALID_SCORE;
                } else {
                    if (d >= ex_nonmain_cap) return INVALID_SCORE;
                }
            }
            if (cx == tx && cy == ty) break;
            int e2 = 2 * err;
            if (e2 >= dy) { err += dy; cx += sx; }
            if (e2 <= dx) { err += dx; cy += sy; }
        }
        float dist = score_distance_between(ay * cols_ + ax, loc);
        if (dist < 0.0f) return INVALID_SCORE;
        total_score += distance_score(dist);
        ax = tx; ay = ty;
    }
    return total_score;
}

// ================================================================
// is_hot_zone_exempt — agent 所在 zone 密度 >= hard_cap
// ================================================================
bool GameBalanceOptScheduler::is_hot_zone_exempt(int loc) const {
    if (loc < 0) return false;
    int zi = (loc / cols_ / 24) * n24c_ + (loc % cols_ / 24);
    int zone_hard_cap = cfg().game_zone_hard_cap;
    return (zi >= 0 && zi < (int)zone24_density_.size()
            && zone24_density_[zi] >= zone_hard_cap);
}

// ================================================================
// base_score — 三层豁免打分
//   Layer1: main zone 豁免（沿路所有 main zone 密度 < exempt_main → 直通）
//   Layer2: branch task（所有 errand 不穿热区 + Bresenham 无障碍 → 直通）
//   Layer3: 逃逸豁免（connection_exempt_score）
//   Default: zone 密度 + 方向锁惩罚
// ================================================================
float GameBalanceOptScheduler::base_score(int a_loc, int tid, int step, int start_idx) const {
    (void)step;
    auto it = env_->task_pool.find(tid);
    if (it == env_->task_pool.end() || it->second.locations.empty())
        return INVALID_SCORE;
    if (!ready_) return 1.0f;
    auto& locs = it->second.locations;
    if (task_has_channel_errand(locs)) return INVALID_SCORE;
    if (!directional_future_flow_allows(a_loc, locs, start_idx, tid)) return INVALID_SCORE;

    int zone_hard_cap = cfg().game_zone_hard_cap;
    int exempt_main   = cfg().game_exempt_main;
    int density_cap   = cfg().game_density_cap;
    int route_penalty  = cfg().game_route_penalty;

    if (cfg().game_score_exempt_enabled) {
        // ---- Layer 1: main zone 豁免 ----
        {
            int a_m, a_b;
            project(a_loc, a_m, a_b);
            if (a_m >= 0) {
                bool exempt = true;
                int prev_m = a_m;
                for (auto& loc : locs) {
                    int tm, tb;
                    project(loc, tm, tb);
                    if (tm < 0) continue;
                    int lo = std::min(prev_m, tm);
                    int hi = std::max(prev_m, tm);
                    for (auto& zr : zone_ranges_) {
                        if (zr.min_m <= hi && zr.max_m >= lo &&
                            zone_is_main_[zr.zone_id] &&
                            zone24_density_[zr.zone_id] >= exempt_main) {
                            exempt = false; break;
                        }
                    }
                    if (!exempt) break;
                    prev_m = tm;
                }
                if (exempt) {
                    float total = 0;
                    int prev = a_loc;
                    for (auto& loc : locs) {
                        float dist = score_distance_between(prev, loc);
                        if (dist < 0.0f) return INVALID_SCORE;
                        total += distance_score(dist);
                        prev = loc;
                    }
                    return total;
                }
            }
        }

        // ---- Layer 2: branch task 检查 ----
        {
            bool is_branch = true;
            int check_prev = a_loc;
            int max_z = (int)zone_is_main_.size();
            for (size_t ei = (size_t)start_idx; ei < locs.size(); ei++) {
                int loc = locs[ei];
                int x1 = check_prev % cols_, y1 = check_prev / cols_;
                int x2 = loc % cols_,        y2 = loc / cols_;
                // 起点 zone 检查
                int z1 = (y1 / 24) * n24c_ + (x1 / 24);
                if (z1 >= 0 && z1 < max_z) {
                    int d1 = zone24_density_[z1];
                    if (zone_is_main_[z1]) {
                        if (d1 >= zone_hard_cap) { is_branch = false; break; }
                    } else {
                        if (d1 >= 70) { is_branch = false; break; }
                    }
                }
                // 终点 zone 检查
                int z2 = (y2 / 24) * n24c_ + (x2 / 24);
                if (z2 >= 0 && z2 < max_z) {
                    int d2 = zone24_density_[z2];
                    if (zone_is_main_[z2]) {
                        if (d2 >= zone_hard_cap) { is_branch = false; break; }
                    } else {
                        if (d2 >= 70) { is_branch = false; break; }
                    }
                }
                // Bresenham 无障碍检查
                int dx = abs(x2 - x1), sx = x1 < x2 ? 1 : -1;
                int dy = -abs(y2 - y1), sy = y1 < y2 ? 1 : -1;
                int err = dx + dy;
                int cx = x1, cy = y1;
                while (true) {
                    if (env_->map[cy * cols_ + cx] != 0) { is_branch = false; break; }
                    if (cx == x2 && cy == y2) break;
                    int e2 = 2 * err;
                    if (e2 >= dy) { err += dy; cx += sx; }
                    if (e2 <= dx) { err += dx; cy += sy; }
                }
                if (!is_branch) break;
                check_prev = loc;
            }

            if (is_branch) {
                float total = 0;
                int prev = a_loc;
                for (size_t ei = (size_t)start_idx; ei < locs.size(); ei++) {
                    int r2 = locs[ei] / cols_, c2 = locs[ei] % cols_;
                    int z2 = (r2 / 24) * n24c_ + (c2 / 24);
                    if (z2 >= 0 && z2 < max_z && zone24_density_[z2] >= 70)
                        return INVALID_SCORE;
                    float dist = score_distance_between(prev, locs[ei]);
                    if (dist < 0.0f) return INVALID_SCORE;
                    total += distance_score(dist);
                    prev = locs[ei];
                }
                return total;
            }

            // ---- Layer 3: 逃逸豁免 ----
            if (start_idx == 0) {
                float esc = connection_exempt_score(a_loc, locs);
                if (is_valid_score(esc)) return esc;
            } else {
                std::vector<int> remaining(locs.begin() + start_idx, locs.end());
                float esc = connection_exempt_score(a_loc, remaining);
                if (is_valid_score(esc)) return esc;
            }
        }
    }

    // ---- Default: zone 密度 + 方向锁 打分 ----
    int a_m, a_b;
    project(a_loc, a_m, a_b);
    if (a_m < 0) return 1.0f;

    if (use_direct_distance_score() && start_idx >= 0 &&
        start_idx < static_cast<int>(locs.size())) {
        auto cached = task_static_cache_.find(tid);
        if (cached != task_static_cache_.end() &&
            ((cached->second.distance_ready && cached->second.distance_valid &&
              start_idx < static_cast<int>(cached->second.tail_distance.size())) ||
             (!cached->second.distance_ready && cached->second.coarse_distance_valid &&
              start_idx < static_cast<int>(cached->second.coarse_tail_distance.size())))) {
            const float pickup_dist = score_distance_between(a_loc, locs[start_idx]);
            if (pickup_dist < 0.0f) return INVALID_SCORE;
            float total = distance_score(pickup_dist) *
                std::max(0.0f, cfg().game_pickup_weight);
            const float internal_dist = cached->second.distance_ready
                ? cached->second.tail_distance[start_idx]
                : cached->second.coarse_tail_distance[start_idx];
            total += distance_score(internal_dist);
            total -= directional_future_flow_soft_penalty(
                a_loc, locs, start_idx, tid);
            return total;
        }
    }

    float total_score = 0;
    int prev_m = a_m, prev_b = a_b;
    int prev_loc = a_loc;

    for (size_t ei = (size_t)start_idx; ei < locs.size(); ei++) {
        int tm, tb;
        project(locs[ei], tm, tb);
        if (tm < 0) continue;

        int lo = std::min(prev_m, tm);
        int hi = std::max(prev_m, tm);
        int intent = (tm > prev_m) ? +1 : -1;

        int d_max_seg = 0;
        if (!use_direct_distance_score()) {
            for (auto& zr : zone_ranges_) {
                if (zr.min_m <= hi && zr.max_m >= lo) {
                    int d = zone24_density_[zr.zone_id];
                    // 方向锁: 逆流阻断
                    if (zr.lock_dir != 0 && zr.lock_dir != intent) {
                        return INVALID_SCORE;
                    }
                    // 密度上限: 极端拥塞阻断
                    if (d >= density_cap) {
                        return INVALID_SCORE;
                    }
                    if (d > d_max_seg) d_max_seg = d;
                }
            }
        }

        float seg_dist = score_distance_between(prev_loc, locs[ei]);
        if (seg_dist < 0.0f) return INVALID_SCORE;
        float seg_score = distance_score(seg_dist);
        if (use_direct_distance_score() && ei == (size_t)start_idx) {
            seg_score *= std::max(0.0f, cfg().game_pickup_weight);
        }

        if (d_max_seg > route_penalty) {
            int idx = std::min(d_max_seg, route_penalty + 20) - route_penalty;
            seg_score = apply_route_density_penalty(seg_score, DIVISOR_TABLE[idx]);
        }

        total_score += seg_score;
        prev_loc = locs[ei];
        prev_m = tm; prev_b = tb;
    }

    if (use_direct_distance_score()) {
        total_score -= directional_future_flow_soft_penalty(a_loc, locs, start_idx, tid);
    }
    return total_score;
}
// ================================================================
void GameBalanceOptScheduler::schedule(int time_limit,
                                       std::vector<int>& proposed_schedule,
                                       SharedEnvironment* env) {
    auto t0 = std::chrono::steady_clock::now();
    auto deadline = t0 + std::chrono::milliseconds(time_limit);
    env_ = env;

    int n_agents = env->num_of_agents;
    if (n_agents <= 0 || env->curr_states.empty() ||
        (int)env->curr_states.size() < n_agents ||
        (int)proposed_schedule.size() < n_agents) return;

    // ---- 每 tick 更新 ----
    update_zone_density(env);
    update_zone_direction(env);
    seed_directional_future_flow(env);
    update_dynamic_zone_cap(env);

    // ---- PibtLock 同步 ----
    if ((int)parked_task_.size() != n_agents)
        parked_task_.assign(n_agents, -1);
    if ((int)agent_idle_ticks_.size() != n_agents)
        agent_idle_ticks_.assign(n_agents, 0);

    auto& lock = pibt_lock();
    for (int a = 0; a < n_agents && a < PibtLock::MAX_AGENTS; a++) {
        lock.parked_task[a] = parked_task_[a];
        lock.agent_blocked[a] = 0;
        lock.agent_priority_penalty[a] = 0.0f;
    }

    int step = step_++;
    const int diag_interval = std::max(1, cfg().game_sched_diag_interval);
    const bool diag_on = cfg().game_sched_diag && (step % diag_interval == 0);
    struct ScheduleDiag {
        int step = 0;
        int busy_agents = 0;
        int free_agents_initial = 0;
        int free_agents_after_phase1 = 0;
        int max_total_busy = 0;
        int assign_budget = 0;
        int search_radius = 0;
        int assigned_count = 0;
        int budget_stop = 0;
        int timeout_stop = 0;
        int considered_agents = 0;
        int no_candidates_in_radius = 0;
        int no_valid_candidates = 0;
        int neighbor_checked_cap_hits = 0;
        int channel_free_tasks = 0;
        int prefilter_task_total = 0;
        int prefilter_task_kept = 0;
        int prefilter_reject_missing = 0;
        int prefilter_reject_channel = 0;
        int prefilter_reject_capacity = 0;
        int prefilter_reject_flow = 0;
        int prefilter_reject_lock = 0;
        int prefilter_reject_density = 0;
        int prefilter_reject_branch = 0;
        int prefilter_reject_other = 0;
        int length_gate_active = 0;
        int length_gate_input = 0;
        int length_gate_kept = 0;
        int length_gate_short_total[2] = {0, 0};
        int length_gate_short_active[2] = {0, 0};
        int length_gate_short_quota[2] = {0, 0};
        int length_gate_short_kept[2] = {0, 0};
        int length_gate_cut[4] = {0, 0, 0, 0};
        int length_gate_final_active[5] = {0, 0, 0, 0, 0};
        int legal_agents_after_task_filter = 0;
        int legal_agents_after_full_filter = 0;
        int region_mode = 0;
        long long region_checks = 0;
        long long region_cache_hits = 0;
        long long region_cache_misses = 0;
        long long region_reject_flow = 0;
        long long region_task_scans = 0;
        long long region_full_checks = 0;
        long long region_batch_flushes = 0;
        size_t free_tasks = 0;
        size_t task_zones = 0;
        long long attempts = 0;
        long long reject_missing = 0;
        long long reject_channel = 0;
        long long reject_flow = 0;
        long long reject_zone_or_route = 0;
        int reassign_replace_attempts = 0;
        int reassign_replace_accepts = 0;
        int reassign_attempts = 0;
        int reassign_accepts = 0;
        int idle_global_eligible = 0;
        int idle_global_assigned = 0;
    } diag;
    if (diag_on) diag.step = step;
    set_game_prefilter_stats({});

    const int eff_density_cap = cfg().game_density_cap;

    // ---- 分类 agent ----
    std::vector<int> free_agents;
    std::unordered_set<int> assigned_tasks;

    const int curr_timestep = std::max(0,
        std::max(env->system_timestep, env->curr_timestep));
    const int idle_tick_delta = last_idle_update_timestep_ < 0
        ? 0 : std::max(0, curr_timestep - last_idle_update_timestep_);
    last_idle_update_timestep_ = curr_timestep;
    for (int a = 0; a < n_agents; a++) {
        if (env->curr_task_schedule[a] < 0) {
            free_agents.push_back(a);
            agent_idle_ticks_[a] += idle_tick_delta;
        } else {
            assigned_tasks.insert(env->curr_task_schedule[a]);
            agent_idle_ticks_[a] = 0;
        }
    }
    if (diag_on) diag.free_agents_initial = (int)free_agents.size();

    // ---- 收集 free tasks ----
    std::unordered_set<int> free_tasks_set;
    for (int tid : env->new_tasks) free_tasks_set.insert(tid);
    for (auto& [tid, task] : env->task_pool) {
        if (!assigned_tasks.count(tid)) free_tasks_set.insert(tid);
    }
    if (diag_on) {
        diag.free_tasks = free_tasks_set.size();
        for (int tid : free_tasks_set) {
            auto tit = env->task_pool.find(tid);
            if (tit != env->task_pool.end() && task_has_channel_errand(tit->second.locations))
                diag.channel_free_tasks++;
        }
    }

    // ---- 过滤 goal 在 forbidden 格子上的任务 ----
    for (auto it = free_tasks_set.begin(); it != free_tasks_set.end(); ) {
        auto tit = env->task_pool.find(*it);
        if (tit != env->task_pool.end() && !tit->second.locations.empty()) {
            bool skip = false;
            for (int loc : tit->second.locations) {
                if (ORZ_Highway::is_task_forbidden(loc)) { skip = true; break; }
            }
            if (skip) {
                it = free_tasks_set.erase(it);
                continue;
            }
        }
        ++it;
    }

    // Tasks are unavailable during initialize. Cache their immutable geometry
    // incrementally within a small slice of the online schedule budget.
    std::vector<int> uncached_tasks;
    uncached_tasks.reserve(free_tasks_set.size());
    for (int tid : free_tasks_set) {
        auto cached = task_static_cache_.find(tid);
        if (cached == task_static_cache_.end() ||
            !cached->second.distance_ready || !cached->second.zone_paths_ready)
            uncached_tasks.push_back(tid);
    }
    if (!uncached_tasks.empty()) {
        const int cache_ms = std::max(1, std::min(50, time_limit / 20));
        const auto cache_deadline = std::min(
            deadline, std::chrono::steady_clock::now() +
                std::chrono::milliseconds(cache_ms));
        // Random reads from the 17+ GB BFS matrix are bandwidth-bound. A
        // single worker avoids page thrashing and respects the short budget.
        prewarm_task_static_cache(uncached_tasks, cache_deadline, 1);
    }

    // ---- Phase 1: 多 errand 续接 + parked task 处理 ----
    for (auto it = free_agents.begin(); it != free_agents.end(); ) {
        int a = *it;

        // 检查 parked task
        int parked = (a < (int)parked_task_.size()) ? parked_task_[a] : -1;
        if (parked >= 0) {
            int a_loc = env->curr_states[a].location;
            float bs = base_score(a_loc, parked, step, 2);
            if (is_valid_score(bs)) {
                proposed_schedule[a] = parked;
                add_directional_future_flow_for_task(a_loc, parked, 2);
                parked_task_[a] = -1;
                free_tasks_set.erase(parked);
                it = free_agents.erase(it);
                agent_idle_ticks_[a] = 0;
            } else {
                proposed_schedule[a] = -1;
                if (!is_hot_zone_exempt(a_loc)) {
                    lock.agent_blocked[a] = 1;
                }
                ++it;
            }
            continue;
        }

        // 多 errand 续接
        int cur = env->curr_task_schedule[a];
        if (cur >= 0 && env->task_pool.count(cur) &&
            env->task_pool.at(cur).locations.size() > 1) {
            proposed_schedule[a] = cur;
            free_tasks_set.erase(cur);
            it = free_agents.erase(it);
            agent_idle_ticks_[a] = 0;
        } else {
            ++it;
        }
    }

    if (free_agents.empty()) {
        if (diag_on) {
            std::cerr << "[GameSchedDiag] step=" << diag.step
                      << " free_initial=" << diag.free_agents_initial
                      << " free_after_phase1=0 assigned=0"
                      << " free_tasks=" << diag.free_tasks
                      << " channel_free_tasks=" << diag.channel_free_tasks
                      << " dyn_cap=" << (cfg().game_dir_flow_dynamic_cap ? 1 : 0)
                      << " cap_window=" << dynamic_cap_last_window_ticks_
                      << " cap_avg=" << dynamic_cap_last_avg_
                      << " cap_min=" << dynamic_cap_last_min_
                      << " cap_max=" << dynamic_cap_last_max_
                      << " stall_avg=" << dynamic_cap_last_stall_avg_
                      << " stall_max=" << dynamic_cap_last_stall_max_
                      << " stall_busy=" << dynamic_cap_last_busy_stall_avg_
                      << " stall_free=" << dynamic_cap_last_free_stall_avg_
                      << " cap_active=" << dynamic_cap_last_active_
                      << " cap_lo_hi=" << dynamic_cap_last_below_base_
                      << "/" << dynamic_cap_last_above_base_
                      << " cap_at_min=" << dynamic_cap_last_at_min_
                      << " cap_up=" << dynamic_cap_last_up_
                      << " cap_down=" << dynamic_cap_last_down_
                      << " note=no_free_agents\n";
        }
        return;
    }
    if (diag_on) diag.free_agents_after_phase1 = (int)free_agents.size();

    // ---- Phase 2: Zone-based 贪心分配 ----
    int busy_agents = n_agents - (int)free_agents.size();
    float bal_ratio = cfg().game_bal_assign_ratio;
    int max_total_busy = (bal_ratio < 0.999f)
        ? std::max(1, (int)(bal_ratio * (float)n_agents))
        : n_agents;
    int assign_budget = std::max(0, max_total_busy - busy_agents);
    if (diag_on) {
        diag.busy_agents = busy_agents;
        diag.max_total_busy = max_total_busy;
        diag.assign_budget = assign_budget;
    }

    const int search_radius = cfg().game_search_radius;
    if (diag_on) diag.search_radius = search_radius;
    if (diag_on) diag.region_mode = cfg().game_region_match ? 1 : 0;

    // 先过滤掉与 agent 无关就必然非法的任务，避免它们占据每个 zone 的扫描名额。
    std::unordered_set<int> filtered_free_tasks_set;
    diag.prefilter_task_total = (int)free_tasks_set.size();
    for (int tid : free_tasks_set) {
        auto tit = env->task_pool.find(tid);
        if (tit == env->task_pool.end() || tit->second.locations.empty()) {
            diag.prefilter_reject_missing++;
            continue;
        }
        const auto& locs = tit->second.locations;
        if (task_has_channel_errand(locs)) {
            diag.prefilter_reject_channel++;
            continue;
        }
        // Global prefilter only checks task-internal zone capacity. Concrete
        // distances and directed reachability belong to the shortlisted
        // agent-task validation below.
        if (cfg().game_prefilter_capacity && locs.size() > 1 &&
            !directional_future_flow_allows(
                locs.front(), locs, 1, tid)) {
            diag.prefilter_reject_flow++;
            diag.prefilter_reject_capacity++;
            continue;
        }
        filtered_free_tasks_set.insert(tid);
    }
    diag.prefilter_task_kept = (int)filtered_free_tasks_set.size();

    // Length pacing experiment: split all currently revealed tasks into five
    // equal-count quantiles. In each of the two shortest bins, running tasks
    // consume quota first and only the remaining quota is opened at random.
    // The three longer bins remain unrestricted.
    const int length_gate_open_tick = std::max(0,
        cfg().game_short_task_test_ticks -
        std::max(0, cfg().game_short_task_final_open_ticks));
    const bool length_gate_active = cfg().game_short_task_throttle &&
        curr_timestep < length_gate_open_tick &&
        filtered_free_tasks_set.size() >= 5;
    if (length_gate_active) {
        auto task_length = [&](const Task& task) {
            const auto& locs = task.locations;
            int length = 0;
            for (std::size_t i = 1; i < locs.size(); ++i) {
                const int a = locs[i - 1];
                const int b = locs[i];
                length += std::abs(a / cols_ - b / cols_) +
                          std::abs(a % cols_ - b % cols_);
            }
            return length;
        };
        if (!short_task_length_cuts_ready_) {
            std::vector<int> initial_lengths;
            initial_lengths.reserve(env->task_pool.size());
            for (const auto& [tid, task] : env->task_pool) {
                (void)tid;
                if (!task.locations.empty()) initial_lengths.push_back(task_length(task));
            }
            std::sort(initial_lengths.begin(), initial_lengths.end());
            if (initial_lengths.size() >= 5) {
                for (int i = 0; i < 4; ++i) {
                    const std::size_t pos = std::min(
                        initial_lengths.size() - 1,
                        initial_lengths.size() * static_cast<std::size_t>(i + 1) / 5 - 1);
                    short_task_length_cuts_[i] = initial_lengths[pos];
                }
                std::array<long long, 5> sums{{0, 0, 0, 0, 0}};
                std::array<int, 5> counts{{0, 0, 0, 0, 0}};
                for (int length : initial_lengths) {
                    int bin = 0;
                    while (bin < 4 && length > short_task_length_cuts_[bin]) ++bin;
                    sums[bin] += length;
                    ++counts[bin];
                }
                for (int bin = 0; bin < 5; ++bin) {
                    short_task_length_means_[bin] = counts[bin] > 0
                        ? static_cast<float>(sums[bin]) / counts[bin] : 1.0f;
                }
                short_task_length_cuts_ready_ = true;
            }
        }

        std::vector<int> bins[5];
        std::unordered_map<int, unsigned char> task_length_bin;
        task_length_bin.reserve(env->task_pool.size());
        for (const auto& [tid, task] : env->task_pool) {
            if (task.locations.empty()) continue;
            const int length = task_length(task);
            int bin = 0;
            while (bin < 4 && length > short_task_length_cuts_[bin]) ++bin;
            bins[bin].push_back(tid);
            task_length_bin[tid] = static_cast<unsigned char>(bin);
        }
        for (int i = 0; i < 4; ++i)
            diag.length_gate_cut[i] = short_task_length_cuts_[i];

        std::unordered_set<int> gated_tasks;
        gated_tasks.reserve(filtered_free_tasks_set.size());
        float release_ratio = cfg().game_short_task_release_ratio;
        if (!std::isfinite(release_ratio)) release_ratio = 0.20f;
        release_ratio = std::clamp(release_ratio, 0.0f, 0.50f);
        const float other_ratio = std::max(0.0f,
            (1.0f - 2.0f * release_ratio) / 3.0f);
        float weighted_mean = 0.0f;
        for (int bin = 0; bin < 5; ++bin) {
            const float completion_share = bin < 2 ? release_ratio : other_ratio;
            weighted_mean += completion_share * short_task_length_means_[bin];
        }
        weighted_mean = std::max(1e-6f, weighted_mean);
        for (int tid : assigned_tasks) {
            auto bit = task_length_bin.find(tid);
            if (bit != task_length_bin.end() && bit->second < 2)
                ++diag.length_gate_short_active[bit->second];
        }
        for (int bin = 0; bin < 5; ++bin) {
            auto& tids = bins[bin];
            if (bin < 2) {
                diag.length_gate_short_total[bin] = static_cast<int>(tids.size());
                const float active_share =
                    release_ratio * short_task_length_means_[bin] / weighted_mean;
                const int quota = std::min<int>(tids.size(),
                    static_cast<int>(std::ceil(max_total_busy * active_share)));
                diag.length_gate_short_quota[bin] = quota;
                const int open_budget = std::max(
                    0, quota - diag.length_gate_short_active[bin]);
                std::vector<int> legal_free;
                legal_free.reserve(tids.size());
                for (int tid : tids) {
                    if (filtered_free_tasks_set.count(tid)) legal_free.push_back(tid);
                }
                std::shuffle(legal_free.begin(), legal_free.end(), short_task_rng_);
                const int keep = std::min<int>(open_budget, legal_free.size());
                diag.length_gate_short_kept[bin] = keep;
                for (int i = 0; i < keep; ++i) gated_tasks.insert(legal_free[i]);
            } else {
                for (int tid : tids) {
                    if (filtered_free_tasks_set.count(tid)) gated_tasks.insert(tid);
                }
            }
        }
        diag.length_gate_active = 1;
        diag.length_gate_input = static_cast<int>(filtered_free_tasks_set.size());
        diag.length_gate_kept = static_cast<int>(gated_tasks.size());
        filtered_free_tasks_set.swap(gated_tasks);
    }
    int blocked_zone_count = 0;
    std::ostringstream blocked_zone_ids;
    if (game_directional_flow_enabled()) {
        const int nz = n24c_ * n24r_;
        const int base_cap = std::max(0, cfg().game_dir_flow_cap);
        const int gap = std::max(0, cfg().game_dir_flow_gap);
        float exempt_ratio = cfg().game_dir_flow_horizon_exempt_ratio;
        if (!std::isfinite(exempt_ratio))
            exempt_ratio = GAME_DIR_FLOW_HORIZON_EXEMPT_RATIO;
        exempt_ratio = std::max(0.0f, exempt_ratio);
        const float exempt_gap = static_cast<float>(gap) * exempt_ratio;
        if (base_cap > 0 &&
            (int)zone_future_pos_.size() == nz &&
            (int)zone_future_neg_.size() == nz &&
            (int)zone_future_exempt_pos_.size() == nz &&
            (int)zone_future_exempt_neg_.size() == nz) {
            for (int zid = 0; zid < nz; ++zid) {
                const float pos = zone_future_pos_[zid];
                const float neg = zone_future_neg_[zid];
                const float exempt_pos = zone_future_exempt_pos_[zid];
                const float exempt_neg = zone_future_exempt_neg_[zid];
                const float pos_cap = directional_zone_cap(zid, +1);
                const float neg_cap = directional_zone_cap(zid, -1);
                const float exempt_pos_cap = pos_cap * exempt_ratio;
                const float exempt_neg_cap = neg_cap * exempt_ratio;
                const bool pos_blocked = (pos + 1.0f >= pos_cap) ||
                    (gap > 0 && pos + 1.0f > neg + (float)gap);
                const bool neg_blocked = (neg + 1.0f >= neg_cap) ||
                    (gap > 0 && neg + 1.0f > pos + (float)gap);
                const bool exempt_pos_blocked =
                    exempt_pos + 1.0f > exempt_pos_cap ||
                    (gap > 0 && exempt_pos + 1.0f > exempt_neg + exempt_gap);
                const bool exempt_neg_blocked =
                    exempt_neg + 1.0f > exempt_neg_cap ||
                    (gap > 0 && exempt_neg + 1.0f > exempt_pos + exempt_gap);
                if (pos_blocked || neg_blocked ||
                    exempt_pos_blocked || exempt_neg_blocked) {
                    if (blocked_zone_count > 0) blocked_zone_ids << ",";
                    blocked_zone_ids << zid;
                    ++blocked_zone_count;
                }
            }
        }
    }
    set_game_prefilter_stats({
        diag.prefilter_task_total,
        diag.prefilter_task_kept,
        diag.prefilter_task_total - diag.prefilter_task_kept,
        diag.prefilter_reject_missing,
        diag.prefilter_reject_channel,
        diag.prefilter_reject_capacity,
        diag.prefilter_reject_flow,
        diag.prefilter_reject_lock,
        diag.prefilter_reject_density,
        diag.prefilter_reject_branch,
        diag.prefilter_reject_other,
        blocked_zone_count,
        blocked_zone_ids.str(),
        0,
        0,
        0,
        0
    });

    // 按 zone 分桶 free tasks
    std::unordered_map<int, std::vector<int>> tasks_by_zone;
    for (int tid : filtered_free_tasks_set) {
        auto tit = env->task_pool.find(tid);
        if (tit != env->task_pool.end() && !tit->second.locations.empty())
            tasks_by_zone[zid_100(tit->second.locations[0], cols_)].push_back(tid);
    }
    if (diag_on) diag.task_zones = tasks_by_zone.size();

    std::unordered_map<int, int> region_rep_loc;
    for (const auto& [z, tids] : tasks_by_zone) {
        for (int tid : tids) {
            auto tit = env->task_pool.find(tid);
            if (tit != env->task_pool.end() && !tit->second.locations.empty()) {
                region_rep_loc[z] = tit->second.locations.front();
                break;
            }
        }
    }

    if (diag_on) {
        auto has_prefiltered_task_in_radius = [&](int a_loc) -> bool {
            int a_zone = zid_100(a_loc, cols_);
            auto sz = tasks_by_zone.find(a_zone);
            if (sz != tasks_by_zone.end() && !sz->second.empty()) return true;
            for (int radius = 1; radius <= search_radius; radius++) {
                for (const auto& [z2, tids] : tasks_by_zone) {
                    if (zh_dist(a_zone, z2) == radius && !tids.empty()) return true;
                }
            }
            return false;
        };
        for (int a : free_agents) {
            if (proposed_schedule[a] >= 0) continue;
            if (has_prefiltered_task_in_radius(env->curr_states[a].location))
                diag.legal_agents_after_task_filter++;
        }
    }

    std::unordered_set<int> done_tasks;
    int assigned_count = 0;
    int idle_global_count = 0;
    int idle_global_assigned_count = 0;
    std::unordered_map<long long, bool> region_allow_cache;
    const int region_flow_batch = std::max(1, cfg().game_region_flow_batch);
    int region_flow_since_cache_clear = 0;

    auto note_region_flow_update = [&]() {
        if (++region_flow_since_cache_clear < region_flow_batch) return;
        region_allow_cache.clear();
        region_flow_since_cache_clear = 0;
        if (diag_on) diag.region_batch_flushes++;
    };

    auto score_candidate = [&](int a_loc, int tid) -> float {
        if (!diag_on) return base_score_ex(a_loc, tid, step, 0, eff_density_cap);
        diag.attempts++;
        auto tit = env->task_pool.find(tid);
        if (tit == env->task_pool.end() || tit->second.locations.empty()) {
            diag.reject_missing++;
            return INVALID_SCORE;
        }
        const auto& locs = tit->second.locations;
        if (task_has_channel_errand(locs)) {
            diag.reject_channel++;
            return INVALID_SCORE;
        }
        if (!directional_future_flow_allows(a_loc, locs, 0, tid)) {
            diag.reject_flow++;
            return INVALID_SCORE;
        }
        const float bs = base_score_ex(a_loc, tid, step, 0, eff_density_cap);
        if (!is_valid_score(bs)) diag.reject_zone_or_route++;
        return bs;
    };

    auto cheap_task_score = [&](int a_loc, int tid) -> float {
        auto tit = env->task_pool.find(tid);
        if (tit == env->task_pool.end() || tit->second.locations.empty())
            return INVALID_SCORE;
        const int tloc = tit->second.locations.front();
        float dist = score_distance_between(a_loc, tloc);
        if (dist < 0.0f) return INVALID_SCORE;
        return distance_score(dist);
    };

    auto region_allows = [&](int a_zone, int a_loc, int task_zone) -> bool {
        long long key = (static_cast<long long>(a_zone) << 32) ^
                        static_cast<unsigned int>(task_zone);
        auto cached = region_allow_cache.find(key);
        if (cached != region_allow_cache.end()) {
            if (diag_on) diag.region_cache_hits++;
            return cached->second;
        }
        if (diag_on) {
            diag.region_checks++;
            diag.region_cache_misses++;
        }
        auto rit = region_rep_loc.find(task_zone);
        if (rit == region_rep_loc.end()) {
            region_allow_cache[key] = false;
            return false;
        }
        std::vector<int> rep_route(1, rit->second);
        bool ok = directional_future_flow_allows(a_loc, rep_route, 0);
        if (!ok && diag_on) diag.region_reject_flow++;
        region_allow_cache[key] = ok;
        return ok;
    };

    if (cfg().game_distance_score_mode == 3) {
        struct GlobalCand {
            float score;
            int tid;
        };
        struct CoarseCand {
            float score;
            int tid;
        };
        struct HeapItem {
            float score;
            int agent_pos;
            int cand_pos;
            bool operator<(const HeapItem& other) const {
                return score < other.score;
            }
        };

        const int cand_topk = std::max(1, cfg().game_global_cand_topk);
        // Per-zone coarse-score shortlist. Only these tasks pay for the full
        // future-flow and density evaluation below.
        const int full_score_batch =
            std::max(1, cfg().game_global_max_task_scan);
        std::vector<std::vector<GlobalCand>> all_candidates(free_agents.size());
        std::vector<unsigned char> idle_global_agent(n_agents, 0);
        if (cfg().game_idle_global_match) {
            std::vector<int> idle_order;
            idle_order.reserve(free_agents.size());
            const int idle_threshold = std::max(1, cfg().game_idle_global_ticks);
            for (int a : free_agents) {
                if (agent_idle_ticks_[a] >= idle_threshold)
                    idle_order.push_back(a);
            }
            std::shuffle(idle_order.begin(), idle_order.end(), idle_global_rng_);
            idle_global_count = std::min<int>(
                std::max(0, cfg().game_idle_global_max_agents), idle_order.size());
            for (int i = 0; i < idle_global_count; ++i)
                idle_global_agent[idle_order[i]] = 1;
            if (diag_on) diag.idle_global_eligible = idle_global_count;
        }
        bool idle_global_phase = false;

        auto build_candidates = [&]() {
            const auto rebuild_start = std::chrono::steady_clock::now();
            for (auto& v : all_candidates) v.clear();
            int n_threads = cfg().num_threads;
            if (n_threads <= 0) {
                n_threads = static_cast<int>(std::thread::hardware_concurrency());
                if (n_threads <= 0) n_threads = 4;
            }
            n_threads = std::max(1, std::min<int>(n_threads, free_agents.size()));
            std::vector<std::thread> threads;
            threads.reserve(n_threads);
            int chunk = ((int)free_agents.size() + n_threads - 1) / n_threads;
            for (int t = 0; t < n_threads; ++t) {
                int begin = t * chunk;
                int end = std::min<int>(free_agents.size(), begin + chunk);
                if (begin >= end) continue;
                threads.emplace_back([&, begin, end]() {
                    for (int pos = begin; pos < end; ++pos) {
	                        if (std::chrono::steady_clock::now() > deadline) break;
	                        int a = free_agents[pos];
	                        if (proposed_schedule[a] >= 0) continue;
	                        const bool use_global_idle_search =
	                            idle_global_phase && idle_global_agent[a] != 0;
	                        if (idle_global_phase && !use_global_idle_search) continue;
	                        int a_loc = env->curr_states[a].location;
	                        int a_zone = zid_100(a_loc, cols_);
	                        auto& cand = all_candidates[pos];
	                        std::vector<std::pair<int, int>> zone_order;
	                        zone_order.reserve(tasks_by_zone.size());
	                        for (const auto& [z2, tids] : tasks_by_zone) {
	                            if (tids.empty()) continue;
	                            int zd = zh_dist(a_zone, z2);
	                            if (use_global_idle_search || zd <= search_radius)
	                                zone_order.emplace_back(zd, z2);
	                        }
	                        std::sort(zone_order.begin(), zone_order.end(),
	                            [](const auto& x, const auto& y) {
	                                if (x.first != y.first) return x.first < y.first;
	                                return x.second < y.second;
	                            });
	                        for (const auto& [zd, z2] : zone_order) {
	                            (void)zd;
	                            if ((int)cand.size() >= cand_topk ||
	                                std::chrono::steady_clock::now() > deadline)
	                                break;
	                            auto tz = tasks_by_zone.find(z2);
	                            if (tz == tasks_by_zone.end()) continue;
	                            const auto& tids = tz->second;

	                            // First rank every task in this nearby zone by the
	                            // cheap distance-only score: pickup_weight *
	                            // agent->first-errand distance plus all errand legs.
	                            std::vector<CoarseCand> coarse;
	                            coarse.reserve(tids.size());
	                            for (size_t task_pos = 0; task_pos < tids.size(); ++task_pos) {
	                                if ((task_pos & 15U) == 0U &&
	                                    std::chrono::steady_clock::now() > deadline)
	                                    break;
	                                const int tid = tids[task_pos];
	                                if (done_tasks.count(tid)) continue;
	                                auto tit = env->task_pool.find(tid);
	                                if (tit == env->task_pool.end() ||
	                                    tit->second.locations.empty()) continue;
	                                const auto& locs = tit->second.locations;
	                                auto cached = task_static_cache_.find(tid);
	                                if (cached == task_static_cache_.end()) continue;
	                                const float pickup_dist =
	                                    score_distance_between(a_loc, locs.front());
	                                if (pickup_dist < 0.0f) continue;
	                                float score = distance_score(pickup_dist) *
	                                    std::max(0.0f, cfg().game_pickup_weight);
	                                if (cached->second.distance_ready) {
	                                    if (!cached->second.distance_valid) continue;
	                                    score += distance_score(
	                                        cached->second.intrinsic_distance);
	                                } else {
	                                    if (!cached->second.coarse_distance_valid) continue;
	                                    score += distance_score(
	                                        cached->second.coarse_intrinsic_distance);
	                                }
	                                coarse.push_back({score, tid});
	                            }
	                            if (coarse.empty()) continue;
	                            auto coarse_better =
	                                [](const CoarseCand& x, const CoarseCand& y) {
	                                    if (x.score != y.score) return x.score > y.score;
	                                    return x.tid < y.tid;
	                                };
	                            if ((int)coarse.size() > full_score_batch) {
	                                std::nth_element(coarse.begin(),
	                                    coarse.begin() + full_score_batch,
	                                    coarse.end(), coarse_better);
	                                coarse.resize(full_score_batch);
	                            }
	                            std::sort(coarse.begin(), coarse.end(), coarse_better);

	                            // Only the best distance candidates enter the
	                            // expensive full score (future-flow, density, etc.).
	                            const int limit = static_cast<int>(coarse.size());
	                            for (int i = 0; i < limit; ++i) {
	                                if (std::chrono::steady_clock::now() > deadline) break;
	                                const int tid = coarse[i].tid;
	                                const float score =
	                                    base_score_ex(a_loc, tid, step, 0, eff_density_cap);
	                                if (is_valid_score(score)) cand.push_back({score, tid});
	                            }
	                            if ((int)cand.size() > cand_topk) {
	                                std::nth_element(cand.begin(), cand.begin() + cand_topk,
	                                    cand.end(), [](const GlobalCand& x, const GlobalCand& y) {
	                                        return x.score > y.score;
	                                    });
	                                cand.resize(cand_topk);
	                            }
	                            std::sort(cand.begin(), cand.end(),
	                                [](const GlobalCand& x, const GlobalCand& y) {
	                                    if (x.score != y.score) return x.score > y.score;
	                                    return x.tid < y.tid;
	                                });
	                        }
                    }
                });
            }
            for (auto& th : threads) th.join();
            (void)rebuild_start;
            (void)n_threads;
        };

        while (assigned_count < assign_budget) {
            if (std::chrono::steady_clock::now() > deadline) {
                if (diag_on) diag.timeout_stop = 1;
                break;
            }
            build_candidates();

            std::priority_queue<HeapItem> heap;
            int active_agents = 0;
            for (int pos = 0; pos < (int)all_candidates.size(); ++pos) {
                if (!all_candidates[pos].empty()) {
                    heap.push({all_candidates[pos][0].score, pos, 0});
                    active_agents++;
                }
            }
            if (diag_on) {
                diag.considered_agents += active_agents;
                diag.region_batch_flushes++;
            }
            if (heap.empty()) {
                if (!idle_global_phase && idle_global_count > 0) {
                    idle_global_phase = true;
                    continue;
                }
                break;
            }

            int batch_assigned = 0;
            while (!heap.empty() && assigned_count < assign_budget &&
                   batch_assigned < region_flow_batch) {
                if (std::chrono::steady_clock::now() > deadline) {
                    if (diag_on) diag.timeout_stop = 1;
                    break;
                }
                HeapItem item = heap.top();
                heap.pop();
                if (item.agent_pos < 0 || item.agent_pos >= (int)free_agents.size()) continue;
                auto& cand = all_candidates[item.agent_pos];
                if (item.cand_pos < 0 || item.cand_pos >= (int)cand.size()) continue;

                int a = free_agents[item.agent_pos];
                int tid = cand[item.cand_pos].tid;
                auto push_next = [&]() {
                    int next = item.cand_pos + 1;
                    if (next < (int)cand.size())
                        heap.push({cand[next].score, item.agent_pos, next});
                };

                if (proposed_schedule[a] >= 0 || done_tasks.count(tid)) {
                    push_next();
                    continue;
                }

                int a_loc = env->curr_states[a].location;
                float bs = score_candidate(a_loc, tid);
                if (!is_valid_score(bs)) {
                    push_next();
                    continue;
                }

                proposed_schedule[a] = tid;
                add_directional_future_flow_for_task(a_loc, tid, 0);
                auto tit = env->task_pool.find(tid);
                if (tit != env->task_pool.end() && tit->second.locations.size() > 2)
                    parked_task_[a] = tid;
                else
                    parked_task_[a] = -1;
                done_tasks.insert(tid);
                ++assigned_count;
                ++batch_assigned;
                if (idle_global_phase && idle_global_agent[a] != 0) {
                    ++idle_global_assigned_count;
                    if (diag_on) ++diag.idle_global_assigned;
                }
                agent_idle_ticks_[a] = 0;
                if (diag_on) diag.legal_agents_after_full_filter++;
            }

            region_allow_cache.clear();
            if (batch_assigned == 0) {
                if (!idle_global_phase && idle_global_count > 0) {
                    idle_global_phase = true;
                    continue;
                }
                break;
            }
        }
        if (assigned_count >= assign_budget && diag_on) diag.budget_stop = 1;
    } else for (int a : free_agents) {
        if (assigned_count >= assign_budget) {
            if (diag_on) diag.budget_stop = 1;
            break;
        }
        if (std::chrono::steady_clock::now() > deadline) {
            if (diag_on) diag.timeout_stop = 1;
            break;
        }
        if (proposed_schedule[a] >= 0) continue;
        if (diag_on) diag.considered_agents++;

        int a_loc = env->curr_states[a].location;
        int a_zone = zid_100(a_loc, cols_);
        int best_tid = -1;
        float best_score = -1e9f;
        const long long attempts_before_agent = diag_on ? diag.attempts : 0;
        bool has_valid_candidate = false;
        bool saw_region_candidate = false;

        if (cfg().game_region_match) {
            std::vector<int> region_order;
            for (const auto& [z2, tids] : tasks_by_zone) {
                if (tids.empty()) continue;
                if (zh_dist(a_zone, z2) <= search_radius)
                    region_order.push_back(z2);
            }
            std::sort(region_order.begin(), region_order.end(), [&](int x, int y) {
                int dx = zh_dist(a_zone, x);
                int dy = zh_dist(a_zone, y);
                if (dx != dy) return dx < dy;
                return x < y;
            });

            std::unordered_set<int> tried_tasks_for_agent;
            for (int z2 : region_order) {
                auto tz = tasks_by_zone.find(z2);
                if (tz == tasks_by_zone.end() || tz->second.empty()) continue;
                saw_region_candidate = true;
                if (!region_allows(a_zone, a_loc, z2))
                    continue;

                while (true) {
                    int local_best_tid = -1;
                    float local_best_score = INVALID_SCORE;
                    for (int tid : tz->second) {
                        if (done_tasks.count(tid) || tried_tasks_for_agent.count(tid))
                            continue;
                        if (diag_on) diag.region_task_scans++;
                        float cs = cheap_task_score(a_loc, tid);
                        if (cs > local_best_score) {
                            local_best_score = cs;
                            local_best_tid = tid;
                        }
                    }
                    if (local_best_tid < 0)
                        break;

                    tried_tasks_for_agent.insert(local_best_tid);
                    if (diag_on) diag.region_full_checks++;
                    float bs = score_candidate(a_loc, local_best_tid);
                    if (is_valid_score(bs)) {
                        has_valid_candidate = true;
                        best_tid = local_best_tid;
                        best_score = bs;
                        break;
                    }
                }
                if (best_tid >= 0)
                    break;
            }
        } else {
            // 同 zone 搜索
            auto sz = tasks_by_zone.find(a_zone);
            if (sz != tasks_by_zone.end()) {
                for (int tid : sz->second) {
                    if (done_tasks.count(tid)) continue;
                    float bs = score_candidate(a_loc, tid);
                    if (is_valid_score(bs)) has_valid_candidate = true;
                    if (bs > best_score) { best_score = bs; best_tid = tid; }
                }
            }

            // 邻居 zone 搜索（IDEA1: 自适应搜索半径）
            if (best_tid < 0) {
                for (int radius = 1; radius <= search_radius && best_tid < 0; radius++) {
                    for (auto& [z2, tids] : tasks_by_zone) {
                        if (z2 == a_zone || zh_dist(a_zone, z2) != radius) continue;
                        int checked = 0;
                        for (int tid : tids) {
                            if (done_tasks.count(tid)) continue;
                            if (checked++ >= 20) {
                                if (diag_on) diag.neighbor_checked_cap_hits++;
                                break;
                            }
                            float bs = score_candidate(a_loc, tid);
                            if (is_valid_score(bs)) has_valid_candidate = true;
                            if (bs > best_score) { best_score = bs; best_tid = tid; }
                        }
                    }
                }
            }
        }

        if (diag_on && has_valid_candidate)
            diag.legal_agents_after_full_filter++;

        if (best_tid >= 0) {
            proposed_schedule[a] = best_tid;
            if (cfg().game_region_match) {
                // The final per-agent check must see all assignments already accepted
                // in this tick, especially during the large initial assignment wave.
                // Batch only controls when the coarse region cache is invalidated.
                add_directional_future_flow_for_task(a_loc, best_tid, 0);
                note_region_flow_update();
            } else {
                add_directional_future_flow_for_task(a_loc, best_tid, 0);
            }
            auto tit = env->task_pool.find(best_tid);
            if (tit != env->task_pool.end() && tit->second.locations.size() > 2)
                parked_task_[a] = best_tid;
            else
                parked_task_[a] = -1;
            done_tasks.insert(best_tid);
            ++assigned_count;
            agent_idle_ticks_[a] = 0;
        } else {
            proposed_schedule[a] = -1;
            if (diag_on) {
                if (diag.attempts == attempts_before_agent && !saw_region_candidate) diag.no_candidates_in_radius++;
                else diag.no_valid_candidates++;
            }
        }
    }
    if (diag_on) diag.assigned_count = assigned_count;

    // 未匹配的 free agent 保持未分配。
    for (int a : free_agents) {
        if (proposed_schedule[a] < 0) proposed_schedule[a] = -1;
    }

    // ---- Phase 3: reassign swap LNS ----
    // Compare alternatives against net flow: remove the current route first,
    // then restore it or commit the accepted replacement.
    if (cfg().game_reassign && cfg().game_lns_pct > 0) {
        const auto lns_start = std::chrono::steady_clock::now();
        const int lns_ms = std::max(1, time_limit * cfg().game_lns_pct / 100);
        const auto lns_deadline = std::min(deadline, lns_start + std::chrono::milliseconds(lns_ms));

        std::vector<int> reassign_agents;
        reassign_agents.reserve(n_agents);
        const int min_dist = std::max(0, cfg().game_min_dist);
        for (int a = 0; a < n_agents; ++a) {
            if (a >= (int)proposed_schedule.size() || a >= (int)env->curr_states.size())
                continue;
            const int tid = proposed_schedule[a];
            if (tid < 0) continue;
            auto tit = env->task_pool.find(tid);
            if (tit == env->task_pool.end() || tit->second.locations.empty())
                continue;
            const Task& task = tit->second;
            if (task.idx_next_loc != 0) continue;
            const int a_loc = env->curr_states[a].location;
            if (a_loc < 0) continue;
            float pickup_dist = score_distance_between(a_loc, task.locations.front());
            if (pickup_dist >= 0.0f && pickup_dist < (float)min_dist)
                continue;
            reassign_agents.push_back(a);
        }

        const float keep_bias = std::max(0.0f, cfg().game_keep_bias);
        auto pair_score_for_swap = [&](int a, int tid) -> float {
            if (a < 0 || a >= (int)env->curr_states.size() || tid < 0)
                return INVALID_SCORE;
            const int a_loc = env->curr_states[a].location;
            if (a_loc < 0) return INVALID_SCORE;
            return base_score_ex(a_loc, tid, step, 0, eff_density_cap);
        };

        auto adjust_agent_task_flow = [&](int a, int tid, int weight) {
            if (a < 0 || a >= (int)env->curr_states.size() || tid < 0) return;
            const int a_loc = env->curr_states[a].location;
            if (a_loc < 0) return;
            adjust_directional_future_flow_for_task(a_loc, tid, 0, weight);
        };

        auto pair_score_with_net_flow = [&](int a, int ta, int b, int tb) -> float {
            const float sa = pair_score_for_swap(a, ta);
            if (!is_valid_score(sa)) return INVALID_SCORE;
            adjust_agent_task_flow(a, ta, 1);
            const float sb = pair_score_for_swap(b, tb);
            adjust_agent_task_flow(a, ta, -1);
            if (!is_valid_score(sb)) return INVALID_SCORE;
            return sa + sb;
        };

        if (!reassign_agents.empty() && !env->new_tasks.empty()) {
            std::vector<int> new_free_tasks;
            new_free_tasks.reserve(env->new_tasks.size());
            for (int tid : env->new_tasks) {
                if (filtered_free_tasks_set.count(tid) && !done_tasks.count(tid)) {
                    auto tit = env->task_pool.find(tid);
                    if (tit != env->task_pool.end() && !tit->second.locations.empty() &&
                        !task_has_channel_errand(tit->second.locations))
                        new_free_tasks.push_back(tid);
                }
            }

            for (int a : reassign_agents) {
                if (std::chrono::steady_clock::now() >= lns_deadline) {
                    if (diag_on) diag.timeout_stop = 1;
                    break;
                }
                if (new_free_tasks.empty()) break;
                const int old_tid = proposed_schedule[a];
                if (old_tid < 0) continue;

                adjust_agent_task_flow(a, old_tid, -1);
                const float old_score_raw = pair_score_for_swap(a, old_tid);
                if (!is_valid_score(old_score_raw)) {
                    adjust_agent_task_flow(a, old_tid, 1);
                    continue;
                }
                const float old_score = old_score_raw + keep_bias;

                int best_idx = -1;
                float best_score = old_score;
                for (int i = 0; i < (int)new_free_tasks.size(); ++i) {
                    if (std::chrono::steady_clock::now() >= lns_deadline) break;
                    ++diag.reassign_replace_attempts;
                    const int new_tid = new_free_tasks[i];
                    const float ns = pair_score_for_swap(a, new_tid);
                    if (is_valid_score(ns) && ns > best_score + 1e-3f) {
                        best_score = ns;
                        best_idx = i;
                    }
                }

                if (best_idx >= 0) {
                    const int new_tid = new_free_tasks[best_idx];
                    adjust_agent_task_flow(a, new_tid, 1);
                    proposed_schedule[a] = new_tid;
                    if (old_tid >= 0) {
                        filtered_free_tasks_set.insert(old_tid);
                        done_tasks.erase(old_tid);
                    }
                    filtered_free_tasks_set.erase(new_tid);
                    done_tasks.insert(new_tid);
                    auto tit = env->task_pool.find(new_tid);
                    parked_task_[a] = (tit != env->task_pool.end() &&
                                       tit->second.locations.size() > 2) ? new_tid : -1;
                    new_free_tasks[best_idx] = new_free_tasks.back();
                    new_free_tasks.pop_back();
                    ++diag.reassign_replace_accepts;
                } else {
                    adjust_agent_task_flow(a, old_tid, 1);
                }
            }
        }

        if (reassign_agents.size() >= 2) {
            std::mt19937 rng(20260710u + (uint32_t)step);
            std::uniform_int_distribution<int> pick(0, (int)reassign_agents.size() - 1);
            const int max_attempts = std::max<int>(
                1, std::min<long long>(20000LL, (long long)reassign_agents.size() * 8LL));
            for (int attempt = 0; attempt < max_attempts; ++attempt) {
                if (std::chrono::steady_clock::now() >= lns_deadline) {
                    if (diag_on) diag.timeout_stop = 1;
                    break;
                }
                int ia = pick(rng);
                int ib = pick(rng);
                if (ia == ib) continue;
                int a = reassign_agents[ia];
                int b = reassign_agents[ib];
                if (a == b) continue;
                const int ta = proposed_schedule[a];
                const int tb = proposed_schedule[b];
                if (ta < 0 || tb < 0 || ta == tb) continue;

                ++diag.reassign_attempts;
                adjust_agent_task_flow(a, ta, -1);
                adjust_agent_task_flow(b, tb, -1);

                const float old_pair = pair_score_with_net_flow(a, ta, b, tb);
                const float new_pair = pair_score_with_net_flow(a, tb, b, ta);
                const float old_score = is_valid_score(old_pair)
                    ? old_pair + 2.0f * keep_bias : INVALID_SCORE;
                const float new_score = new_pair;
                if (new_score > old_score + 1e-3f) {
                    adjust_agent_task_flow(a, tb, 1);
                    adjust_agent_task_flow(b, ta, 1);
                    proposed_schedule[a] = tb;
                    proposed_schedule[b] = ta;
                    std::swap(parked_task_[a], parked_task_[b]);
                    ++diag.reassign_accepts;
                } else {
                    adjust_agent_task_flow(a, ta, 1);
                    adjust_agent_task_flow(b, tb, 1);
                }
            }
        }
    }

    set_game_prefilter_stats({
        diag.prefilter_task_total,
        diag.prefilter_task_kept,
        diag.prefilter_task_total - diag.prefilter_task_kept,
        diag.prefilter_reject_missing,
        diag.prefilter_reject_channel,
        diag.prefilter_reject_capacity,
        diag.prefilter_reject_flow,
        diag.prefilter_reject_lock,
        diag.prefilter_reject_density,
        diag.prefilter_reject_branch,
        diag.prefilter_reject_other,
        blocked_zone_count,
        blocked_zone_ids.str(),
        diag.reassign_replace_attempts,
        diag.reassign_replace_accepts,
        diag.reassign_attempts,
        diag.reassign_accepts,
        idle_global_count,
        idle_global_assigned_count
    });

    if (short_task_length_cuts_ready_) {
        for (int a = 0; a < n_agents; ++a) {
            if (a >= (int)proposed_schedule.size()) continue;
            const int tid = proposed_schedule[a];
            if (tid < 0) continue;
            auto tit = env->task_pool.find(tid);
            if (tit == env->task_pool.end()) continue;
            const auto& locs = tit->second.locations;
            int length = 0;
            for (std::size_t i = 1; i < locs.size(); ++i) {
                const int from = locs[i - 1];
                const int to = locs[i];
                length += std::abs(from / cols_ - to / cols_) +
                          std::abs(from % cols_ - to % cols_);
            }
            int bin = 0;
            while (bin < 4 && length > short_task_length_cuts_[bin]) ++bin;
            ++diag.length_gate_final_active[bin];
        }
    }

    // URGENCY: sync urgency info to PibtLock
    if (cfg().game_urgency_priority) {
        for (int a = 0; a < n_agents && a < PibtLock::MAX_AGENTS; a++) {
            if (proposed_schedule[a] >= 0 && lock.agent_blocked[a] == 0) {
                // 已有task的agent: 如果距离目标很近，给予优先级加成
                auto tit = env->task_pool.find(proposed_schedule[a]);
                if (tit != env->task_pool.end() && !tit->second.locations.empty()) {
                    int tloc = tit->second.locations[tit->second.idx_next_loc];
                    int aloc = env->curr_states[a].location;
                    int ar = aloc / cols_, ac = aloc % cols_;
                    int tr = tloc / cols_, tc = tloc % cols_;
                    int dist = std::abs(ar - tr) + std::abs(ac - tc);
                    if (dist <= cfg().game_urgency_threshold) {
                        float boost = cfg().game_urgency_weight / (float)(dist + 1);
                        lock.agent_priority_penalty[a] = boost;
                    }
                }
            }
        }
    }
    if (diag_on) {
        std::cerr << "[GameSchedDiag]"
                  << " step=" << diag.step
                  << " busy=" << diag.busy_agents << "/" << n_agents
                  << " free_initial=" << diag.free_agents_initial
                  << " free_after_phase1=" << diag.free_agents_after_phase1
                  << " max_busy=" << diag.max_total_busy
                  << " assign_budget=" << diag.assign_budget
                  << " assigned=" << diag.assigned_count
                  << " idle_global=" << diag.idle_global_eligible
                  << "/" << diag.idle_global_assigned
                  << " budget_stop=" << diag.budget_stop
                  << " timeout_stop=" << diag.timeout_stop
                  << " radius=" << diag.search_radius
                  << " free_tasks=" << diag.free_tasks
                  << " channel_free_tasks=" << diag.channel_free_tasks
                  << " prefilter_task_total=" << diag.prefilter_task_total
                  << " prefilter_task_kept=" << diag.prefilter_task_kept
                  << " prefilter_task_keep_ratio="
                  << (diag.prefilter_task_total > 0
                      ? (float)diag.prefilter_task_kept / (float)diag.prefilter_task_total : 0.0f)
                  << " prefilter_reject_missing=" << diag.prefilter_reject_missing
                  << " prefilter_reject_channel=" << diag.prefilter_reject_channel
                  << " prefilter_reject_capacity=" << diag.prefilter_reject_capacity
                  << " prefilter_reject_flow=" << diag.prefilter_reject_flow
                  << " prefilter_reject_lock=" << diag.prefilter_reject_lock
                  << " prefilter_reject_density=" << diag.prefilter_reject_density
                  << " prefilter_reject_branch=" << diag.prefilter_reject_branch
                  << " prefilter_reject_other=" << diag.prefilter_reject_other
                  << " len_gate=" << diag.length_gate_active
                  << " len_tasks=" << diag.length_gate_kept
                  << "/" << diag.length_gate_input
                  << " len_s0=" << diag.length_gate_short_active[0]
                  << "+" << diag.length_gate_short_kept[0]
                  << "/" << diag.length_gate_short_quota[0]
                  << "(" << diag.length_gate_short_total[0] << ")"
                  << " len_s1=" << diag.length_gate_short_active[1]
                  << "+" << diag.length_gate_short_kept[1]
                  << "/" << diag.length_gate_short_quota[1]
                  << "(" << diag.length_gate_short_total[1] << ")"
                  << " len_cut=" << diag.length_gate_cut[0] << ","
                  << diag.length_gate_cut[1] << ","
                  << diag.length_gate_cut[2] << ","
                  << diag.length_gate_cut[3]
                  << " len_avg=" << short_task_length_means_[0] << ","
                  << short_task_length_means_[1] << ","
                  << short_task_length_means_[2] << ","
                  << short_task_length_means_[3] << ","
                  << short_task_length_means_[4]
                  << " len_act=" << diag.length_gate_final_active[0] << ","
                  << diag.length_gate_final_active[1] << ","
                  << diag.length_gate_final_active[2] << ","
                  << diag.length_gate_final_active[3] << ","
                  << diag.length_gate_final_active[4]
                  << " legal_agents_after_task_filter=" << diag.legal_agents_after_task_filter
                  << " legal_agents_after_task_filter_ratio="
                  << (diag.free_agents_after_phase1 > 0
                      ? (float)diag.legal_agents_after_task_filter / (float)diag.free_agents_after_phase1 : 0.0f)
                  << " legal_agents_after_full_filter=" << diag.legal_agents_after_full_filter
                  << " legal_agents_after_full_filter_ratio="
                  << (diag.considered_agents > 0
                      ? (float)diag.legal_agents_after_full_filter / (float)diag.considered_agents : 0.0f)
                  << " region_mode=" << diag.region_mode
                  << " region_checks=" << diag.region_checks
                  << " region_cache_hits=" << diag.region_cache_hits
                  << " region_cache_misses=" << diag.region_cache_misses
                  << " region_reject_flow=" << diag.region_reject_flow
                  << " region_task_scans=" << diag.region_task_scans
                  << " region_full_checks=" << diag.region_full_checks
                  << " region_batch_flushes=" << diag.region_batch_flushes
                  << " task_zones=" << diag.task_zones
                  << " considered_agents=" << diag.considered_agents
                  << " no_candidates_in_radius=" << diag.no_candidates_in_radius
                  << " no_valid_candidates=" << diag.no_valid_candidates
                  << " attempts=" << diag.attempts
                  << " reject_missing=" << diag.reject_missing
                  << " reject_channel=" << diag.reject_channel
                  << " reject_flow=" << diag.reject_flow
                  << " reject_zone_or_route=" << diag.reject_zone_or_route
                  << " reassign_replace_attempts=" << diag.reassign_replace_attempts
                  << " reassign_replace_accepts=" << diag.reassign_replace_accepts
                  << " reassign_attempts=" << diag.reassign_attempts
                  << " reassign_accepts=" << diag.reassign_accepts
                  << " neighbor_checked_cap_hits=" << diag.neighbor_checked_cap_hits
                  << " dyn_cap=" << (cfg().game_dir_flow_dynamic_cap ? 1 : 0)
                  << " cap_window=" << dynamic_cap_last_window_ticks_
                  << " cap_avg=" << dynamic_cap_last_avg_
                  << " cap_min=" << dynamic_cap_last_min_
                  << " cap_max=" << dynamic_cap_last_max_
                  << " stall_avg=" << dynamic_cap_last_stall_avg_
                  << " stall_max=" << dynamic_cap_last_stall_max_
                  << " stall_busy=" << dynamic_cap_last_busy_stall_avg_
                  << " stall_free=" << dynamic_cap_last_free_stall_avg_
                  << " cap_active=" << dynamic_cap_last_active_
                  << " cap_lo_hi=" << dynamic_cap_last_below_base_
                  << "/" << dynamic_cap_last_above_base_
                  << " cap_at_min=" << dynamic_cap_last_at_min_
                  << " cap_up=" << dynamic_cap_last_up_
                  << " cap_down=" << dynamic_cap_last_down_
                  << " blocked_zone_count=" << blocked_zone_count
                  << " blocked_zone_ids=" << blocked_zone_ids.str()
                  << "\n";
    }
}

// ================================================================
// base_score_ex — 三层豁免打分 (支持动态density_cap)
// ================================================================
float GameBalanceOptScheduler::base_score_ex(int a_loc, int tid, int step,
        int start_idx, int eff_density_cap,
        ScoreRejectReason* reject_reason) const {
    if (reject_reason != nullptr) *reject_reason = ScoreRejectReason::None;
    auto reject = [&](ScoreRejectReason reason) {
        if (reject_reason != nullptr) *reject_reason = reason;
        return INVALID_SCORE;
    };
    (void)step;
    auto it = env_->task_pool.find(tid);
    if (it == env_->task_pool.end() || it->second.locations.empty())
        return reject(ScoreRejectReason::Missing);
    if (!ready_) return 1.0f;
    auto& locs = it->second.locations;
    if (task_has_channel_errand(locs)) return reject(ScoreRejectReason::Channel);
    if (!directional_future_flow_allows(a_loc, locs, start_idx, tid))
        return reject(ScoreRejectReason::FutureFlow);

    int zone_hard_cap = cfg().game_zone_hard_cap;
    int exempt_main   = cfg().game_exempt_main;
    int density_cap   = eff_density_cap;
    int route_penalty  = cfg().game_route_penalty;

    if (cfg().game_score_exempt_enabled) {
        // ---- Layer 1: main zone 豁免 ----
        {
            int a_m, a_b;
            project(a_loc, a_m, a_b);
            if (a_m >= 0) {
                bool exempt = true;
                int prev_m = a_m;
                for (auto& loc : locs) {
                    int tm, tb;
                    project(loc, tm, tb);
                    if (tm < 0) continue;
                    int lo = std::min(prev_m, tm);
                    int hi = std::max(prev_m, tm);
                    for (auto& zr : zone_ranges_) {
                        if (zr.min_m <= hi && zr.max_m >= lo &&
                            zone_is_main_[zr.zone_id] &&
                            zone24_density_[zr.zone_id] >= exempt_main) {
                            exempt = false; break;
                        }
                    }
                    if (!exempt) break;
                    prev_m = tm;
                }
                if (exempt) {
                    float total = 0;
                    int prev = a_loc;
                    for (auto& loc : locs) {
                        float dist = score_distance_between(prev, loc);
                        if (dist < 0.0f) return reject(ScoreRejectReason::Other);
                        total += distance_score(dist);
                        prev = loc;
                    }
                    return total;
                }
            }
        }

        // ---- Layer 2: branch task 检查 ----
        {
            bool is_branch = true;
            int check_prev = a_loc;
            int max_z = (int)zone_is_main_.size();
            for (size_t ei = (size_t)start_idx; ei < locs.size(); ei++) {
                int loc = locs[ei];
                int x1 = check_prev % cols_, y1 = check_prev / cols_;
                int x2 = loc % cols_,        y2 = loc / cols_;
                int z1 = (y1 / 24) * n24c_ + (x1 / 24);
                if (z1 >= 0 && z1 < max_z) {
                    int d1 = zone24_density_[z1];
                    if (zone_is_main_[z1]) {
                        if (d1 >= zone_hard_cap) { is_branch = false; break; }
                    } else {
                        if (d1 >= 70) { is_branch = false; break; }
                    }
                }
                int z2 = (y2 / 24) * n24c_ + (x2 / 24);
                if (z2 >= 0 && z2 < max_z) {
                    int d2 = zone24_density_[z2];
                    if (zone_is_main_[z2]) {
                        if (d2 >= zone_hard_cap) { is_branch = false; break; }
                    } else {
                        if (d2 >= 70) { is_branch = false; break; }
                    }
                }
                int dx = abs(x2 - x1), sx = x1 < x2 ? 1 : -1;
                int dy = -abs(y2 - y1), sy = y1 < y2 ? 1 : -1;
                int err = dx + dy;
                int cx = x1, cy = y1;
                while (true) {
                    if (env_->map[cy * cols_ + cx] != 0) { is_branch = false; break; }
                    if (cx == x2 && cy == y2) break;
                    int e2 = 2 * err;
                    if (e2 >= dy) { err += dy; cx += sx; }
                    if (e2 <= dx) { err += dx; cy += sy; }
                }
                if (!is_branch) break;
                check_prev = loc;
            }

            if (is_branch) {
                float total = 0;
                int prev = a_loc;
                for (size_t ei = (size_t)start_idx; ei < locs.size(); ei++) {
                    int r2 = locs[ei] / cols_, c2 = locs[ei] % cols_;
                    int z2 = (r2 / 24) * n24c_ + (c2 / 24);
                    if (z2 >= 0 && z2 < max_z && zone24_density_[z2] >= 70)
                        return reject(ScoreRejectReason::Branch);
                    float dist = score_distance_between(prev, locs[ei]);
                    if (dist < 0.0f) return reject(ScoreRejectReason::Other);
                    total += distance_score(dist);
                    prev = locs[ei];
                }
                return total;
            }

            // ---- Layer 3: 逃逸豁免 ----
            if (start_idx == 0) {
                float esc = connection_exempt_score(a_loc, locs);
                if (is_valid_score(esc)) return esc;
            } else {
                std::vector<int> remaining(locs.begin() + start_idx, locs.end());
                float esc = connection_exempt_score(a_loc, remaining);
                if (is_valid_score(esc)) return esc;
            }
        }
    }

    // ---- Default: zone 密度 + 方向锁 打分 ----
    int a_m, a_b;
    project(a_loc, a_m, a_b);
    if (a_m < 0) return 1.0f;

    if (use_direct_distance_score() && start_idx >= 0 &&
        start_idx < static_cast<int>(locs.size())) {
        auto cached = task_static_cache_.find(tid);
        if (cached != task_static_cache_.end() &&
            ((cached->second.distance_ready && cached->second.distance_valid &&
              start_idx < static_cast<int>(cached->second.tail_distance.size())) ||
             (!cached->second.distance_ready && cached->second.coarse_distance_valid &&
              start_idx < static_cast<int>(cached->second.coarse_tail_distance.size())))) {
            const float pickup_dist = score_distance_between(a_loc, locs[start_idx]);
            if (pickup_dist < 0.0f) return reject(ScoreRejectReason::Other);
            float total = distance_score(pickup_dist) *
                std::max(0.0f, cfg().game_pickup_weight);
            const float internal_dist = cached->second.distance_ready
                ? cached->second.tail_distance[start_idx]
                : cached->second.coarse_tail_distance[start_idx];
            total += distance_score(internal_dist);
            total -= directional_future_flow_soft_penalty(
                a_loc, locs, start_idx, tid);
            return total;
        }
    }

    float total_score = 0;
    int prev_m = a_m, prev_b = a_b;
    int prev_loc = a_loc;

    for (size_t ei = (size_t)start_idx; ei < locs.size(); ei++) {
        int tm, tb;
        project(locs[ei], tm, tb);
        if (tm < 0) continue;

        int lo = std::min(prev_m, tm);
        int hi = std::max(prev_m, tm);
        int intent = (tm > prev_m) ? +1 : -1;

        int d_max_seg = 0;
        if (!use_direct_distance_score()) {
            for (auto& zr : zone_ranges_) {
                if (zr.min_m <= hi && zr.max_m >= lo) {
                    int d = zone24_density_[zr.zone_id];
                    // 方向锁: 逆流阻断
                    if (zr.lock_dir != 0 && zr.lock_dir != intent) {
                        return reject(ScoreRejectReason::LockDir);
                    }
                    // 密度上限
                    if (d >= density_cap) {
                        return reject(ScoreRejectReason::Density);
                    }
                    if (d > d_max_seg) d_max_seg = d;
                }
            }
        }

        float seg_dist = score_distance_between(prev_loc, locs[ei]);
        if (seg_dist < 0.0f) return reject(ScoreRejectReason::Other);
        float seg_score = distance_score(seg_dist);
        if (use_direct_distance_score() && ei == (size_t)start_idx) {
            seg_score *= std::max(0.0f, cfg().game_pickup_weight);
        }

        if (d_max_seg > route_penalty) {
            int idx = std::min(d_max_seg, route_penalty + 20) - route_penalty;
            seg_score = apply_route_density_penalty(seg_score, DIVISOR_TABLE[idx]);
        }

        total_score += seg_score;
        prev_loc = locs[ei];
        prev_m = tm; prev_b = tb;
    }

    if (use_direct_distance_score()) {
        total_score -= directional_future_flow_soft_penalty(a_loc, locs, start_idx, tid);
    }
    return total_score;
}

} // namespace optsvr
