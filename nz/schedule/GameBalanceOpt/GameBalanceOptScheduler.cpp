// optsvr/schedule/GameBalanceOpt/GameBalanceOptScheduler.cpp
// 中心线zone调度器实现 — BFS 中心线 + 24×24 zone 密度 + 方向锁 + 三层豁免
#include "GameBalanceOptScheduler.h"
#include "orz_highway.h"
#include "ScheduleStatsExport.h"
#include <climits>
#include <cmath>
#include <iostream>

namespace nz {

// ================================================================
// 常量（对应 IDEA/GameOpt 的 config.h 宏 + RuntimeConfig 参数）
// ================================================================
static constexpr int    CONTAIN_EXPAND   = 1;
static constexpr int    ROUTE_THRESHOLD  = 20;
static constexpr float  INVALID_SCORE    = -1e9f;
static constexpr float  DIVISOR_TABLE[21] = {
    1.0f, 1.1f, 1.2f, 1.3f, 1.4f,
    1.5f, 1.6f, 1.7f, 1.8f, 1.9f,
    2.0f, 2.1f, 2.2f, 2.3f, 2.4f,
    2.5f, 2.6f, 2.7f, 2.8f, 2.9f,
    3.0f
};

static inline bool use_direct_distance_score() {
    return cfg().game_distance_score_mode == 1;
}

static inline bool is_valid_score(float score) {
    return score > INVALID_SCORE * 0.5f;
}

static inline float distance_score(float dist) {
    return use_direct_distance_score() ? -dist : 1.0f / (dist + 1.0f);
}

static inline float apply_route_density_penalty(float score, float divisor) {
    return use_direct_distance_score() ? score * divisor : score / divisor;
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

bool GameBalanceOptScheduler::game_directional_flow_enabled() const {
    return cfg().planner_mode == 16 && cfg().game_dir_flow_enabled &&
           cfg().game_dir_flow_cap > 0;
}

bool GameBalanceOptScheduler::task_has_channel_errand(const std::vector<int>& locs) const {
    if (cfg().planner_mode != 16 || !cfg().game_skip_channel_errands) return false;
    for (int loc : locs) {
        if (ORZ_Highway::is_highway_cell(loc)) return true;
    }
    return false;
}

void GameBalanceOptScheduler::reset_directional_future_flow() {
    const int nz = n24c_ * n24r_;
    zone_future_pos_.assign(std::max(0, nz), 0.0f);
    zone_future_neg_.assign(std::max(0, nz), 0.0f);
}

void GameBalanceOptScheduler::add_directional_future_flow_route(
    int agent_loc, const std::vector<int>& locs, int start_idx, int weight) {
    if (!game_directional_flow_enabled() || !ready_ || locs.empty() || weight <= 0) return;
    const int nz = n24c_ * n24r_;
    if ((int)zone_future_pos_.size() != nz || (int)zone_future_neg_.size() != nz)
        reset_directional_future_flow();

    int prev_m = -1, prev_b = -1;
    project(agent_loc, prev_m, prev_b);
    if (prev_m < 0) return;

    float decay = cfg().game_dir_flow_decay;
    if (!std::isfinite(decay)) decay = GAME_DIR_FLOW_DECAY;
    decay = std::max(0.0f, std::min(1.0f, decay));
    float cur_weight = static_cast<float>(weight);
    int last_zid = -1;

    auto add_zone_flow = [&](const ZoneMainRange& zr, int intent) {
        const int zid = zr.zone_id;
        if (zid < 0 || zid >= nz || zid == last_zid) return;
        if (intent > 0) zone_future_pos_[zid] += cur_weight;
        else zone_future_neg_[zid] += cur_weight;
        last_zid = zid;
        cur_weight *= decay;
    };

    for (int i = std::max(0, start_idx); i < static_cast<int>(locs.size()); ++i) {
        int tm = -1, tb = -1;
        project(locs[i], tm, tb);
        if (tm < 0) continue;
        if (tm == prev_m) {
            prev_m = tm;
            prev_b = tb;
            continue;
        }

        const int intent = (tm > prev_m) ? +1 : -1;
        const int lo = std::min(prev_m, tm);
        const int hi = std::max(prev_m, tm);
        if (intent > 0) {
            for (const auto& zr : zone_ranges_) {
                if (zr.min_m <= hi && zr.max_m >= lo) add_zone_flow(zr, intent);
            }
        } else {
            for (auto it = zone_ranges_.rbegin(); it != zone_ranges_.rend(); ++it) {
                if (it->min_m <= hi && it->max_m >= lo) add_zone_flow(*it, intent);
            }
        }

        prev_m = tm;
        prev_b = tb;
    }
}

void GameBalanceOptScheduler::add_directional_future_flow_for_task(
    int agent_loc, int tid, int start_idx) {
    if (!game_directional_flow_enabled() || env_ == nullptr) return;
    auto it = env_->task_pool.find(tid);
    if (it == env_->task_pool.end() || it->second.locations.empty()) return;
    const auto& locs = it->second.locations;
    int idx = start_idx;
    if (idx < 0) idx = it->second.idx_next_loc;
    idx = std::max(0, std::min(idx, static_cast<int>(locs.size()) - 1));
    add_directional_future_flow_route(agent_loc, locs, idx);
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
    int agent_loc, const std::vector<int>& locs, int start_idx) const {
    if (!game_directional_flow_enabled() || !ready_ || locs.empty()) return true;
    const int nz = n24c_ * n24r_;
    if ((int)zone_future_pos_.size() != nz || (int)zone_future_neg_.size() != nz)
        return true;

    const int cap = std::max(0, cfg().game_dir_flow_cap);
    const int gap = std::max(0, cfg().game_dir_flow_gap);
    int prev_m = -1, prev_b = -1;
    project(agent_loc, prev_m, prev_b);
    if (prev_m < 0) return true;

    float decay = cfg().game_dir_flow_decay;
    if (!std::isfinite(decay)) decay = GAME_DIR_FLOW_DECAY;
    decay = std::max(0.0f, std::min(1.0f, decay));
    float cur_weight = 1.0f;
    int last_zid = -1;

    auto zone_allows = [&](const ZoneMainRange& zr, int intent) {
        const int zid = zr.zone_id;
        if (zid < 0 || zid >= nz) return true;
        if (zid == last_zid) return true;
        const float same = (intent > 0) ? zone_future_pos_[zid] : zone_future_neg_[zid];
        const float opp = (intent > 0) ? zone_future_neg_[zid] : zone_future_pos_[zid];
        const float next_same = same + cur_weight;
        last_zid = zid;
        cur_weight *= decay;
        if (next_same >= (float)cap) return false;
        if (gap > 0 && next_same > opp + (float)gap) return false;
        return true;
    };

    for (int i = std::max(0, start_idx); i < static_cast<int>(locs.size()); ++i) {
        int tm = -1, tb = -1;
        project(locs[i], tm, tb);
        if (tm < 0) continue;
        if (tm == prev_m) {
            prev_m = tm;
            prev_b = tb;
            continue;
        }

        const int intent = (tm > prev_m) ? +1 : -1;
        const int lo = std::min(prev_m, tm);
        const int hi = std::max(prev_m, tm);
        if (intent > 0) {
            for (const auto& zr : zone_ranges_) {
                if (zr.min_m > hi || zr.max_m < lo) continue;
                if (!zone_allows(zr, intent)) return false;
            }
        } else {
            for (auto it = zone_ranges_.rbegin(); it != zone_ranges_.rend(); ++it) {
                if (it->min_m > hi || it->max_m < lo) continue;
                if (!zone_allows(*it, intent)) return false;
            }
        }

        prev_m = tm;
        prev_b = tb;
    }
    return true;
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
// compute_zone_widths — compute minimum cross-section width per zone
// Uses DTW (distance-to-wall) to estimate corridor width = 2*max_dtw_in_zone
// ================================================================
void GameBalanceOptScheduler::compute_zone_widths(const SharedEnvironment* env, const std::vector<int>& dtw) {
    int nz = n24c_ * n24r_;
    zone_min_width_.assign(nz, 999);

    // For each zone, find the maximum DTW value among walkable cells
    // The "width" of the zone's narrowest point is approximated by
    // scanning each row within the zone and counting consecutive walkable cells
    for (int zi = 0; zi < nz; zi++) {
        int zone_row = zi / n24c_;
        int zone_col = zi % n24c_;
        int start_r = zone_row * 24;
        int start_c = zone_col * 24;
        int end_r = std::min(start_r + 24, rows_);
        int end_c = std::min(start_c + 24, cols_);

        int min_row_width = 999;
        bool has_walkable = false;
        for (int r = start_r; r < end_r; r++) {
            int consecutive = 0;
            int max_consecutive = 0;
            for (int c = start_c; c < end_c; c++) {
                int loc = r * cols_ + c;
                if (env->map[loc] == 0) {
                    consecutive++;
                    has_walkable = true;
                } else {
                    if (consecutive > max_consecutive) max_consecutive = consecutive;
                    consecutive = 0;
                }
            }
            if (consecutive > max_consecutive) max_consecutive = consecutive;
            if (max_consecutive > 0 && max_consecutive < min_row_width)
                min_row_width = max_consecutive;
        }
        // Also check columns
        for (int c = start_c; c < end_c; c++) {
            int consecutive = 0;
            int max_consecutive = 0;
            for (int r = start_r; r < end_r; r++) {
                int loc = r * cols_ + c;
                if (env->map[loc] == 0) {
                    consecutive++;
                    has_walkable = true;
                } else {
                    if (consecutive > max_consecutive) max_consecutive = consecutive;
                    consecutive = 0;
                }
            }
            if (consecutive > max_consecutive) max_consecutive = consecutive;
            if (max_consecutive > 0 && max_consecutive < min_row_width)
                min_row_width = max_consecutive;
        }
        zone_min_width_[zi] = has_walkable ? min_row_width : 0;
    }
}

// ================================================================
// initialize
// ================================================================
void GameBalanceOptScheduler::initialize(SharedEnvironment* env, int /*preprocess_time_limit_ms*/) {
    env_ = env;
    build_centerline(env);
    pibt_lock().reset();

    // 宽度感知: 计算 zone 宽度
    if (cfg().game_width_aware) {
        std::vector<int> dtw;
        compute_dtw(env, dtw);
        compute_zone_widths(env, dtw);
        auto& lock = pibt_lock();
        int nz2 = n24c_ * n24r_;
        for (int zi = 0; zi < nz2 && zi < PibtLock::MAX_ZONES; zi++)
            lock.zone_width[zi] = zone_min_width_[zi];
    }

    // Sync zone_n24c to PibtLock for PIBT counter-flow
    pibt_lock().zone_n24c = n24c_;

    // agent空闲计数器（IDEA4: idle exempt）
    agent_idle_ticks_.assign(env->num_of_agents, 0);
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

    if (cfg().planner_mode == 16) {
        std::fill(zone_direction_.begin(), zone_direction_.end(), 0);
        for (auto& zr : zone_ranges_) zr.lock_dir = 0;
        auto& lock = pibt_lock();
        std::fill(lock.seg_lock_dir, lock.seg_lock_dir + PibtLock::MAX_SEGMENTS, 0);
        std::fill(lock.zone_dir, lock.zone_dir + PibtLock::MAX_ZONES, 0);
        return;
    }

    int zone_hard_cap = cfg().game_zone_hard_cap;
    int dir_threshold = cfg().game_dir_threshold;

    // IDEA3: 后期放宽zone约束
    int effective_zone_cap = zone_hard_cap;
    int effective_density_cap = cfg().game_density_cap;
    if (cfg().game_late_relax) {
        float progress = (float)step_ / (float)cfg().game_total_steps;
        if (progress > cfg().game_late_phase_ratio) {
            effective_zone_cap = cfg().game_late_zone_cap;
            effective_density_cap = cfg().game_late_density_cap;
        }
    }

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
        if (zid < 0 || zid >= nz || zone24_density_[zid] < effective_zone_cap)
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
        if (zid < 0 || zid >= nz || zone24_density_[zid] < effective_zone_cap) continue;

        // WIDTH_AWARE: 窄zone更严格
        if (cfg().game_width_aware && zid < (int)zone_min_width_.size()) {
            int width = zone_min_width_[zid];
            if (width <= cfg().game_narrow_threshold) {
                int narrow_cap = std::max(2, (int)(width * cfg().game_narrow_cap_ratio));
                if (zone24_density_[zid] < narrow_cap) continue;
            }
        }

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
        if (zone24_density_[zid] >= effective_zone_cap) continue;
        bool nearby_hot = false;
        for (int d = -CONTAIN_EXPAND; d <= CONTAIN_EXPAND; d++) {
            int ni = i + d;
            if (ni < 0 || ni >= (int)zone_ranges_.size()) continue;
            int nzid = zone_ranges_[ni].zone_id;
            if (nzid >= 0 && zone24_density_[nzid] >= effective_zone_cap) {
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
        float dist = std::sqrt((float)((tx - ax)*(tx - ax) + (ty - ay)*(ty - ay)));
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
    if (!directional_future_flow_allows(a_loc, locs, start_idx)) return INVALID_SCORE;

    int zone_hard_cap = cfg().game_zone_hard_cap;
    int exempt_main   = cfg().game_exempt_main;
    int density_cap   = cfg().game_density_cap;
    int route_penalty  = cfg().game_route_penalty;

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
                    int r1 = prev / cols_, c1 = prev % cols_;
                    int r2 = loc / cols_,  c2 = loc % cols_;
                    float dist = std::sqrt((float)((c2-c1)*(c2-c1) + (r2-r1)*(r2-r1)));
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
                int r1 = prev / cols_, c1 = prev % cols_;
                int r2 = locs[ei] / cols_, c2 = locs[ei] % cols_;
                int z2 = (r2 / 24) * n24c_ + (c2 / 24);
                if (z2 >= 0 && z2 < max_z && zone24_density_[z2] >= 70)
                    return INVALID_SCORE;
                float dist = std::sqrt((float)((c2-c1)*(c2-c1) + (r2-r1)*(r2-r1)));
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

    // ---- Default: zone 密度 + 方向锁 打分 ----
    int a_m, a_b;
    project(a_loc, a_m, a_b);
    if (a_m < 0) return 1.0f;

    float total_score = 0;
    int prev_m = a_m, prev_b = a_b;

    for (size_t ei = (size_t)start_idx; ei < locs.size(); ei++) {
        int tm, tb;
        project(locs[ei], tm, tb);
        if (tm < 0) continue;

        int lo = std::min(prev_m, tm);
        int hi = std::max(prev_m, tm);
        int intent = (tm > prev_m) ? +1 : -1;

        int d_max_seg = 0;
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

        float seg_dist = (float)(std::abs(prev_m - tm) + std::abs(prev_b - tb));
        float seg_score = distance_score(seg_dist);

        if (d_max_seg > route_penalty) {
            int idx = std::min(d_max_seg, route_penalty + 20) - route_penalty;
            seg_score = apply_route_density_penalty(seg_score, DIVISOR_TABLE[idx]);
        }

        total_score += seg_score;
        prev_m = tm; prev_b = tb;
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
    } diag;
    if (diag_on) diag.step = step;

    // 时间进度
    float progress = (float)step / (float)cfg().game_total_steps;
    bool is_late_phase = progress > cfg().game_late_phase_ratio;

    // IDEA3: 后期放宽zone约束 — 传递给base_score使用的effective参数
    int eff_density_cap = cfg().game_density_cap;
    if (cfg().game_late_relax && is_late_phase) {
        eff_density_cap = cfg().game_late_density_cap;
    }

    // ---- 分类 agent ----
    std::vector<int> free_agents;
    std::unordered_set<int> assigned_tasks;

    for (int a = 0; a < n_agents; a++) {
        if (env->curr_task_schedule[a] < 0) {
            free_agents.push_back(a);
            agent_idle_ticks_[a]++;
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

    // 搜索半径（IDEA1: 后期扩展）
    int search_radius = cfg().game_search_radius;
    if (cfg().game_late_search_enable && is_late_phase) {
        search_radius = cfg().game_late_search_radius;
    }
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
        if (cfg().game_prefilter_capacity && locs.size() > 1 &&
            [&]() {
                ScoreRejectReason reason = ScoreRejectReason::None;
                const float score =
                    base_score_ex(locs.front(), tid, step, 1, eff_density_cap, &reason);
                if (is_valid_score(score)) return false;
                switch (reason) {
                    case ScoreRejectReason::FutureFlow:
                        diag.prefilter_reject_flow++;
                        break;
                    case ScoreRejectReason::LockDir:
                        diag.prefilter_reject_lock++;
                        break;
                    case ScoreRejectReason::Density:
                        diag.prefilter_reject_density++;
                        break;
                    case ScoreRejectReason::Branch:
                        diag.prefilter_reject_branch++;
                        break;
                    case ScoreRejectReason::Missing:
                    case ScoreRejectReason::Channel:
                    case ScoreRejectReason::Other:
                    case ScoreRejectReason::None:
                        diag.prefilter_reject_other++;
                        break;
                }
                return true;
            }()) {
            diag.prefilter_reject_capacity++;
            continue;
        }
        filtered_free_tasks_set.insert(tid);
    }
    diag.prefilter_task_kept = (int)filtered_free_tasks_set.size();
    int blocked_zone_count = 0;
    if (game_directional_flow_enabled()) {
        const int nz = n24c_ * n24r_;
        const int cap = std::max(0, cfg().game_dir_flow_cap);
        const int gap = std::max(0, cfg().game_dir_flow_gap);
        if (cap > 0 &&
            (int)zone_future_pos_.size() == nz &&
            (int)zone_future_neg_.size() == nz) {
            for (int zid = 0; zid < nz; ++zid) {
                const float pos = zone_future_pos_[zid];
                const float neg = zone_future_neg_[zid];
                const bool pos_blocked = (pos + 1.0f >= (float)cap) ||
                    (gap > 0 && pos + 1.0f > neg + (float)gap);
                const bool neg_blocked = (neg + 1.0f >= (float)cap) ||
                    (gap > 0 && neg + 1.0f > pos + (float)gap);
                if (pos_blocked || neg_blocked)
                    ++blocked_zone_count;
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
        blocked_zone_count
    });

    // 按 zone 分桶 free tasks
    std::unordered_map<int, std::vector<int>> tasks_by_zone;
    for (int tid : filtered_free_tasks_set) {
        auto tit = env->task_pool.find(tid);
        if (tit != env->task_pool.end() && !tit->second.locations.empty())
            tasks_by_zone[zid_100(tit->second.locations[0], cols_)].push_back(tid);
    }
    if (diag_on) diag.task_zones = tasks_by_zone.size();

    // IDEA5: Task Age排序 — 对每个zone的task按age降序
    if (cfg().game_age_sort) {
        for (auto& [z, tids] : tasks_by_zone) {
            std::sort(tids.begin(), tids.end(), [&](int a, int b) {
                auto ait = env->task_pool.find(a);
                auto bit = env->task_pool.find(b);
                int age_a = (ait != env->task_pool.end()) ? (step - ait->second.t_revealed) : 0;
                int age_b = (bit != env->task_pool.end()) ? (step - bit->second.t_revealed) : 0;
                return age_a > age_b; // 等待更久的排前面
            });
        }
    }

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
        if (!directional_future_flow_allows(a_loc, locs, 0)) {
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
        int ar = a_loc / cols_, ac = a_loc % cols_;
        int tr = tloc / cols_, tc = tloc % cols_;
        float dist = std::sqrt((float)((tr - ar) * (tr - ar) + (tc - ac) * (tc - ac)));
        float score = distance_score(dist);
        if (cfg().game_age_sort) {
            float age = (float)(step - tit->second.t_revealed) / (float)cfg().game_total_steps;
            score += cfg().game_age_sort_weight * age;
        }
        return score;
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

    for (int a : free_agents) {
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
                    // IDEA5: age score bonus
                    if (cfg().game_age_sort && bs > -1e8f) {
                        auto tit = env->task_pool.find(tid);
                        if (tit != env->task_pool.end()) {
                            float age = (float)(step - tit->second.t_revealed) / (float)cfg().game_total_steps;
                            bs += cfg().game_age_sort_weight * age;
                        }
                    }
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
                            if (cfg().game_age_sort && bs > -1e8f) {
                                auto tit = env->task_pool.find(tid);
                                if (tit != env->task_pool.end()) {
                                    float age = (float)(step - tit->second.t_revealed) / (float)cfg().game_total_steps;
                                    bs += cfg().game_age_sort_weight * age;
                                }
                            }
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
                  << " neighbor_checked_cap_hits=" << diag.neighbor_checked_cap_hits
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
    if (!directional_future_flow_allows(a_loc, locs, start_idx))
        return reject(ScoreRejectReason::FutureFlow);

    int zone_hard_cap = cfg().game_zone_hard_cap;
    int exempt_main   = cfg().game_exempt_main;
    int density_cap   = eff_density_cap;
    int route_penalty  = cfg().game_route_penalty;

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
                    int r1 = prev / cols_, c1 = prev % cols_;
                    int r2 = loc / cols_,  c2 = loc % cols_;
                    float dist = std::sqrt((float)((c2-c1)*(c2-c1) + (r2-r1)*(r2-r1)));
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
                int r1 = prev / cols_, c1 = prev % cols_;
                int r2 = locs[ei] / cols_, c2 = locs[ei] % cols_;
                int z2 = (r2 / 24) * n24c_ + (c2 / 24);
                if (z2 >= 0 && z2 < max_z && zone24_density_[z2] >= 70)
                    return reject(ScoreRejectReason::Branch);
                float dist = std::sqrt((float)((c2-c1)*(c2-c1) + (r2-r1)*(r2-r1)));
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

    // ---- Default: zone 密度 + 方向锁 打分 ----
    int a_m, a_b;
    project(a_loc, a_m, a_b);
    if (a_m < 0) return 1.0f;

    float total_score = 0;
    int prev_m = a_m, prev_b = a_b;

    for (size_t ei = (size_t)start_idx; ei < locs.size(); ei++) {
        int tm, tb;
        project(locs[ei], tm, tb);
        if (tm < 0) continue;

        int lo = std::min(prev_m, tm);
        int hi = std::max(prev_m, tm);
        int intent = (tm > prev_m) ? +1 : -1;

        int d_max_seg = 0;
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

        float seg_dist = (float)(std::abs(prev_m - tm) + std::abs(prev_b - tb));
        float seg_score = distance_score(seg_dist);

        if (d_max_seg > route_penalty) {
            int idx = std::min(d_max_seg, route_penalty + 20) - route_penalty;
            seg_score = apply_route_density_penalty(seg_score, DIVISOR_TABLE[idx]);
        }

        total_score += seg_score;
        prev_m = tm; prev_b = tb;
    }

    return total_score;
}

} // namespace nz
