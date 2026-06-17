// optsvr/schedule/IronZoneSchedule/IronZoneScheduler.cpp
// IRON mode 10: coarse service-zone assignment with compact diagnostics.
#include "IronZoneScheduler.h"
#include "HeuristicMatrix.h"
#include "RuntimeConfig.h"
#include "lane_data.h"
#include "lane_traffic_light.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <deque>
#include <limits>
#include <numeric>
#include <queue>
#include <thread>
#include <tuple>
#include <unordered_set>

namespace optsvr {

namespace {

constexpr int H_CAP = 100000;

double elapsed_ms(std::chrono::steady_clock::time_point a,
                  std::chrono::steady_clock::time_point b) {
    return std::chrono::duration_cast<std::chrono::microseconds>(b - a).count() / 1000.0;
}

int clamp_int(int v, int lo, int hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

} // namespace

void IronZoneScheduler::initialize(SharedEnvironment* env, int /*preprocess_time_limit_ms*/) {
    env_ = env;
    rows_ = env ? env->rows : 0;
    cols_ = env ? env->cols : 0;
    map_size_ = rows_ * cols_;

    zone_rows_ = std::max(1, cfg().iron_zone_rows);
    zone_cols_ = std::max(1, cfg().iron_zone_cols);
    zone_count_ = zone_rows_ * zone_cols_;
    build_components(env);

    std::fprintf(stderr,
        "[IronZone] init zones=%dx%d comps=%d variant=%d radius=%d..%d topk=%d weights(dist=%.2f,len=%.2f,cross=%.2f,end=%.2f,age=%.2f,balance=%.2f,migrate=%.2f) early(ticks=%d,dist=%.2f,cross=%.2f,long=%.2f) pool(short=%d,starved=%d,th=%d) aggressive(on=%d,iters=%d,probe=%d,margin=%.1f) opp(ratio=%.3f,dist=%d,comp=%d,cross=%d) assign_ratio=%.2f\n",
        zone_rows_, zone_cols_,
        component_count_,
        cfg().iron_assign_variant,
        cfg().iron_local_radius, cfg().iron_max_radius,
        cfg().iron_candidate_topk,
        cfg().iron_dist_weight, cfg().iron_task_len_weight,
        cfg().iron_cross_zone_w, cfg().iron_endpoint_w,
        cfg().iron_age_weight, cfg().iron_balance_w, cfg().iron_migration_w,
        cfg().iron_early_phase_ticks, cfg().iron_early_dist_w,
        cfg().iron_early_cross_w, cfg().iron_early_long_w,
        cfg().iron_short_task_topk, cfg().iron_starved_task_topk,
        cfg().iron_starved_zone_threshold,
        cfg().iron_aggressive_assign ? 1 : 0,
        cfg().iron_aggressive_iters,
        cfg().iron_aggressive_probe,
        cfg().iron_aggressive_replace_margin,
        cfg().iron_opportunity_ratio,
        cfg().iron_opportunity_dist,
        cfg().iron_opportunity_completion,
        cfg().iron_opportunity_cross,
        cfg().iron_assign_ratio);
    
    // Precompute future flow zone adjacency (256×256 independent grid)
    if (cfg().iron_future_flow_lock_th != 0.0f && env) {
        ff_zone_rows_ = std::max(1, cfg().iron_future_flow_zone_rows);
        ff_zone_cols_ = std::max(1, cfg().iron_future_flow_zone_cols);
        ff_zone_count_ = ff_zone_rows_ * ff_zone_cols_;
        ff_dir_.assign(ff_zone_count_ * 4, 0.0f);
        ff_dominant_.assign(ff_zone_count_, -1);
        ff_tide_.assign(ff_zone_count_, 0.0f);
        ff_compute_zone_adjacency(env);
        // Load or compute all-pairs cell-level BFS zone paths
        if (!ff_load_cache("ff_cache.bin")) {
            ff_precompute_all_pairs(env);
            ff_save_cache("ff_cache.bin");
        }
    }
}

int IronZoneScheduler::zone_of_loc(int loc) const {
    if (loc < 0 || loc >= map_size_ || cols_ <= 0) return -1;
    int r = loc / cols_;
    int c = loc % cols_;
    int zr = std::min(zone_rows_ - 1,
                      static_cast<int>((long long)r * zone_rows_ / std::max(1, rows_)));
    int zc = std::min(zone_cols_ - 1,
                      static_cast<int>((long long)c * zone_cols_ / std::max(1, cols_)));
    return zr * zone_cols_ + zc;
}

// ===================================================================
// Future flow: zone adjacency + BFS path (independent 256×256 zone grid)
// ===================================================================
void IronZoneScheduler::ff_compute_zone_adjacency(SharedEnvironment* env) {
    if (ff_zone_count_ <= 0) return;
    ff_zone_adj_.assign(ff_zone_count_, std::vector<int>());
    
    int cell_h = std::max(1, rows_ / std::max(1, ff_zone_rows_));
    int cell_w = std::max(1, cols_ / std::max(1, ff_zone_cols_));
    
    for (int zr = 0; zr < ff_zone_rows_; zr++) {
        for (int zc = 0; zc < ff_zone_cols_; zc++) {
            int z = zr * ff_zone_cols_ + zc;
            
            // Right neighbor
            if (zc + 1 < ff_zone_cols_) {
                int c_border = (zc + 1) * cell_w;
                if (c_border >= cols_) c_border = cols_ - 1;
                int c_left = c_border - 1;
                int r_start = zr * cell_h;
                int r_end = std::min((zr + 1) * cell_h, rows_);
                bool connected = false;
                for (int r = r_start; r < r_end && !connected; r++) {
                    int loc_left = r * cols_ + c_left;
                    int loc_right = r * cols_ + c_border;
                    if (c_left >= 0 && c_border < cols_ &&
                        env->map[loc_left] == 0 && env->map[loc_right] == 0)
                        connected = true;
                }
                if (connected) {
                    ff_zone_adj_[z].push_back(zr * ff_zone_cols_ + (zc + 1));
                    ff_zone_adj_[zr * ff_zone_cols_ + (zc + 1)].push_back(z);
                }
            }
            // Down neighbor
            if (zr + 1 < ff_zone_rows_) {
                int r_border = (zr + 1) * cell_h;
                if (r_border >= rows_) r_border = rows_ - 1;
                int r_top = r_border - 1;
                int c_start = zc * cell_w;
                int c_end = std::min((zc + 1) * cell_w, cols_);
                bool connected = false;
                for (int c = c_start; c < c_end && !connected; c++) {
                    int loc_top = r_top * cols_ + c;
                    int loc_bottom = r_border * cols_ + c;
                    if (r_top >= 0 && r_border < rows_ &&
                        env->map[loc_top] == 0 && env->map[loc_bottom] == 0)
                        connected = true;
                }
                if (connected) {
                    ff_zone_adj_[z].push_back((zr + 1) * ff_zone_cols_ + zc);
                    ff_zone_adj_[(zr + 1) * ff_zone_cols_ + zc].push_back(z);
                }
            }
        }
    }
}

// ===================================================================
// All-pairs cell-level BFS precomputation (offline, persisted)
// ===================================================================
void IronZoneScheduler::ff_precompute_all_pairs(SharedEnvironment* env) {
    if (ff_zone_count_ <= 0) return;
    const int count = ff_zone_count_;
    std::fprintf(stderr, "[IronZone] Precomputing all-pairs zone paths (cell BFS)...\n");
    auto t0 = std::chrono::steady_clock::now();
    
    ff_path_next_.assign(count * count, 65535);
    ff_path_dist_.assign(count * count, 65535);
    
    // Collect walkable cells per zone
    std::vector<std::vector<int>> zone_cells(count);
    for (int loc = 0; loc < map_size_; loc++) {
        if (env->map[loc] == 0) {
            int z = ff_zone_of_loc(loc);
            if (z >= 0 && z < count) zone_cells[z].push_back(loc);
        }
    }
    
    const int dr[4] = {-1, 1, 0, 0};
    const int dc[4] = {0, 0, -1, 1};
    
    for (int z_src = 0; z_src < count; z_src++) {
        if (zone_cells[z_src].empty()) continue;
        
        std::vector<uint16_t> dist(map_size_, 65535);
        std::vector<uint16_t> exit_zone(map_size_, 65535);
        std::deque<int> q;
        
        for (int loc : zone_cells[z_src]) {
            dist[loc] = 0;
            exit_zone[loc] = z_src;
            q.push_back(loc);
        }
        
        ff_path_next_[z_src * count + z_src] = z_src;
        ff_path_dist_[z_src * count + z_src] = 0;
        std::vector<bool> reached(count, false);
        reached[z_src] = true;
        
        while (!q.empty()) {
            int cur = q.front(); q.pop_front();
            int cur_z = ff_zone_of_loc(cur);
            uint16_t cd = dist[cur] + 1;
            
            int r = cur / cols_, c = cur % cols_;
            for (int d = 0; d < 4; d++) {
                int nr = r + dr[d], nc = c + dc[d];
                if (nr < 0 || nr >= rows_ || nc < 0 || nc >= cols_) continue;
                int nxt = nr * cols_ + nc;
                if (env->map[nxt] != 0) continue;
                if (cd >= dist[nxt]) continue;
                
                dist[nxt] = cd;
                int nz = ff_zone_of_loc(nxt);
                exit_zone[nxt] = (cur_z == z_src && nz != z_src) ? nz : exit_zone[cur];
                
                if (!reached[nz]) {
                    reached[nz] = true;
                    ff_path_next_[z_src * count + nz] = exit_zone[nxt];
                    ff_path_dist_[z_src * count + nz] = cd;
                }
                q.push_back(nxt);
            }
        }
        
        if ((z_src + 1) % 500 == 0) {
            auto t = std::chrono::steady_clock::now();
            float elapsed = std::chrono::duration<float>(t - t0).count();
            std::fprintf(stderr, "  [IronZone] BFS %d/%d (%.1fs)\n", z_src+1, count, elapsed);
        }
    }
    
    ff_path_ready_ = true;
    auto t1 = std::chrono::steady_clock::now();
    float total = std::chrono::duration<float>(t1 - t0).count();
    std::fprintf(stderr, "[IronZone] All-pairs BFS done in %.1fs\n", total);
}

void IronZoneScheduler::ff_save_cache(const char* path) const {
    FILE* f = std::fopen(path, "wb");
    if (!f) { std::fprintf(stderr, "[IronZone] Cannot write cache: %s\n", path); return; }
    int count = ff_zone_count_;
    std::fwrite(&count, sizeof(int), 1, f);
    std::fwrite(ff_path_next_.data(), sizeof(uint16_t), count * count, f);
    std::fwrite(ff_path_dist_.data(), sizeof(uint16_t), count * count, f);
    std::fclose(f);
    std::fprintf(stderr, "[IronZone] Cache saved (%d zones, %.1fMB)\n", count, 
        3.0 * count * count * sizeof(uint16_t) / (1024.0 * 1024.0));
}

bool IronZoneScheduler::ff_load_cache(const char* path) {
    FILE* f = std::fopen(path, "rb");
    if (!f) return false;
    int count;
    std::fread(&count, sizeof(int), 1, f);
    if (count != ff_zone_count_) { std::fclose(f); return false; }
    ff_path_next_.resize(count * count);
    ff_path_dist_.resize(count * count);
    std::fread(ff_path_next_.data(), sizeof(uint16_t), count * count, f);
    std::fread(ff_path_dist_.data(), sizeof(uint16_t), count * count, f);
    std::fclose(f);
    ff_path_ready_ = true;
    std::fprintf(stderr, "[IronZone] Cache loaded (%d zones)\n", count);
    return true;
}

std::vector<int> IronZoneScheduler::ff_get_zone_path_fast(int z_from, int z_to) const {
    std::vector<int> path;
    if (!ff_path_ready_ || z_from < 0 || z_to < 0 || 
        z_from >= ff_zone_count_ || z_to >= ff_zone_count_) return path;
    int cur = z_from;
    path.push_back(cur);
    while (cur != z_to) {
        cur = ff_path_next_[cur * ff_zone_count_ + z_to];
        if (cur >= ff_zone_count_) { path.clear(); return path; }
        path.push_back(cur);
    }
    return path;
}

int IronZoneScheduler::ff_zone_of_loc(int loc) const {
    if (loc < 0 || loc >= map_size_ || cols_ <= 0) return -1;
    int r = loc / cols_;
    int c = loc % cols_;
    int zr = std::min(ff_zone_rows_ - 1,
                      static_cast<int>((long long)r * ff_zone_rows_ / std::max(1, rows_)));
    int zc = std::min(ff_zone_cols_ - 1,
                      static_cast<int>((long long)c * ff_zone_cols_ / std::max(1, cols_)));
    return zr * ff_zone_cols_ + zc;
}

std::vector<int> IronZoneScheduler::ff_get_zone_path(int z_from, int z_to) {
    if (z_from < 0 || z_to < 0 || z_from >= ff_zone_count_ || z_to >= ff_zone_count_)
        return {};
    if (z_from == z_to) return {z_from};
    
    // Use precomputed cell-level BFS table when available (much faster)
    if (ff_path_ready_) {
        return ff_get_zone_path_fast(z_from, z_to);
    }
    
    if (ff_bfs_cached_source_ != z_from) {
        ff_bfs_dist_.assign(ff_zone_count_, 65535);
        ff_bfs_parent_.assign(ff_zone_count_, -1);
        std::deque<int> q;
        ff_bfs_dist_[z_from] = 0;
        ff_bfs_parent_[z_from] = z_from;
        q.push_back(z_from);
        while (!q.empty()) {
            int cur = q.front(); q.pop_front();
            uint16_t nd = ff_bfs_dist_[cur] + 1;
            for (int nz : ff_zone_adj_[cur]) {
                if (nd < ff_bfs_dist_[nz]) {
                    ff_bfs_dist_[nz] = nd;
                    ff_bfs_parent_[nz] = cur;
                    q.push_back(nz);
                }
            }
        }
        ff_bfs_cached_source_ = z_from;
    }
    
    if (ff_bfs_parent_[z_to] < 0) return {};
    std::vector<int> path;
    int cur = z_to;
    while (cur != z_from) {
        path.push_back(cur);
        cur = ff_bfs_parent_[cur];
    }
    path.push_back(z_from);
    std::reverse(path.begin(), path.end());
    return path;
}

int IronZoneScheduler::ff_dir_from_delta(int dr, int dc) const {
    if (dr == 0 && dc == 0) return -1;
    if (std::abs(dr) >= std::abs(dc))
        return dr > 0 ? DIR_S : DIR_N;
    else
        return dc > 0 ? DIR_E : DIR_W;
}

void IronZoneScheduler::ff_add_future_flow_path(int z_from, int z_to, float weight,
                                                  int cumul_offset) {
    if (z_from == z_to) return;
    auto path = ff_path_ready_ ? ff_get_zone_path_fast(z_from, z_to) : ff_get_zone_path(z_from, z_to);
    if (path.empty()) return;
    int cz = ff_cell_size();
    
    // 遍历路径，记录每段方向
    for (size_t i = 0; i < path.size(); i++) {
        int z = path[i];
        int cumul = (cumul_offset + static_cast<int>(i)) * cz;
        float decay = 1.0f / (1.0f + static_cast<float>(cumul) * cfg().iron_future_flow_decay);
        float delta = cfg().iron_future_flow_k * weight * decay;
        
        if (i == 0) {
            // 起点zone：根据出方向记录
            int nr = path[1] / ff_zone_cols_, nc = path[1] % ff_zone_cols_;
            int cr = z / ff_zone_cols_, cc = z % ff_zone_cols_;
            int d = ff_dir_from_delta(nr - cr, nc - cc);
            if (d >= 0) ff_dir_[z * 4 + d] += delta;
        } else if (i == path.size() - 1) {
            // 终点zone：根据入方向记录
            int pr = path[i-1] / ff_zone_cols_, pc = path[i-1] % ff_zone_cols_;
            int cr = z / ff_zone_cols_, cc = z % ff_zone_cols_;
            int d = ff_dir_from_delta(cr - pr, cc - pc);
            if (d >= 0) ff_dir_[z * 4 + d] += delta;
        } else {
            // 中间zone：入方向和出方向应一致（直线路径）
            int pr = path[i-1] / ff_zone_cols_, pc = path[i-1] % ff_zone_cols_;
            int nr = path[i+1] / ff_zone_cols_, nc = path[i+1] % ff_zone_cols_;
            int cr = z / ff_zone_cols_, cc = z % ff_zone_cols_;
            int d = ff_dir_from_delta(cr - pr, cc - pc);
            if (d >= 0) ff_dir_[z * 4 + d] += delta;
        }
    }
}

void IronZoneScheduler::ff_update_tide() {
    if (ff_zone_count_ <= 0) return;
    const float min_flow = 0.5f;
    const float tide_inc = 0.03f;
    const float tide_cap = 0.3f;
    const float tide_decay = 0.9f;
    for (int z = 0; z < ff_zone_count_; z++) {
        float* d = &ff_dir_[z * 4];
        float total = d[0] + d[1] + d[2] + d[3];
        if (total < min_flow) {
            ff_tide_[z] *= tide_decay;
            if (ff_tide_[z] < 0.01f) {
                ff_dominant_[z] = -1;
                ff_tide_[z] = 0.0f;
            }
            continue;
        }
        int best_dir = 0;
        if (d[1] > d[best_dir]) best_dir = 1;
        if (d[2] > d[best_dir]) best_dir = 2;
        if (d[3] > d[best_dir]) best_dir = 3;
        int opp = (best_dir + 2) % 4;
        float ratio = d[best_dir] / std::max(0.1f, d[opp]);
        
        if (ratio > 1.3f && ff_dominant_[z] == best_dir) {
            // 持续同方向：缓慢增强
            ff_tide_[z] = std::min(tide_cap, ff_tide_[z] + tide_inc);
        } else if (ratio > 1.3f) {
            // 新方向：初始潮汐
            ff_tide_[z] = tide_inc;
            ff_dominant_[z] = best_dir;
        } else {
            // 方向趋于均衡：衰减
            ff_tide_[z] *= tide_decay;
            if (ff_tide_[z] < 0.01f) {
                ff_dominant_[z] = -1;
                ff_tide_[z] = 0.0f;
            }
        }
    }
}

int IronZoneScheduler::component_of_loc(int loc) const {
    if (loc < 0 || loc >= static_cast<int>(component_id_.size())) return -1;
    return component_id_[loc];
}

void IronZoneScheduler::build_components(SharedEnvironment* env) {
    component_count_ = 0;
    component_id_.assign(map_size_, -1);
    if (!env || map_size_ <= 0 || cols_ <= 0) return;

    std::deque<int> q;
    for (int loc = 0; loc < map_size_; loc++) {
        if (env->map[loc] != 0 || component_id_[loc] != -1) continue;

        const int cid = component_count_++;
        component_id_[loc] = cid;
        q.push_back(loc);

        while (!q.empty()) {
            int cur = q.front();
            q.pop_front();
            int r = cur / cols_;
            int c = cur % cols_;

            auto push_if_free = [&](int next) {
                if (next < 0 || next >= map_size_) return;
                if (env->map[next] != 0 || component_id_[next] != -1) return;
                component_id_[next] = cid;
                q.push_back(next);
            };

            if (r > 0) push_if_free(cur - cols_);
            if (r + 1 < rows_) push_if_free(cur + cols_);
            if (c > 0) push_if_free(cur - 1);
            if (c + 1 < cols_) push_if_free(cur + 1);
        }
    }
}

bool IronZoneScheduler::task_remaining_single_component(
    const Task& task, int start_idx, int* component) const
{
    if (component) *component = -1;
    if (start_idx < 0 || start_idx >= static_cast<int>(task.locations.size())) {
        return false;
    }

    int cid = -1;
    for (int i = start_idx; i < static_cast<int>(task.locations.size()); i++) {
        int loc = task.locations[i];
        int loc_component = component_of_loc(loc);
        if (loc_component < 0) return false;
        if (cid < 0) {
            cid = loc_component;
        } else if (loc_component != cid) {
            return false;
        }
    }
    if (component) *component = cid;
    return cid >= 0;
}

int IronZoneScheduler::zone_distance(int za, int zb) const {
    if (za < 0 || zb < 0) return zone_rows_ + zone_cols_;
    int ar = za / zone_cols_;
    int ac = za % zone_cols_;
    int br = zb / zone_cols_;
    int bc = zb % zone_cols_;
    return std::abs(ar - br) + std::abs(ac - bc);
}

int IronZoneScheduler::distance_fast(SharedEnvironment* env, int source, int target) const {
    if (!env || source < 0 || target < 0 ||
        source >= map_size_ || target >= map_size_) {
        return H_CAP;
    }
    if (!heuristic_matrix().is_ready()) {
        int sr = source / cols_;
        int sc = source % cols_;
        int tr = target / cols_;
        int tc = target % cols_;
        return std::abs(sr - tr) + std::abs(sc - tc);
    }
    int h = get_h_fast(env, source, target);
    return h >= H_CAP ? H_CAP : h;
}

int IronZoneScheduler::effective_tick(SharedEnvironment* env) const {
    if (!env) return 0;
    if (env->system_timestep >= 0) return env->system_timestep;
    return env->curr_timestep;
}

float IronZoneScheduler::early_phase_factor() const {
    int phase_ticks = cfg().iron_early_phase_ticks;
    if (!env_ || phase_ticks <= 0) return 0.0f;
    int tick = effective_tick(env_);
    if (tick >= phase_ticks) return 0.0f;
    int remaining = phase_ticks - tick;
    return static_cast<float>(remaining) / static_cast<float>(phase_ticks);
}

int IronZoneScheduler::first_remaining_index(const Task& task) const {
    if (task.locations.empty()) return -1;
    int idx = task.idx_next_loc;
    if (idx < 0 || idx >= static_cast<int>(task.locations.size())) idx = 0;
    return idx;
}

int IronZoneScheduler::remaining_task_length(SharedEnvironment* env, const Task& task) const {
    int idx = first_remaining_index(task);
    if (idx < 0) return 0;
    int length = 0;
    int prev = task.locations[idx];
    for (int i = idx + 1; i < static_cast<int>(task.locations.size()); i++) {
        int next = task.locations[i];
        length += distance_fast(env, prev, next);
        prev = next;
    }
    return length;
}

float IronZoneScheduler::score_candidate(
    const AgentMeta& agent,
    const TaskMeta& task,
    const std::vector<int>& finish_zone_pressure,
    const std::vector<int>& zone_shortage) const
{
    int dist = distance_fast(env_, agent.loc, task.start);

    // 干净打分：只有距离 + task长度
    float score = cfg().iron_dist_weight * static_cast<float>(dist)
                + cfg().iron_task_len_weight * static_cast<float>(task.length);
    (void)finish_zone_pressure;
    (void)zone_shortage;

    // 方向锁：全局预过滤标记的逆流task直接拦
    if (cfg().iron_future_flow_lock_th != 0.0f && !ff_dir_.empty()) {
        if (task.ff_blocked) {
            ff_locked_candidates_++;
            return 1e9f;
        }
    }
    return score;
}

void IronZoneScheduler::print_round_stats(const RoundStats& s, int tick) const {
    // CSV log
    static FILE* csv = nullptr;
    if (!csv) {
        csv = std::fopen("ironzone_debug.csv", "w");
        if (csv) std::fprintf(csv, "tick,assigned,busy,unmatched,cand,freeT,avgDist,avgLen,rounds,build_ms,solve_ms,1dir,2opp,2adj,3dir,4dir,active,avgHeat,maxHeat,maxFlow,maxWith,maxOpp,nN,nS,nE,nW,anyN,anyS,anyE,anyW,avgN,avgS,avgE,avgW,locked,zGe5,zGe10,zGe15\n");
    }
    if (csv) {
        double avg_cand = s.candidate_agents > 0
            ? static_cast<double>(s.candidate_total) / s.candidate_agents : 0.0;
        std::fprintf(csv, "%d,%d,%d,%d,%.1f,%d,%.1f,%.1f,%d,%.1f,%.1f\n",
            tick, s.assigned, s.busy_before + s.assigned, s.unmatched_agents,
            avg_cand, s.free_tasks,
            s.assigned > 0 ? static_cast<double>(s.dist_sum) / s.assigned : 0.0,
            s.assigned > 0 ? static_cast<double>(s.len_sum) / s.assigned : 0.0,
            s.aggressive_rounds, s.build_ms, s.solve_ms);
        std::fflush(csv);
    }

    double avg_dist = s.assigned > 0 ? static_cast<double>(s.dist_sum) / s.assigned : 0.0;
    double avg_len = s.assigned > 0 ? static_cast<double>(s.len_sum) / s.assigned : 0.0;
    double avg_zd = s.assigned > 0 ? static_cast<double>(s.zone_dist_sum) / s.assigned : 0.0;
    double avg_completion = s.assigned > 0
        ? static_cast<double>(s.completion_cost_sum) / s.assigned
        : 0.0;
    double avg_cand = s.candidate_agents > 0
        ? static_cast<double>(s.candidate_total) / s.candidate_agents
        : 0.0;
    double avg_start_shortage = s.assigned > 0
        ? static_cast<double>(s.start_shortage_sum) / s.assigned
        : 0.0;
    double avg_finish_shortage = s.assigned > 0
        ? static_cast<double>(s.finish_shortage_sum) / s.assigned
        : 0.0;
    double avg_migration_gain = s.assigned > 0
        ? static_cast<double>(s.migration_gain_sum) / s.assigned
        : 0.0;
    double cross_pct = s.assigned > 0 ? 100.0 * s.cross_zone / s.assigned : 0.0;
    double long_pct = s.assigned > 0 ? 100.0 * s.long_tasks / s.assigned : 0.0;
    int busy_after = s.busy_before + s.assigned;

    std::fprintf(stderr,
        "[IronZone] tick=%d phase=%.2f variant=%d target=%d(base=%d) freeA=%d freeT=%d assigned=%d busy=%d unmatched=%d opp(cap=%d,extra=%d,reject=%d) avgDist=%.1f avgLen=%.1f avgComp=%.1f cross=%.1f%% avgZ=%.2f shortage(start=%.1f,finish=%.1f,gain=%.1f,zones=%d) pool(short=%d,starved=%d) skip(comp=%d,empty=%d,quota=%d,gate=%d) freeLen(<=500=%d,<=1000=%d,<=1500=%d,>1500=%d) assignLen(<=500=%d,<=1000=%d,<=1500=%d,>1500=%d) assignDist(<=50=%d,<=150=%d,>150=%d) long>=%d=%.1f%% cand=%.1f opt(rounds=%d,seed=%d,replace=%d,swap=%d) heap(pops=%d,skip=%d,fb=%d,gfb=%d) time(build=%.1fms,solve=%.1fms)\n",
        tick, s.early_phase, s.assign_variant, s.target_active, s.base_target_active,
        s.free_agents, s.free_tasks, s.assigned, busy_after, s.unmatched_agents,
        s.opportunity_cap, s.opportunity_assigned, s.opportunity_rejected,
        avg_dist, avg_len, avg_completion, cross_pct, avg_zd,
        avg_start_shortage, avg_finish_shortage, avg_migration_gain,
        s.starved_zones, s.short_pool, s.starved_pool,
        s.skipped_component, s.candidate_empty_agents, s.quota_skips, s.gate_skips,
        s.free_len_short, s.free_len_mid, s.free_len_long, s.free_len_huge,
        s.assign_len_short, s.assign_len_mid, s.assign_len_long, s.assign_len_huge,
        s.assign_dist_near, s.assign_dist_mid, s.assign_dist_far,
        cfg().iron_long_task_th, long_pct, avg_cand,
        s.aggressive_rounds, s.aggressive_seeded,
        s.aggressive_replacements, s.aggressive_swaps,
        s.heap_pops, s.lazy_skips, s.fallback_assigned, s.global_pool_assigned,
        s.build_ms, s.solve_ms);
    
    // FF stats
    if (!ff_dir_.empty() && ff_zone_count_ > 0) {
        int active_zones = 0;
        double avg_heat = 0.0, max_heat = 0.0;
        float max_flow = 0.0f, max_flow_with = 0.0f, max_flow_opp = 0.0f;
        int one_dir = 0, two_opp = 0, two_adj = 0, three_dir = 0, four_dir = 0;
        int nN = 0, nS = 0, nE = 0, nW = 0;
        int anyN = 0, anyS = 0, anyE = 0, anyW = 0;
        double sumN = 0, sumS = 0, sumE = 0, sumW = 0;
        for (int z = 0; z < ff_zone_count_; z++) {
            float d0 = ff_dir_[z * 4], d1 = ff_dir_[z * 4 + 1];
            float d2 = ff_dir_[z * 4 + 2], d3 = ff_dir_[z * 4 + 3];
            float total = d0 + d1 + d2 + d3;
            if (total > 0.5f) {
                active_zones++;
                float heat = total / (total + 10.0f);
                avg_heat += heat;
                if (heat > max_heat) max_heat = heat;
                if (total > max_flow) max_flow = total;
                // Count non-zero directions
                int ndir = (d0 > 0.5f) + (d1 > 0.5f) + (d2 > 0.5f) + (d3 > 0.5f);
                if (ndir == 1) one_dir++;
                else if (ndir == 2) {
                    if ((d0 > 0.5f && d2 > 0.5f) || (d1 > 0.5f && d3 > 0.5f))
                        two_opp++;
                    else
                        two_adj++;
                } else if (ndir == 3) three_dir++;
                else if (ndir >= 4) four_dir++;
                if (d0 > 0.5f && d2 > 0.5f) {
                    float with = std::max(d0, d2);
                    float opp = std::min(d0, d2);
                    if (with > max_flow_with) max_flow_with = with;
                    if (opp > max_flow_opp) max_flow_opp = opp;
                }
                if (d1 > 0.5f && d3 > 0.5f) {
                    float with = std::max(d1, d3);
                    float opp = std::min(d1, d3);
                    if (with > max_flow_with) max_flow_with = with;
                    if (opp > max_flow_opp) max_flow_opp = opp;
                }
            }
            // Per-direction counts (any flow > 0.01, strong flow > 0.5)
            if (d0 > 0.01f) { anyE++; sumE += d0; if (d0 > 0.5f) nE++; }
            if (d1 > 0.01f) { anyS++; sumS += d1; if (d1 > 0.5f) nS++; }
            if (d2 > 0.01f) { anyW++; sumW += d2; if (d2 > 0.5f) nW++; }
            if (d3 > 0.01f) { anyN++; sumN += d3; if (d3 > 0.5f) nN++; }
        }
        if (active_zones > 0) avg_heat /= active_zones;
        // Zone lock distribution
        int z_ge5=0, z_ge10=0, z_ge15=0;
        for (int z = 0; z < ff_zone_count_; z++) {
            float e = ff_dir_[z*4], s = ff_dir_[z*4+1];
            float w = ff_dir_[z*4+2], n = ff_dir_[z*4+3];
            float total = e + s + w + n;
            if (total < 0.5f) continue;
            float opp = std::max({std::min(e,s), std::min(w,e), std::min(n,s)});
            if (opp >= 5) z_ge5++;
            if (opp >= 10) z_ge10++;
            if (opp >= 15) z_ge15++;
        }
        if (csv) {
            std::fprintf(csv, ",%d,%d,%d,%d,%d,%d,%.3f,%.3f,%.1f,%.1f,%.1f,%d,%d,%d,%d,%d,%d,%d,%d,%.2f,%.2f,%.2f,%.2f,%d,%d,%d,%d\n",
                one_dir, two_opp, two_adj, three_dir, four_dir,
                active_zones, avg_heat, max_heat,
                max_flow, max_flow_with, max_flow_opp,
                nN, nS, nE, nW,
                anyN, anyS, anyE, anyW,
                nN>0?sumN/nN:0, nS>0?sumS/nS:0, nE>0?sumE/nE:0, nW>0?sumW/nW:0,
                ff_locked_candidates_, z_ge5, z_ge10, z_ge15);
            std::fflush(csv);
        }
        std::fprintf(stderr,
            " [FF] active=%d avgHeat=%.3f maxHeat=%.3f 1dir=%d 2opp=%d 2adj=%d 3dir=%d 4dir=%d | N=%d S=%d E=%d W=%d any(N=%d S=%d E=%d W=%d) locked=%d\n",
            active_zones, avg_heat, max_heat,
            one_dir, two_opp, two_adj, three_dir, four_dir,
            nN, nS, nE, nW, anyN, anyS, anyE, anyW,
            ff_locked_candidates_);
    }
}

void IronZoneScheduler::schedule(int time_limit_ms,
                                 std::vector<int>& proposed_schedule,
                                 SharedEnvironment* env) {
    if (!env || env->num_of_agents <= 0) return;
    env_ = env;
    const int cur_tick = effective_tick(env);

    auto start_time = std::chrono::steady_clock::now();
    auto deadline = start_time + std::chrono::milliseconds(std::max(1, time_limit_ms));

    const int n_agents = env->num_of_agents;
    if (static_cast<int>(proposed_schedule.size()) < n_agents ||
        static_cast<int>(env->curr_states.size()) < n_agents) {
        return;
    }

    try {
        task_pool_snapshot_ = env->task_pool;
        std::vector<State> states_copy = env->curr_states;
        curr_locations_snapshot_.assign(n_agents, -1);
        for (int a = 0; a < n_agents && a < static_cast<int>(states_copy.size()); a++) {
            curr_locations_snapshot_[a] = states_copy[a].location;
        }
        schedule_snapshot_.assign(proposed_schedule.begin(), proposed_schedule.end());
    } catch (...) {
        std::fprintf(stderr, "[IronZone] snapshot copy failed, skipping schedule round\n");
        return;
    }

    RoundStats stats;
    stats.early_phase = early_phase_factor();
    stats.assign_variant = cfg().iron_assign_variant;
    stats.free_agents = 0;
    
    // 每tick清零红绿灯免名单
    if (cfg().iron_lane_enabled)
        std::fill(IronLane::g_exempt_from_traffic_light, IronLane::g_exempt_from_traffic_light + n_agents, 0);
    
    std::vector<int> zone_agent_supply(zone_count_, 0);
    for (int a = 0; a < n_agents; a++) {
        int z = zone_of_loc(curr_locations_snapshot_[a]);
        if (z >= 0 && z < zone_count_) zone_agent_supply[z]++;
    }
    std::vector<int> zone_free_agents(zone_count_, 0);

    std::vector<AgentMeta> free_agents;
    free_agents.reserve(n_agents);
    std::unordered_set<int> assigned_tasks;
    assigned_tasks.reserve(n_agents * 2 + 1);

    for (int a = 0; a < n_agents; a++) {
        int tid = schedule_snapshot_[a];
        if (tid < 0) {
            int loc = curr_locations_snapshot_[a];
            int z = zone_of_loc(loc);
            int comp = component_of_loc(loc);
            if (loc >= 0 && loc < map_size_ && z >= 0 && comp >= 0) {
                free_agents.push_back({a, loc, z, comp});
                zone_free_agents[z]++;
            }
        } else {
            assigned_tasks.insert(tid);
            stats.busy_before++;
        }
    }
    stats.free_agents = static_cast<int>(free_agents.size());

    std::vector<TaskMeta> tasks;
    tasks.reserve(task_pool_snapshot_.size());
    std::vector<std::vector<int>> tasks_by_zone(zone_count_);
    std::vector<int> zone_free_tasks(zone_count_, 0);
    std::vector<int> finish_zone_pressure = zone_agent_supply;

    for (const auto& [tid, task] : task_pool_snapshot_) {
        if (task.locations.empty()) continue;
        int idx = first_remaining_index(task);
        if (idx < 0) continue;
        int task_component = -1;
        if (!task_remaining_single_component(task, idx, &task_component)) {
            stats.skipped_component++;
            continue;
        }
        int finish = task.locations.back();
        int finish_zone = zone_of_loc(finish);

        if (assigned_tasks.find(tid) != assigned_tasks.end()) {
            if (finish_zone >= 0 && finish_zone < zone_count_) {
                finish_zone_pressure[finish_zone]++;
            }
            continue;
        }

        int start = task.locations[idx];
        int start_zone = zone_of_loc(start);
        if (start_zone < 0 || finish_zone < 0) continue;

        TaskMeta meta;
        meta.tid = tid;
        meta.start = start;
        meta.finish = finish;
        meta.start_zone = start_zone;
        meta.finish_zone = finish_zone;
        meta.component = task_component;
        meta.length = remaining_task_length(env, task);
        meta.age = std::max(0, cur_tick - task.t_revealed);
        if (meta.length <= 500) stats.free_len_short++;
        else if (meta.length <= 1000) stats.free_len_mid++;
        else if (meta.length <= 1500) stats.free_len_long++;
        else stats.free_len_huge++;

        int task_index = static_cast<int>(tasks.size());
        tasks.push_back(meta);
        tasks_by_zone[start_zone].push_back(task_index);
        zone_free_tasks[start_zone]++;
    }
    stats.free_tasks = static_cast<int>(tasks.size());

    // ---- pretick: reset future flow + add flow from busy agents ----
    std::vector<std::vector<int>> agents_by_zone;  // only built when ff enabled
    bool ff_enabled = (cfg().iron_future_flow_lock_th != 0.0f && !ff_dir_.empty());
    
    if (ff_enabled) {
        ff_locked_candidates_ = 0;
        ff_dir_.assign(ff_zone_count_ * 4, 0.0f);
        ff_bfs_cached_source_ = -1;
        
        // Build agents_by_zone from free_agents (using FF zone grid)
        agents_by_zone.resize(ff_zone_count_);
        for (int ai = 0; ai < static_cast<int>(free_agents.size()); ai++) {
            int z = ff_zone_of_loc(free_agents[ai].loc);
            if (z >= 0 && z < ff_zone_count_) {
                agents_by_zone[z].push_back(ai);
            }
        }
        
        // Pretick: add flow from agents with ongoing tasks
        for (int a = 0; a < n_agents; a++) {
            int tid = schedule_snapshot_[a];
            if (tid < 0) continue;
            int a_loc = curr_locations_snapshot_[a];
            if (a_loc < 0 || a_loc >= map_size_) continue;
            auto it = task_pool_snapshot_.find(tid);
            if (it == task_pool_snapshot_.end() || it->second.locations.empty()) continue;
            const auto& locs = it->second.locations;
            
            // 用first_remaining_index找到第一个未访问waypoint
            int first_idx = first_remaining_index(it->second);
            if (first_idx < 0) continue;
            // 从agent_loc开始，一段段经过未完成waypoint到finish，累积距离decay
            int z_prev = ff_zone_of_loc(a_loc);
            int cumul = 0;
            for (int i = first_idx; i < static_cast<int>(locs.size()); i++) {
                int z_curr = ff_zone_of_loc(locs[i]);
                ff_add_future_flow_path(z_prev, z_curr, cfg().iron_future_flow_w, cumul);
                cumul += static_cast<int>(ff_get_zone_path(z_prev, z_curr).size());
                z_prev = z_curr;
            }
        }
        // Precompute per-zone flow stats (info only)
        int n_active_zone = 0;
        for (int z = 0; z < ff_zone_count_; z++) {
            float e = ff_dir_[z*4], s = ff_dir_[z*4+1];
            float w = ff_dir_[z*4+2], n = ff_dir_[z*4+3];
            float t = e + s + w + n;
            if (t >= 0.5f) n_active_zone++;
        }
        std::fprintf(stderr, "[IronZone] pretick: zones=%d active=%d ff_path_ready_=%d\\n",
            ff_zone_count_, n_active_zone, ff_path_ready_ ? 1 : 0);
    }
    
    // ---- 全局预过滤：方向锁 + 拥塞上限 ----
    if (ff_enabled && ff_path_ready_ &&
        (cfg().iron_future_flow_lock_th != 0.0f || cfg().iron_future_flow_congest_cap > 0.0f)) {
        float lth = cfg().iron_future_flow_lock_th;
        float cap = cfg().iron_future_flow_congest_cap;
        for (int ti = 0; ti < static_cast<int>(tasks.size()); ti++) {
            TaskMeta& t = tasks[ti];
            auto it = task_pool_snapshot_.find(t.tid);
            if (it == task_pool_snapshot_.end() || it->second.locations.size() < 2) continue;
            auto& locs = it->second.locations;
            int sidx = -1;
            for (int i = 0; i < static_cast<int>(locs.size()); i++)
                if (locs[i] == t.start) { sidx = i; break; }
            if (sidx < 0) sidx = 0;
            int z_prev = ff_zone_of_loc(locs[sidx]);
            if (z_prev < 0 || z_prev >= ff_zone_count_) continue;
            bool blocked = false;
            for (int i = sidx + 1; i < static_cast<int>(locs.size()) && !blocked; i++) {
                int z_tgt = ff_zone_of_loc(locs[i]);
                if (z_tgt < 0 || z_tgt >= ff_zone_count_ || z_tgt == z_prev) { z_prev = z_tgt; continue; }
                int cur = z_prev;
                while (cur != z_tgt && !blocked) {
                    int next = ff_path_next_[cur * ff_zone_count_ + z_tgt];
                    if (next >= ff_zone_count_) break;
                    const float* zd = &ff_dir_[cur * 4];
                    float zt = zd[0]+zd[1]+zd[2]+zd[3];
                    if (cap > 0.0f && zt >= cap) {
                        blocked = true;
                        break;
                    }
                    if (lth > 0.0f && zt >= lth) {
                        int dom = 0;
                        if (zd[1] > zd[0]) dom = 1;
                        if (zd[2] > zd[0] && zd[2] > zd[1]) dom = 2;
                        if (zd[3] > zd[0] && zd[3] > zd[1] && zd[3] > zd[2]) dom = 3;
                        int zr = cur / ff_zone_cols_, zc2 = cur % ff_zone_cols_;
                        int nr = next / ff_zone_cols_, nc = next % ff_zone_cols_;
                        int dir_z = ff_dir_from_delta(nr - zr, nc - zc2);
                        if (dir_z >= 0 && dir_z == ((dom + 2) % 4)) {
                            blocked = true;
                        }
                    }
                    cur = next;
                }
                z_prev = z_tgt;
            }
            if (blocked) {
                t.ff_blocked = true;
            }
        }
    }
    
    std::vector<int> zone_shortage(zone_count_, 0);
    int shortage_cap = std::max(1, cfg().iron_balance_cap);
    int starved_threshold = std::max(1, cfg().iron_starved_zone_threshold);
    for (int z = 0; z < zone_count_; z++) {
        int raw_shortage = zone_free_tasks[z] - zone_free_agents[z];
        zone_shortage[z] = clamp_int(raw_shortage, -shortage_cap, shortage_cap);
        if (zone_shortage[z] >= starved_threshold) stats.starved_zones++;
    }

    if (free_agents.empty() || tasks.empty()) {
        int interval = cfg().iron_stats_interval;
        if (interval > 0 && cur_tick - last_stats_tick_ >= interval) {
            last_stats_tick_ = cur_tick;
            print_round_stats(stats, cur_tick);
        }
        return;
    }

    int max_total = n_agents;
    if (cfg().iron_assign_ratio < 0.999f) {
        max_total = std::max(1, static_cast<int>(cfg().iron_assign_ratio * n_agents));
    }
    if (cfg().iron_assign_variant == 1 && stats.early_phase > 0.0) {
        float progress = 1.0f - static_cast<float>(stats.early_phase);
        float target_ratio = 0.70f + 0.30f * progress;
        max_total = std::min(max_total,
                             std::max(1, static_cast<int>(target_ratio * n_agents)));
    } else if (cfg().iron_assign_variant == 4) {
        float progress = std::min(1.0f, std::max(0.0f, cur_tick / 4500.0f));
        float target_ratio = 0.78f + 0.22f * progress;
        max_total = std::min(max_total,
                             std::max(1, static_cast<int>(target_ratio * n_agents)));
    } else if (cfg().iron_assign_variant == 5) {
        float progress = std::min(1.0f, std::max(0.0f, cur_tick / 3000.0f));
        float target_ratio = 0.88f + 0.12f * progress;
        max_total = std::min(max_total,
                             std::max(1, static_cast<int>(target_ratio * n_agents)));
    } else if (cfg().iron_assign_variant == 6) {
        float progress = std::min(1.0f, std::max(0.0f, cur_tick / 2200.0f));
        float target_ratio = 0.84f + 0.16f * progress;
        max_total = std::min(max_total,
                             std::max(1, static_cast<int>(target_ratio * n_agents)));
    } else if (cfg().iron_assign_variant == 7) {
        float progress = std::min(1.0f, std::max(0.0f, cur_tick / 3500.0f));
        float target_ratio = 0.82f + 0.18f * progress;
        max_total = std::min(max_total,
                             std::max(1, static_cast<int>(target_ratio * n_agents)));
    } else if (cfg().iron_assign_variant == 8) {
        float progress = std::min(1.0f, std::max(0.0f, cur_tick / 10000.0f));
        float target_ratio = 0.70f + 0.30f * progress;
        max_total = std::min(max_total,
                             std::max(1, static_cast<int>(target_ratio * n_agents)));
    } else if (cfg().iron_assign_variant == 9) {
        max_total = std::min(max_total,
                             std::max(1, static_cast<int>(0.70f * n_agents)));
    } else if (cfg().iron_assign_variant == 10) {
        max_total = std::min(max_total,
                             std::max(1, static_cast<int>(0.75f * n_agents)));
    } else if (cfg().iron_assign_variant == 11) {
        float progress = std::min(1.0f, std::max(0.0f, cur_tick / 8000.0f));
        float target_ratio = 0.70f + 0.10f * progress;
        max_total = std::min(max_total,
                             std::max(1, static_cast<int>(target_ratio * n_agents)));
    } else if (cfg().iron_assign_variant == 12) {
        max_total = std::min(max_total,
                             std::max(1, static_cast<int>(0.80f * n_agents)));
    } else if (cfg().iron_assign_variant == 13 ||
               cfg().iron_assign_variant == 14 ||
               cfg().iron_assign_variant == 15 ||
               cfg().iron_assign_variant == 16) {
        max_total = std::min(max_total,
                             std::max(1, static_cast<int>(0.70f * n_agents)));
    }
    const int base_target_active = max_total;
    stats.base_target_active = base_target_active;

    int base_new_assign = std::min<int>(free_agents.size(), tasks.size());
    base_new_assign = std::min(base_new_assign,
                               std::max(0, base_target_active - stats.busy_before));

    int opportunity_slots = 0;
    if (cfg().iron_assign_variant == 16 && cfg().iron_opportunity_ratio > 0.0f) {
        stats.opportunity_cap = std::max(0,
            static_cast<int>(cfg().iron_opportunity_ratio * n_agents));
        const int allowed_total = std::min(n_agents,
                                           base_target_active + stats.opportunity_cap);
        const int active_after_base = stats.busy_before + base_new_assign;
        opportunity_slots = std::max(0, allowed_total - active_after_base);
        opportunity_slots = std::min<int>(opportunity_slots,
                                          free_agents.size() - base_new_assign);
        opportunity_slots = std::min<int>(opportunity_slots,
                                          tasks.size() - base_new_assign);
        stats.target_active = allowed_total;
    } else {
        stats.target_active = base_target_active;
    }

    int max_new_assign = base_new_assign + opportunity_slots;
    if (max_new_assign <= 0) return;

    const int topk = std::max(1, cfg().iron_candidate_topk);
    const int local_radius = std::max(0, cfg().iron_local_radius);
    const int max_radius = std::max(local_radius, cfg().iron_max_radius);
    const int target_pool = std::max(topk * 2, topk);
    const bool aggressive_assign =
        cfg().iron_aggressive_assign && cfg().iron_aggressive_iters > 0;

    auto candidate_score = [&](const AgentMeta& agent, const TaskMeta& task) {
        return score_candidate(agent, task, finish_zone_pressure, zone_shortage);
    };

    auto optimization_score = [&](const AgentMeta& agent, const TaskMeta& task) {
        return score_candidate(agent, task, finish_zone_pressure, zone_shortage);
    };

    std::vector<int> short_task_pool;
    int short_topk = std::max(0, cfg().iron_short_task_topk);
    if (short_topk > 0) {
        short_task_pool.resize(tasks.size());
        std::iota(short_task_pool.begin(), short_task_pool.end(), 0);
        auto by_short_task = [&](int lhs, int rhs) {
            if (tasks[lhs].length != tasks[rhs].length) {
                return tasks[lhs].length < tasks[rhs].length;
            }
            return tasks[lhs].tid < tasks[rhs].tid;
        };
        if (static_cast<int>(short_task_pool.size()) > short_topk) {
            std::nth_element(short_task_pool.begin(),
                             short_task_pool.begin() + short_topk,
                             short_task_pool.end(),
                             by_short_task);
            short_task_pool.resize(short_topk);
        }
        std::sort(short_task_pool.begin(), short_task_pool.end(), by_short_task);
    }
    stats.short_pool = static_cast<int>(short_task_pool.size());

    std::vector<int> starved_task_pool;
    int starved_topk = std::max(0, cfg().iron_starved_task_topk);
    if (starved_topk > 0) {
        starved_task_pool.reserve(std::min<int>(tasks.size(), starved_topk));
        for (int ti = 0; ti < static_cast<int>(tasks.size()); ti++) {
            int z = tasks[ti].start_zone;
            if (z >= 0 &&
                z < static_cast<int>(zone_shortage.size()) &&
                zone_shortage[z] >= starved_threshold) {
                starved_task_pool.push_back(ti);
            }
        }
        auto by_starved_task = [&](int lhs, int rhs) {
            int lz = tasks[lhs].start_zone;
            int rz = tasks[rhs].start_zone;
            int ls = (lz >= 0 && lz < static_cast<int>(zone_shortage.size())) ? zone_shortage[lz] : 0;
            int rs = (rz >= 0 && rz < static_cast<int>(zone_shortage.size())) ? zone_shortage[rz] : 0;
            if (ls != rs) return ls > rs;
            if (tasks[lhs].length != tasks[rhs].length) {
                return tasks[lhs].length < tasks[rhs].length;
            }
            return tasks[lhs].tid < tasks[rhs].tid;
        };
        if (static_cast<int>(starved_task_pool.size()) > starved_topk) {
            std::nth_element(starved_task_pool.begin(),
                             starved_task_pool.begin() + starved_topk,
                             starved_task_pool.end(),
                             by_starved_task);
            starved_task_pool.resize(starved_topk);
        }
        std::sort(starved_task_pool.begin(), starved_task_pool.end(), by_starved_task);
    }
    stats.starved_pool = static_cast<int>(starved_task_pool.size());

    std::vector<std::vector<std::pair<float, int>>> candidates(free_agents.size());

    auto build_start = std::chrono::steady_clock::now();
    int solve_reserve_ms = std::max(40, time_limit_ms / 6);
    if (solve_reserve_ms >= time_limit_ms) solve_reserve_ms = 0;
    auto build_deadline = deadline - std::chrono::milliseconds(solve_reserve_ms);

    auto build_one_agent = [&](int ai) {
        const AgentMeta& agent = free_agents[ai];
        const int pool_keep = std::min(128, std::max(32, topk / 4));
        const int local_keep = std::max(1, topk - pool_keep);
        int chosen_radius = -1;
        int task_count_in_radius = 0;
        for (int radius = local_radius; radius <= max_radius; radius++) {
            task_count_in_radius = 0;
            for (int z = 0; z < zone_count_; z++) {
                if (zone_distance(agent.zone, z) <= radius) {
                    task_count_in_radius += static_cast<int>(tasks_by_zone[z].size());
                }
            }
            if (task_count_in_radius >= target_pool || radius == max_radius) {
                chosen_radius = radius;
                break;
            }
        }

        auto& cand = candidates[ai];
        if (chosen_radius >= 0 && task_count_in_radius > 0) {
            cand.reserve(std::min(task_count_in_radius, topk));
            for (int z = 0; z < zone_count_; z++) {
                if (zone_distance(agent.zone, z) > chosen_radius) continue;
                for (int ti : tasks_by_zone[z]) {
                    if (tasks[ti].component != agent.component) continue;
                    if (tasks[ti].ff_blocked) continue;
                    if (cfg().iron_assign_max_dist > 0) {
                        int d = distance_fast(env, agent.loc, tasks[ti].start);
                        if (d > cfg().iron_assign_max_dist) continue;
                    }
                    float score = candidate_score(agent, tasks[ti]);
                    cand.push_back({score, ti});
                }
            }
        } else {
            cand.reserve(std::min<int>(tasks.size(), topk));
            for (int ti = 0; ti < static_cast<int>(tasks.size()); ti++) {
                if (tasks[ti].component != agent.component) continue;
                if (tasks[ti].ff_blocked) continue;
                if (cfg().iron_assign_max_dist > 0) {
                    int d = distance_fast(env, agent.loc, tasks[ti].start);
                    if (d > cfg().iron_assign_max_dist) continue;
                }
                float score = candidate_score(agent, tasks[ti]);
                cand.push_back({score, ti});
            }
        }

        if (static_cast<int>(cand.size()) > local_keep) {
            std::nth_element(cand.begin(), cand.begin() + local_keep, cand.end());
            cand.resize(local_keep);
        }

        std::vector<std::pair<float, int>> pool_cand;
        pool_cand.reserve(std::min<int>(short_task_pool.size() + starved_task_pool.size(),
                                        pool_keep * 2));
        auto append_pool = [&](const std::vector<int>& pool) {
            for (int ti : pool) {
                if (ti < 0 || ti >= static_cast<int>(tasks.size())) continue;
                if (tasks[ti].component != agent.component) continue;
                if (tasks[ti].ff_blocked) continue;
                if (cfg().iron_assign_max_dist > 0) {
                    int d = distance_fast(env, agent.loc, tasks[ti].start);
                    if (d > cfg().iron_assign_max_dist) continue;
                }
                float score = candidate_score(agent, tasks[ti]);
                pool_cand.push_back({score, ti});
            }
        };
        append_pool(short_task_pool);
        append_pool(starved_task_pool);
        if (static_cast<int>(pool_cand.size()) > pool_keep) {
            std::nth_element(pool_cand.begin(), pool_cand.begin() + pool_keep, pool_cand.end());
            pool_cand.resize(pool_keep);
        }
        cand.insert(cand.end(), pool_cand.begin(), pool_cand.end());

        if (static_cast<int>(cand.size()) > topk) {
            std::nth_element(cand.begin(), cand.begin() + topk, cand.end());
            cand.resize(topk);
        }
        std::sort(cand.begin(), cand.end());
        cand.erase(std::unique(cand.begin(), cand.end(),
                               [](const auto& lhs, const auto& rhs) {
                                   return lhs.second == rhs.second;
                               }),
                   cand.end());
    };

    int n_build_threads = cfg().num_threads;
    if (n_build_threads <= 0) {
        n_build_threads = static_cast<int>(std::thread::hardware_concurrency());
        if (n_build_threads <= 0) n_build_threads = 4;
    }
    n_build_threads = std::min<int>(n_build_threads, free_agents.size());

    if (n_build_threads > 1 && free_agents.size() >= 256) {
        int chunk = (static_cast<int>(free_agents.size()) + n_build_threads - 1) / n_build_threads;
        std::vector<std::thread> workers;
        workers.reserve(n_build_threads);
        for (int t = 0; t < n_build_threads; t++) {
            int begin = t * chunk;
            int end = std::min<int>(begin + chunk, free_agents.size());
            if (begin >= end) break;
            workers.emplace_back([&, begin, end]() {
                for (int ai = begin; ai < end; ai++) {
                    if (((ai - begin) & 31) == 0 &&
                        std::chrono::steady_clock::now() >= build_deadline) {
                        break;
                    }
                    build_one_agent(ai);
                }
            });
        }
        for (auto& th : workers) th.join();
    } else {
        for (int ai = 0; ai < static_cast<int>(free_agents.size()); ai++) {
            if ((ai & 31) == 0 && std::chrono::steady_clock::now() >= build_deadline) break;
            build_one_agent(ai);
        }
    }

    std::vector<unsigned char> task_used(tasks.size(), 0);
    std::vector<unsigned char> agent_used(free_agents.size(), 0);
    std::vector<int> local_schedule = schedule_snapshot_;
    auto solve_start = std::chrono::steady_clock::now();
    auto build_end = std::chrono::steady_clock::now();
    stats.build_ms = elapsed_ms(build_start, build_end);

    // ---- 实时全路径FF检查（可选跳过agent→task1段） ----
    auto realtime_ff_allows = [&](const AgentMeta& agent, const TaskMeta& task, bool from_agent) -> bool {
        if (!ff_enabled || !ff_path_ready_ ||
            (cfg().iron_future_flow_lock_th == 0.0f && cfg().iron_future_flow_congest_cap <= 0.0f)) return true;
        float lth = cfg().iron_future_flow_lock_th;
        float rcap = cfg().iron_future_flow_congest_cap;
        if (lth <= 0.0f && rcap <= 0.0f) return true;
        int z_start;
        if (from_agent) {
            z_start = ff_zone_of_loc(agent.loc);
        } else {
            z_start = ff_zone_of_loc(task.start);
        }
        if (z_start < 0 || z_start >= ff_zone_count_) return true;
        auto it = task_pool_snapshot_.find(task.tid);
        if (it == task_pool_snapshot_.end() || it->second.locations.size() < 2) return true;
        auto& locs = it->second.locations;
        int sidx = -1;
        for (int i = 0; i < static_cast<int>(locs.size()); i++)
            if (locs[i] == task.start) { sidx = i; break; }
        if (sidx < 0) sidx = 0;
        int z_prev = z_start;
        for (int i = sidx; i < static_cast<int>(locs.size()); i++) {
            int z_tgt = ff_zone_of_loc(locs[i]);
            if (z_tgt < 0 || z_tgt >= ff_zone_count_ || z_tgt == z_prev) { z_prev = z_tgt; continue; }
            int cur = z_prev;
            while (cur != z_tgt) {
                int next = ff_path_next_[cur * ff_zone_count_ + z_tgt];
                if (next >= ff_zone_count_) break;
                const float* zd = &ff_dir_[cur * 4];
                float zt = zd[0]+zd[1]+zd[2]+zd[3];
                float rcap = cfg().iron_future_flow_congest_cap;
                if (rcap > 0.0f && zt >= rcap) {
                    ff_locked_candidates_++;
                    return false;
                }
                if (lth > 0.0f && zt >= lth) {
                    int dom = 0;
                    if (zd[1] > zd[0]) dom = 1;
                    if (zd[2] > zd[0] && zd[2] > zd[1]) dom = 2;
                    if (zd[3] > zd[0] && zd[3] > zd[1] && zd[3] > zd[2]) dom = 3;
                    int zr = cur / ff_zone_cols_, zc2 = cur % ff_zone_cols_;
                    int nr = next / ff_zone_cols_, nc = next % ff_zone_cols_;
                    int dir_z = ff_dir_from_delta(nr - zr, nc - zc2);
                    if (dir_z >= 0 && dir_z == ((dom + 2) % 4)) {
                        ff_locked_candidates_++;
                        return false;
                    }
                }
                cur = next;
            }
            z_prev = z_tgt;
        }
        return true;
    };

    // ---- 分配 -> 即时加流 ----
    auto assign_and_add_ff = [&](int ai, int ti) {
        const AgentMeta& agent = free_agents[ai];
        const TaskMeta& task = tasks[ti];
        local_schedule[agent.aid] = task.tid;
        task_used[ti] = 1;
        agent_used[ai] = 1;
        finish_zone_pressure[task.finish_zone]++;
        stats.assigned++;
        int dist = distance_fast(env, agent.loc, task.start);
        int zd = zone_distance(agent.zone, task.start_zone);
        stats.dist_sum += dist;
        stats.len_sum += task.length;
        stats.completion_cost_sum += dist + task.length;
        stats.zone_dist_sum += zd;
        if (dist <= 50) stats.assign_dist_near++;
        else if (dist <= 150) stats.assign_dist_mid++;
        else stats.assign_dist_far++;
        if (task.length >= cfg().iron_long_task_th) stats.long_tasks++;
        // 红绿灯免名单：agent全程在lan同一侧，不穿lane
        if (cfg().iron_lane_enabled) {
            int a_comp = IronLane::get_component(agent.loc);
            int t_comp = IronLane::get_component(task.start);
            IronLane::g_exempt_from_traffic_light[agent.aid] = (a_comp == t_comp && a_comp != 0) ? 1 : 0;
        }
        // 即时加流ff_dir_
        if (ff_enabled) {
            int z_prev = ff_zone_of_loc(agent.loc);
            int cumul = 0;
            auto it = task_pool_snapshot_.find(task.tid);
            if (it != task_pool_snapshot_.end() && it->second.locations.size() >= 2) {
                auto& locs = it->second.locations;
                int sidx = -1;
                for (int i = 0; i < static_cast<int>(locs.size()); i++)
                    if (locs[i] == task.start) { sidx = i; break; }
                if (sidx < 0) sidx = 0;
                for (int i = sidx; i < static_cast<int>(locs.size()); i++) {
                    int zc = ff_zone_of_loc(locs[i]);
                    float w = cfg().iron_future_flow_w;
                    ff_add_future_flow_path(z_prev, zc, w, cumul);
                    cumul += static_cast<int>(ff_get_zone_path(z_prev, zc).size());
                    z_prev = zc;
                }
            }
        }
    };

    // ---- 纯贪心：同zone优先（跳过agent→task1的FF）+ 邻zone（全路径FF） ----
    for (int ai = 0; ai < static_cast<int>(free_agents.size()) && stats.assigned < max_new_assign; ai++) {
        if ((ai & 255) == 0 && std::chrono::steady_clock::now() >= deadline) break;
        const AgentMeta& agent = free_agents[ai];
        int chosen_ti = -1;
        float best_wdist = std::numeric_limits<float>::max();
        
        // 同zone：扫全部，选最短加权距离，FF从task1开始
        if (agent.zone >= 0 && agent.zone < static_cast<int>(tasks_by_zone.size())) {
            for (int ti : tasks_by_zone[agent.zone]) {
                if (task_used[ti]) continue;
                if (tasks[ti].component != agent.component) continue;
                int d = distance_fast(env, agent.loc, tasks[ti].start);
                if (cfg().iron_assign_max_dist > 0 && d > cfg().iron_assign_max_dist) continue;
                float wdist = cfg().iron_dist_weight * d + cfg().iron_task_len_weight * tasks[ti].length;
                if (wdist >= best_wdist) continue;
                if (realtime_ff_allows(agent, tasks[ti], false)) {
                    chosen_ti = ti;
                    best_wdist = wdist;
                }
            }
        }
        
        // 邻zone（同zone没找到）：全路径FF
        if (chosen_ti < 0) {
            for (int z = 0; z < static_cast<int>(tasks_by_zone.size()); z++) {
                if (z == agent.zone) continue;
                int zd = zone_distance(agent.zone, z);
                if (zd <= 0 || zd > max_radius) continue;
                for (int ti : tasks_by_zone[z]) {
                    if (task_used[ti]) continue;
                    if (tasks[ti].component != agent.component) continue;
                    int d = distance_fast(env, agent.loc, tasks[ti].start);
                    if (cfg().iron_assign_max_dist > 0 && d > cfg().iron_assign_max_dist) continue;
                    float wdist = cfg().iron_dist_weight * d + cfg().iron_task_len_weight * tasks[ti].length;
                    if (wdist >= best_wdist) continue;
                    if (realtime_ff_allows(agent, tasks[ti], true)) {
                        chosen_ti = ti;
                        best_wdist = wdist;
                    }
                }
                if (chosen_ti >= 0) break;
            }
        }
        if (chosen_ti >= 0) {
            assign_and_add_ff(ai, chosen_ti);
        } else {
            stats.unmatched_agents++;
        }
    }

    auto solve_end = std::chrono::steady_clock::now();
    stats.solve_ms = elapsed_ms(solve_start, solve_end);
    double total_ms = elapsed_ms(start_time, solve_end);
    if (total_ms > 500.0)
        std::fprintf(stderr, " [TIMING] tick=%d build=%.0fms solve=%.0fms total=%.0fms pops=%d seed=%d rounds=%d swaps=%d locked=%d\n",
            cur_tick, stats.build_ms, stats.solve_ms, total_ms,
            stats.heap_pops, stats.aggressive_seeded, stats.aggressive_rounds, stats.aggressive_swaps, ff_locked_candidates_);
    
    // Update tidal dominant directions
    stats.unmatched_agents = std::max(0, stats.free_agents - stats.assigned);

    for (const AgentMeta& agent : free_agents) {
        if (agent.aid >= 0 && agent.aid < static_cast<int>(proposed_schedule.size())) {
            proposed_schedule[agent.aid] = local_schedule[agent.aid];
        }
    }

    int interval = cfg().iron_stats_interval;
    if (interval > 0 && cur_tick - last_stats_tick_ >= interval) {
        last_stats_tick_ = cur_tick;
        print_round_stats(stats, cur_tick);
    }
    
    // Update tidal dominant directions for next tick
    if (ff_enabled) {
        ff_update_tide();
    }
}

} // namespace optsvr
