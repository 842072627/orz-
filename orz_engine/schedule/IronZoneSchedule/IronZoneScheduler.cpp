// optsvr/schedule/IronZoneSchedule/IronZoneScheduler.cpp
// IRON mode 10: coarse service-zone greedy assignment with compact diagnostics.
#include "IronZoneScheduler.h"
#include "HeuristicMatrix.h"
#include "RuntimeConfig.h"
#include "iron_core_restrict.h"
#include "iron_highway.h"
#include "iron_highway_guide.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <deque>
#include <limits>
#include <unordered_set>
#include <utility>

namespace orz_engine {

namespace {

constexpr int H_CAP = 100000;
constexpr uint32_t FF_CACHE_MAGIC = 0x49524646u; // "IRFF"
constexpr uint32_t FF_CACHE_VERSION = 2u;
constexpr bool IRON_ZONE_LOG_ENABLED = false;

double elapsed_ms(std::chrono::steady_clock::time_point a,
                  std::chrono::steady_clock::time_point b) {
    return std::chrono::duration_cast<std::chrono::microseconds>(b - a).count() / 1000.0;
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
    if (cfg().iron_highway_quota_enabled) {
        build_highway_traffic_regions(env);
    } else {
        traffic_region_id_.clear();
        traffic_region_side_.clear();
        traffic_barrier_.clear();
        traffic_region_count_ = 0;
        traffic_regions_ready_ = false;
    }

    if (IRON_ZONE_LOG_ENABLED) {
        std::fprintf(stderr,
            "[IronZone] init zones=%dx%d comps=%d radius=%d weights(dist=%.2f,len=%.2f)\n",
            zone_rows_, zone_cols_,
            component_count_,
            cfg().iron_max_radius,
            cfg().iron_dist_weight, cfg().iron_task_len_weight);
    }
    if (cfg().debug_level >= 1 && cfg().iron_highway_quota_enabled) {
        int selected_regions = 0;
        for (int side : traffic_region_side_) {
            if (side >= 0) selected_regions++;
        }
        std::fprintf(stderr,
            "[IronZone] highway quota enabled limit=%d regions=%d selected=%d ready=%d\n",
            cfg().iron_highway_quota_limit,
            traffic_region_count_,
            selected_regions,
            traffic_regions_ready_ ? 1 : 0);
    }
    
    // TaskScheduler calls this only for schedule_mode=10, so this is the
    // schedule10-local precompute path and is independent of heuristic precompute.
    if (env) {
        ff_zone_rows_ = std::max(1, cfg().iron_future_flow_zone_rows);
        ff_zone_cols_ = std::max(1, cfg().iron_future_flow_zone_cols);
        ff_zone_count_ = ff_zone_rows_ * ff_zone_cols_;
        ff_dir_.assign(ff_zone_count_ * 4, 0.0f);
        ff_compute_zone_adjacency(env);
        if (!ff_load_cache("ff_cache.bin")) {
            ff_precompute_all_pairs(env);
            if (ff_path_ready_) ff_save_cache("ff_cache.bin");
        }
        if (IRON_ZONE_LOG_ENABLED) {
            std::fprintf(stderr, "[IronZone] future-flow zones=%dx%d path_ready=%d\n",
                         ff_zone_rows_, ff_zone_cols_, ff_path_ready_ ? 1 : 0);
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

// Precompute cell-level all-pairs first-hop paths between future-flow zones.
void IronZoneScheduler::ff_precompute_all_pairs(SharedEnvironment* env) {
    if (!env || ff_zone_count_ <= 0 || map_size_ <= 0 || rows_ <= 0 || cols_ <= 0) {
        ff_path_ready_ = false;
        return;
    }

    const int count = ff_zone_count_;
    if (IRON_ZONE_LOG_ENABLED) {
        std::fprintf(stderr, "[IronZone] precompute ff all-pairs zone paths start zones=%d\n", count);
    }
    auto t0 = std::chrono::steady_clock::now();

    ff_path_next_.assign(static_cast<size_t>(count) * count, 65535);
    ff_path_dist_.assign(static_cast<size_t>(count) * count, 65535);

    std::vector<std::vector<int>> zone_cells(count);
    for (int loc = 0; loc < map_size_; loc++) {
        if (env->map[loc] != 0) continue;
        int z = ff_zone_of_loc(loc);
        if (z >= 0 && z < count) zone_cells[z].push_back(loc);
    }

    const int dr[4] = {-1, 1, 0, 0};
    const int dc[4] = {0, 0, -1, 1};
    constexpr int INF_DIST = std::numeric_limits<int>::max() / 4;

    for (int z_src = 0; z_src < count; z_src++) {
        if (zone_cells[z_src].empty()) continue;

        std::vector<int> dist(map_size_, INF_DIST);
        std::vector<uint16_t> exit_zone(map_size_, 65535);
        std::deque<int> q;

        for (int loc : zone_cells[z_src]) {
            dist[loc] = 0;
            exit_zone[loc] = static_cast<uint16_t>(z_src);
            q.push_back(loc);
        }

        ff_path_next_[static_cast<size_t>(z_src) * count + z_src] =
            static_cast<uint16_t>(z_src);
        ff_path_dist_[static_cast<size_t>(z_src) * count + z_src] = 0;
        std::vector<unsigned char> reached(count, 0);
        reached[z_src] = 1;

        while (!q.empty()) {
            int cur = q.front();
            q.pop_front();
            int cur_z = ff_zone_of_loc(cur);
            int cd = dist[cur] + 1;
            int r = cur / cols_;
            int c = cur % cols_;

            for (int d = 0; d < 4; d++) {
                int nr = r + dr[d];
                int nc = c + dc[d];
                if (nr < 0 || nr >= rows_ || nc < 0 || nc >= cols_) continue;
                int nxt = nr * cols_ + nc;
                if (env->map[nxt] != 0 || cd >= dist[nxt]) continue;

                dist[nxt] = cd;
                int nz = ff_zone_of_loc(nxt);
                uint16_t first_hop =
                    (cur_z == z_src && nz != z_src) ? static_cast<uint16_t>(nz) : exit_zone[cur];
                exit_zone[nxt] = first_hop;

                if (nz >= 0 && nz < count && !reached[nz]) {
                    reached[nz] = 1;
                    size_t idx = static_cast<size_t>(z_src) * count + nz;
                    ff_path_next_[idx] = first_hop;
                    ff_path_dist_[idx] =
                        static_cast<uint16_t>(std::min(
                            cd, static_cast<int>(std::numeric_limits<uint16_t>::max() - 1)));
                }
                q.push_back(nxt);
            }
        }

        if (IRON_ZONE_LOG_ENABLED && (z_src + 1) % 500 == 0) {
            auto now = std::chrono::steady_clock::now();
            double elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - t0).count() / 1000.0;
            std::fprintf(stderr, "  [IronZone] ff BFS %d/%d %.1fs\n", z_src + 1, count, elapsed);
        }
    }

    ff_path_ready_ = true;
    auto t1 = std::chrono::steady_clock::now();
    if (IRON_ZONE_LOG_ENABLED) {
        double total = std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count() / 1000.0;
        std::fprintf(stderr, "[IronZone] precompute ff all-pairs done %.1fs\n", total);
    }
}

void IronZoneScheduler::ff_save_cache(const char* path) const {
    if (!ff_path_ready_ || ff_path_next_.empty() || ff_path_dist_.empty()) return;
    FILE* f = std::fopen(path, "wb");
    if (!f) {
        if (IRON_ZONE_LOG_ENABLED) {
            std::fprintf(stderr, "[IronZone] cannot write ff cache: %s\n", path);
        }
        return;
    }

    const uint32_t fields[] = {
        FF_CACHE_MAGIC,
        FF_CACHE_VERSION,
        static_cast<uint32_t>(rows_),
        static_cast<uint32_t>(cols_),
        static_cast<uint32_t>(ff_zone_rows_),
        static_cast<uint32_t>(ff_zone_cols_),
        static_cast<uint32_t>(ff_zone_count_)
    };
    constexpr size_t field_count = sizeof(fields) / sizeof(fields[0]);
    bool ok = std::fwrite(fields, sizeof(uint32_t), field_count, f) == field_count;
    const size_t n = static_cast<size_t>(ff_zone_count_) * ff_zone_count_;
    ok = ok && std::fwrite(ff_path_next_.data(), sizeof(uint16_t), n, f) == n;
    ok = ok && std::fwrite(ff_path_dist_.data(), sizeof(uint16_t), n, f) == n;
    std::fclose(f);

    if (IRON_ZONE_LOG_ENABLED) {
        double mb = (static_cast<double>(n) * sizeof(uint16_t) * 2.0 +
                     static_cast<double>(sizeof(fields))) / (1024.0 * 1024.0);
        std::fprintf(stderr, "[IronZone] ff cache %s (%d zones, %.1f MiB): %s\n",
                     ok ? "saved" : "write failed", ff_zone_count_, mb, path);
    }
}

bool IronZoneScheduler::ff_load_cache(const char* path) {
    FILE* f = std::fopen(path, "rb");
    if (!f) return false;

    uint32_t fields[7] = {};
    constexpr size_t field_count = sizeof(fields) / sizeof(fields[0]);
    bool ok = std::fread(fields, sizeof(uint32_t), field_count, f) == field_count;
    if (!ok || fields[0] != FF_CACHE_MAGIC || fields[1] != FF_CACHE_VERSION ||
        fields[2] != static_cast<uint32_t>(rows_) ||
        fields[3] != static_cast<uint32_t>(cols_) ||
        fields[4] != static_cast<uint32_t>(ff_zone_rows_) ||
        fields[5] != static_cast<uint32_t>(ff_zone_cols_) ||
        fields[6] != static_cast<uint32_t>(ff_zone_count_)) {
        std::fclose(f);
        return false;
    }

    const size_t n = static_cast<size_t>(ff_zone_count_) * ff_zone_count_;
    std::vector<uint16_t> next(n);
    std::vector<uint16_t> dist(n);
    ok = std::fread(next.data(), sizeof(uint16_t), n, f) == n;
    ok = ok && std::fread(dist.data(), sizeof(uint16_t), n, f) == n;
    std::fclose(f);
    if (!ok) return false;

    ff_path_next_.swap(next);
    ff_path_dist_.swap(dist);
    ff_path_ready_ = true;
    if (IRON_ZONE_LOG_ENABLED) {
        std::fprintf(stderr, "[IronZone] ff cache loaded (%d zones): %s\n", ff_zone_count_, path);
    }
    return true;
}

std::vector<int> IronZoneScheduler::ff_get_zone_path_fast(int z_from, int z_to) const {
    std::vector<int> path;
    if (!ff_path_ready_ || z_from < 0 || z_to < 0 ||
        z_from >= ff_zone_count_ || z_to >= ff_zone_count_) {
        return path;
    }
    int cur = z_from;
    path.push_back(cur);
    for (int guard = 0; cur != z_to && guard <= ff_zone_count_; guard++) {
        uint16_t next = ff_path_next_[static_cast<size_t>(cur) * ff_zone_count_ + z_to];
        if (next >= ff_zone_count_) {
            path.clear();
            return path;
        }
        cur = next;
        path.push_back(cur);
    }
    if (path.empty() || path.back() != z_to) path.clear();
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
    if (ff_path_ready_) return ff_get_zone_path_fast(z_from, z_to);

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
    auto path = ff_path_ready_ ? ff_get_zone_path_fast(z_from, z_to)
                               : ff_get_zone_path(z_from, z_to);
    if (path.size() < 2) return;
    int cz = ff_cell_size();
    
    // Record one representative direction for each zone on the path.
    for (size_t i = 0; i < path.size(); i++) {
        int z = path[i];
        int cumul = (cumul_offset + static_cast<int>(i)) * cz;
        float decay = 1.0f / (1.0f + static_cast<float>(cumul) * cfg().iron_future_flow_decay);
        float delta = cfg().iron_future_flow_k * weight * decay;

        const int cr = z / ff_zone_cols_;
        const int cc = z % ff_zone_cols_;
        const int ref = (i == 0) ? path[1] : path[i - 1];
        const int rr = ref / ff_zone_cols_;
        const int rc = ref % ff_zone_cols_;
        const int d = (i == 0)
            ? ff_dir_from_delta(rr - cr, rc - cc)
            : ff_dir_from_delta(cr - rr, cc - rc);
        if (d >= 0) ff_dir_[z * 4 + d] += delta;
    }
}

float IronZoneScheduler::ff_zone_total_flow(int zone) const {
    if (zone < 0 || zone >= ff_zone_count_ || ff_dir_.empty()) return 0.0f;
    const float* zd = &ff_dir_[zone * 4];
    return zd[0] + zd[1] + zd[2] + zd[3];
}

bool IronZoneScheduler::ff_step_blocked(int cur_zone, int next_zone,
                                        float lock_th, float congest_cap,
                                        bool count_locked) const {
    if (cur_zone < 0 || cur_zone >= ff_zone_count_ ||
        next_zone < 0 || next_zone >= ff_zone_count_ ||
        ff_dir_.empty()) {
        return false;
    }

    const float* zd = &ff_dir_[cur_zone * 4];
    const float total_flow = ff_zone_total_flow(cur_zone);
    if (congest_cap > 0.0f && total_flow >= congest_cap) {
        if (count_locked) ff_locked_candidates_++;
        return true;
    }

    if (lock_th <= 0.0f || total_flow < lock_th) return false;

    int dom = 0;
    if (zd[1] > zd[dom]) dom = 1;
    if (zd[2] > zd[dom]) dom = 2;
    if (zd[3] > zd[dom]) dom = 3;

    const int cr = cur_zone / ff_zone_cols_;
    const int cc = cur_zone % ff_zone_cols_;
    const int nr = next_zone / ff_zone_cols_;
    const int nc = next_zone % ff_zone_cols_;
    const int dir_z = ff_dir_from_delta(nr - cr, nc - cc);
    if (dir_z >= 0 && dir_z == ((dom + 2) % 4)) {
        if (count_locked) ff_locked_candidates_++;
        return true;
    }
    return false;
}

bool IronZoneScheduler::ff_route_allows(int start_zone,
                                        const std::vector<int>& locs,
                                        int start_idx,
                                        float lock_th,
                                        float congest_cap,
                                        bool count_locked) const {
    if (!ff_path_ready_ || start_zone < 0 || start_zone >= ff_zone_count_) return true;

    int z_prev = start_zone;
    for (int i = std::max(0, start_idx); i < static_cast<int>(locs.size()); i++) {
        const int z_tgt = ff_zone_of_loc(locs[i]);
        if (z_tgt < 0 || z_tgt >= ff_zone_count_) continue;
        if (z_tgt == z_prev) continue;

        int cur = z_prev;
        while (cur != z_tgt) {
            const uint16_t next = ff_path_next_[static_cast<size_t>(cur) * ff_zone_count_ + z_tgt];
            if (next >= ff_zone_count_) break;
            if (ff_step_blocked(cur, static_cast<int>(next), lock_th, congest_cap, count_locked)) {
                return false;
            }
            cur = static_cast<int>(next);
        }
        z_prev = z_tgt;
    }
    return true;
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

bool IronZoneScheduler::highway_quota_cell(int loc) const {
    if (loc < 0 || loc >= map_size_) return false;
    uint8_t dummy = 0;
    return IronHighway::get_forbid_mask(loc) != 0 ||
           orz_engine::get_highway_guide(loc, dummy);
}

int IronZoneScheduler::traffic_region_of_loc(int loc) const {
    if (loc < 0 || loc >= static_cast<int>(traffic_region_id_.size())) return -1;
    return traffic_region_id_[loc];
}

int IronZoneScheduler::traffic_region_side(int region) const {
    if (region < 0 || region >= static_cast<int>(traffic_region_side_.size())) return -1;
    return traffic_region_side_[region];
}

void IronZoneScheduler::build_highway_traffic_regions(SharedEnvironment* env) {
    traffic_region_count_ = 0;
    traffic_regions_ready_ = false;
    traffic_region_id_.assign(map_size_, -1);
    traffic_region_side_.clear();
    traffic_barrier_.assign(map_size_, 0);
    if (!env || map_size_ <= 0 || cols_ <= 0) return;

    for (int loc = 0; loc < map_size_; loc++) {
        if (env->map[loc] != 0) continue;
        if (highway_quota_cell(loc) || orz_engine::is_core_non_highway(loc)) {
            traffic_barrier_[loc] = 1;
        }
    }

    std::deque<int> q;
    for (int loc = 0; loc < map_size_; loc++) {
        if (env->map[loc] != 0 || traffic_barrier_[loc] ||
            traffic_region_id_[loc] != -1) {
            continue;
        }

        const int rid = traffic_region_count_++;
        traffic_region_id_[loc] = rid;
        q.push_back(loc);

        while (!q.empty()) {
            int cur = q.front();
            q.pop_front();
            int r = cur / cols_;
            int c = cur % cols_;

            auto push_if_region_cell = [&](int next) {
                if (next < 0 || next >= map_size_) return;
                if (env->map[next] != 0 || traffic_barrier_[next] ||
                    traffic_region_id_[next] != -1) {
                    return;
                }
                traffic_region_id_[next] = rid;
                q.push_back(next);
            };

            if (r > 0) push_if_region_cell(cur - cols_);
            if (r + 1 < rows_) push_if_region_cell(cur + cols_);
            if (c > 0) push_if_region_cell(cur - 1);
            if (c + 1 < cols_) push_if_region_cell(cur + 1);
        }
    }

    std::vector<int> adjacent_edges(traffic_region_count_, 0);
    for (int loc = 0; loc < map_size_; loc++) {
        if (!highway_quota_cell(loc)) continue;
        int r = loc / cols_;
        int c = loc % cols_;
        auto count_neighbor_region = [&](int next) {
            int rid = traffic_region_of_loc(next);
            if (rid >= 0 && rid < traffic_region_count_) adjacent_edges[rid]++;
        };
        if (r > 0) count_neighbor_region(loc - cols_);
        if (r + 1 < rows_) count_neighbor_region(loc + cols_);
        if (c > 0) count_neighbor_region(loc - 1);
        if (c + 1 < cols_) count_neighbor_region(loc + 1);
    }

    std::vector<std::pair<int, int>> adjacent_regions;
    adjacent_regions.reserve(traffic_region_count_);
    for (int rid = 0; rid < traffic_region_count_; rid++) {
        if (adjacent_edges[rid] > 0) adjacent_regions.push_back({adjacent_edges[rid], rid});
    }
    std::sort(adjacent_regions.begin(), adjacent_regions.end(),
              [](const auto& a, const auto& b) {
                  if (a.first != b.first) return a.first > b.first;
                  return a.second < b.second;
              });

    traffic_region_side_.assign(traffic_region_count_, -1);
    const int selected = std::min<int>(2, adjacent_regions.size());
    for (int i = 0; i < selected; i++) {
        traffic_region_side_[adjacent_regions[i].second] = i;
    }
    traffic_regions_ready_ = true;
}

bool IronZoneScheduler::segment_consumes_highway_quota(int from, int to) const {
    if (!cfg().iron_highway_quota_enabled || !traffic_regions_ready_) return false;
    if (highway_quota_cell(from) || highway_quota_cell(to)) return true;

    const int from_region = traffic_region_of_loc(from);
    const int to_region = traffic_region_of_loc(to);
    if (from_region < 0 || to_region < 0 || from_region == to_region) return false;

    const int from_side = traffic_region_side(from_region);
    const int to_side = traffic_region_side(to_region);
    return from_side >= 0 && to_side >= 0 && from_side != to_side;
}

bool IronZoneScheduler::task_consumes_highway_quota(
    const Task& task, int start_idx, int agent_loc) const
{
    if (!cfg().iron_highway_quota_enabled || !traffic_regions_ready_) return false;
    if (start_idx < 0 || start_idx >= static_cast<int>(task.locations.size())) return false;

    int prev = agent_loc;
    for (int i = start_idx; i < static_cast<int>(task.locations.size()); i++) {
        int next = task.locations[i];
        if (segment_consumes_highway_quota(prev, next)) return true;
        prev = next;
    }
    return false;
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

void IronZoneScheduler::print_round_stats(const RoundStats& s, int tick) const {
    const bool log_enabled =
        IRON_ZONE_LOG_ENABLED ||
        (cfg().iron_highway_quota_enabled && cfg().debug_level >= 1);
    if (!log_enabled) return;

    double avg_dist = s.assigned > 0 ? static_cast<double>(s.dist_sum) / s.assigned : 0.0;
    double avg_len = s.assigned > 0 ? static_cast<double>(s.len_sum) / s.assigned : 0.0;
    double avg_zd = s.assigned > 0 ? static_cast<double>(s.zone_dist_sum) / s.assigned : 0.0;
    double avg_completion = s.assigned > 0
        ? static_cast<double>(s.completion_cost_sum) / s.assigned
        : 0.0;
    double cross_pct = s.assigned > 0 ? 100.0 * s.cross_zone / s.assigned : 0.0;
    double long_pct = s.assigned > 0 ? 100.0 * s.long_tasks / s.assigned : 0.0;
    int busy_after = s.busy_before + s.assigned;

    std::fprintf(stderr,
        "[IronZone] tick=%d freeA=%d freeT=%d assigned=%d busy=%d unmatched=%d avgDist=%.1f avgLen=%.1f avgComp=%.1f cross=%.1f%% avgZ=%.2f highwayQuota(active=%d,new=%d,cap=%d,cand=%d,reject=%d) skip(comp=%d,highway=%d) freeLen(<=500=%d,<=1000=%d,<=1500=%d,>1500=%d) assignLen(<=500=%d,<=1000=%d,<=1500=%d,>1500=%d) assignDist(<=50=%d,<=150=%d,>150=%d) long>=%d=%.1f%% time(solve=%.1fms)\n",
        tick,
        s.free_agents, s.free_tasks, s.assigned, busy_after, s.unmatched_agents,
        avg_dist, avg_len, avg_completion, cross_pct, avg_zd,
        s.highway_quota_active, s.highway_quota_new,
        std::max(0, cfg().iron_highway_quota_limit),
        s.highway_quota_candidates, s.highway_quota_reject,
        s.skipped_component, s.skipped_highway_agent,
        s.free_len_short, s.free_len_mid, s.free_len_long, s.free_len_huge,
        s.assign_len_short, s.assign_len_mid, s.assign_len_long, s.assign_len_huge,
        s.assign_dist_near, s.assign_dist_mid, s.assign_dist_far,
        cfg().iron_long_task_th, long_pct, s.solve_ms);
    
    // FF stats
    if (!ff_dir_.empty() && ff_zone_count_ > 0) {
        int active_zones = 0;
        double avg_heat = 0.0, max_heat = 0.0;
        int one_dir = 0, two_opp = 0, two_adj = 0, three_dir = 0, four_dir = 0;
        int nN = 0, nS = 0, nE = 0, nW = 0;
        int anyN = 0, anyS = 0, anyE = 0, anyW = 0;
        for (int z = 0; z < ff_zone_count_; z++) {
            float d0 = ff_dir_[z * 4], d1 = ff_dir_[z * 4 + 1];
            float d2 = ff_dir_[z * 4 + 2], d3 = ff_dir_[z * 4 + 3];
            float total = d0 + d1 + d2 + d3;
            if (total > 0.5f) {
                active_zones++;
                float heat = total / (total + 10.0f);
                avg_heat += heat;
                if (heat > max_heat) max_heat = heat;
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
            }
            // Per-direction counts (any flow > 0.01, strong flow > 0.5)
            if (d0 > 0.01f) { anyE++; if (d0 > 0.5f) nE++; }
            if (d1 > 0.01f) { anyS++; if (d1 > 0.5f) nS++; }
            if (d2 > 0.01f) { anyW++; if (d2 > 0.5f) nW++; }
            if (d3 > 0.01f) { anyN++; if (d3 > 0.5f) nN++; }
        }
        if (active_zones > 0) avg_heat /= active_zones;
        std::fprintf(stderr,
            " [FF] active=%d locked=%d avgHeat=%.3f maxHeat=%.3f 1dir=%d 2opp=%d 2adj=%d 3dir=%d 4dir=%d | N=%d S=%d E=%d W=%d any(N=%d S=%d E=%d W=%d)\n",
            active_zones, ff_locked_candidates_, avg_heat, max_heat,
            one_dir, two_opp, two_adj, three_dir, four_dir,
            nN, nS, nE, nW, anyN, anyS, anyE, anyW);
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
        if (IRON_ZONE_LOG_ENABLED) {
            std::fprintf(stderr, "[IronZone] snapshot copy failed, skipping schedule round\n");
        }
        return;
    }

    RoundStats stats;
    stats.free_agents = 0;
    
    std::vector<AgentMeta> free_agents;
    free_agents.reserve(n_agents);
    std::unordered_set<int> assigned_tasks;
    assigned_tasks.reserve(n_agents * 2 + 1);

    for (int a = 0; a < n_agents; a++) {
        int tid = schedule_snapshot_[a];
        if (tid < 0) {
            int loc = curr_locations_snapshot_[a];
            // 禁止highway上的agent分配新任务
            if (orz_engine::IRON_HIGHWAY_FREEZE_ASSIGN && loc >= 0 && IronHighway::get_forbid_mask(loc) != 0) {
                stats.skipped_highway_agent++;
                continue;
            }
            int z = zone_of_loc(loc);
            int comp = component_of_loc(loc);
            if (loc >= 0 && loc < map_size_ && z >= 0 && comp >= 0) {
                free_agents.push_back({a, loc, z, comp});
            }
        } else {
            assigned_tasks.insert(tid);
            stats.busy_before++;
            if (cfg().iron_highway_quota_enabled && traffic_regions_ready_) {
                auto it = task_pool_snapshot_.find(tid);
                if (it != task_pool_snapshot_.end()) {
                    int idx = first_remaining_index(it->second);
                    if (task_consumes_highway_quota(it->second, idx,
                                                    curr_locations_snapshot_[a])) {
                        stats.highway_quota_active++;
                    }
                }
            }
        }
    }
    stats.free_agents = static_cast<int>(free_agents.size());

    std::vector<TaskMeta> tasks;
    tasks.reserve(task_pool_snapshot_.size());
    std::vector<std::vector<int>> tasks_by_zone(zone_count_);

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
        if (orz_engine::IRON_CORE_TASK_FILTER) {
            bool any_restricted = false;
            for (int li = idx; li < static_cast<int>(task.locations.size()); li++) {
                if (orz_engine::is_core_non_highway(task.locations[li])) {
                    any_restricted = true;
                    break;
                }
            }
            if (any_restricted) {
                stats.skipped_component++;
                continue;
            }
        }
        // 过滤highway+guide上的任务：路点在highway/guide cell上就不分配
        if (orz_engine::IRON_HIGHWAY_SKIP_TASK && !cfg().iron_highway_quota_enabled) {
            bool on_highway = false;
            for (int li = idx; li < static_cast<int>(task.locations.size()); li++) {
                int loc = task.locations[li];
                uint8_t dummy;
                if (IronHighway::get_forbid_mask(loc) != 0 || orz_engine::get_highway_guide(loc, dummy)) {
                    on_highway = true;
                    break;
                }
            }
            if (on_highway) {
                stats.skipped_component++;
                continue;
            }
        }
        int finish_zone = zone_of_loc(finish);

        if (assigned_tasks.find(tid) != assigned_tasks.end()) {
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
        if (meta.length <= 500) stats.free_len_short++;
        else if (meta.length <= 1000) stats.free_len_mid++;
        else if (meta.length <= 1500) stats.free_len_long++;
        else stats.free_len_huge++;

        int task_index = static_cast<int>(tasks.size());
        tasks.push_back(meta);
        tasks_by_zone[start_zone].push_back(task_index);
    }
    stats.free_tasks = static_cast<int>(tasks.size());

    // ---- pretick: reset future flow + add flow from busy agents ----
    bool ff_enabled = (!ff_dir_.empty() &&
        (cfg().iron_future_flow_lock_th != 0.0f ||
         cfg().iron_future_flow_congest_cap > 0.0f));
    ff_locked_candidates_ = 0;
    
    if (ff_enabled) {
        ff_dir_.assign(ff_zone_count_ * 4, 0.0f);
        ff_bfs_cached_source_ = -1;

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
            if (z_prev < 0 || z_prev >= ff_zone_count_) continue;
            int cumul = 0;
            for (int i = first_idx; i < static_cast<int>(locs.size()); i++) {
                int z_curr = ff_zone_of_loc(locs[i]);
                if (z_curr < 0 || z_curr >= ff_zone_count_) continue;
                ff_add_future_flow_path(z_prev, z_curr, cfg().iron_future_flow_w, cumul);
                cumul += static_cast<int>(ff_get_zone_path(z_prev, z_curr).size());
                z_prev = z_curr;
            }
        }
        if (IRON_ZONE_LOG_ENABLED) {
            int n_active_zone = 0;
            for (int z = 0; z < ff_zone_count_; z++) {
                float e = ff_dir_[z*4], s = ff_dir_[z*4+1];
                float w = ff_dir_[z*4+2], n = ff_dir_[z*4+3];
                float t = e + s + w + n;
                if (t >= 0.5f) n_active_zone++;
            }
            std::fprintf(stderr, "[IronZone] pretick: zones=%d active=%d path_ready=%d\n",
                ff_zone_count_, n_active_zone, ff_path_ready_ ? 1 : 0);
        }
    }

    if (ff_enabled && ff_path_ready_ &&
        (cfg().iron_future_flow_lock_th != 0.0f ||
         cfg().iron_future_flow_congest_cap > 0.0f)) {
        const float lock_th = cfg().iron_future_flow_lock_th;
        const float congest_cap = cfg().iron_future_flow_congest_cap;
        for (TaskMeta& task : tasks) {
            auto it = task_pool_snapshot_.find(task.tid);
            if (it == task_pool_snapshot_.end() || it->second.locations.size() < 2) continue;
            const auto& locs = it->second.locations;
            int sidx = -1;
            for (int i = 0; i < static_cast<int>(locs.size()); i++) {
                if (locs[i] == task.start) {
                    sidx = i;
                    break;
                }
            }
            if (sidx < 0) sidx = 0;

            const int z_prev = ff_zone_of_loc(locs[sidx]);
            task.ff_blocked = !ff_route_allows(z_prev, locs, sidx + 1,
                                               lock_th, congest_cap, false);
        }
    }
    
    if (free_agents.empty() || tasks.empty()) {
        int interval = cfg().iron_stats_interval;
        if (interval > 0 && cur_tick - last_stats_tick_ >= interval) {
            last_stats_tick_ = cur_tick;
            print_round_stats(stats, cur_tick);
        }
        return;
    }

    int max_new_assign = std::min<int>(free_agents.size(), tasks.size());
    if (max_new_assign <= 0) return;

    const int max_radius = std::max(0, cfg().iron_max_radius);
    std::vector<unsigned char> task_used(tasks.size(), 0);
    std::vector<int> local_schedule = schedule_snapshot_;
    auto solve_start = std::chrono::steady_clock::now();

    auto realtime_ff_allows = [&](const AgentMeta& agent, const TaskMeta& task,
                                  bool from_agent) -> bool {
        if (!ff_enabled || !ff_path_ready_ ||
            (cfg().iron_future_flow_lock_th == 0.0f &&
             cfg().iron_future_flow_congest_cap <= 0.0f)) {
            return true;
        }
        const float lock_th = cfg().iron_future_flow_lock_th;
        const float congest_cap = cfg().iron_future_flow_congest_cap;
        if (lock_th <= 0.0f && congest_cap <= 0.0f) return true;

        auto it = task_pool_snapshot_.find(task.tid);
        if (it == task_pool_snapshot_.end() || it->second.locations.size() < 2) return true;
        const auto& locs = it->second.locations;
        int sidx = -1;
        for (int i = 0; i < static_cast<int>(locs.size()); i++) {
            if (locs[i] == task.start) {
                sidx = i;
                break;
            }
        }
        if (sidx < 0) sidx = 0;
        const int z_prev = from_agent ? ff_zone_of_loc(agent.loc) : ff_zone_of_loc(task.start);
        return ff_route_allows(z_prev, locs, sidx, lock_th, congest_cap, true);
    };

    auto candidate_consumes_highway_quota = [&](const AgentMeta& agent,
                                                const TaskMeta& task) -> bool {
        if (!cfg().iron_highway_quota_enabled || !traffic_regions_ready_) return false;
        auto it = task_pool_snapshot_.find(task.tid);
        if (it == task_pool_snapshot_.end()) return false;
        int idx = first_remaining_index(it->second);
        return task_consumes_highway_quota(it->second, idx, agent.loc);
    };

    auto highway_quota_allows = [&](const AgentMeta& agent,
                                    const TaskMeta& task) -> bool {
        if (!cfg().iron_highway_quota_enabled || !traffic_regions_ready_) return true;
        if (!candidate_consumes_highway_quota(agent, task)) return true;

        stats.highway_quota_candidates++;
        const int quota_limit = std::max(0, cfg().iron_highway_quota_limit);
        if (stats.highway_quota_active + stats.highway_quota_new >= quota_limit) {
            stats.highway_quota_reject++;
            return false;
        }
        return true;
    };

    // ---- 分配 -> 即时加流 ----
    auto assign_and_add_ff = [&](int ai, int ti) {
        const AgentMeta& agent = free_agents[ai];
        const TaskMeta& task = tasks[ti];
        local_schedule[agent.aid] = task.tid;
        task_used[ti] = 1;
        stats.assigned++;
        int dist = distance_fast(env, agent.loc, task.start);
        int zd = zone_distance(agent.zone, task.start_zone);
        stats.dist_sum += dist;
        stats.len_sum += task.length;
        stats.completion_cost_sum += dist + task.length;
        stats.zone_dist_sum += zd;
        if (task.start_zone != task.finish_zone) stats.cross_zone++;
        if (task.length <= 500) stats.assign_len_short++;
        else if (task.length <= 1000) stats.assign_len_mid++;
        else if (task.length <= 1500) stats.assign_len_long++;
        else stats.assign_len_huge++;
        if (dist <= 50) stats.assign_dist_near++;
        else if (dist <= 150) stats.assign_dist_mid++;
        else stats.assign_dist_far++;
        if (task.length >= cfg().iron_long_task_th) stats.long_tasks++;
        if (candidate_consumes_highway_quota(agent, task)) {
            stats.highway_quota_new++;
        }
        // 即时加流ff_dir_
        if (ff_enabled) {
            int z_prev = ff_zone_of_loc(agent.loc);
            if (z_prev < 0 || z_prev >= ff_zone_count_) return;
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
                    if (zc < 0 || zc >= ff_zone_count_) continue;
                    float w = cfg().iron_future_flow_w;
                    ff_add_future_flow_path(z_prev, zc, w, cumul);
                    cumul += static_cast<int>(ff_get_zone_path(z_prev, zc).size());
                    z_prev = zc;
                }
            }
        }
    };

    // ---- 纯贪心：同zone优先，然后扫邻zone ----
    for (int ai = 0; ai < static_cast<int>(free_agents.size()) && stats.assigned < max_new_assign; ai++) {
        if ((ai & 255) == 0 && std::chrono::steady_clock::now() >= deadline) break;
        const AgentMeta& agent = free_agents[ai];
        int chosen_ti = -1;
        float best_wdist = std::numeric_limits<float>::max();
        
        // 同zone：扫全部，选最短加权距离
        if (agent.zone >= 0 && agent.zone < static_cast<int>(tasks_by_zone.size())) {
            for (int ti : tasks_by_zone[agent.zone]) {
                if (task_used[ti]) continue;
                if (tasks[ti].component != agent.component) continue;
                if (tasks[ti].ff_blocked) continue;
                int d = distance_fast(env, agent.loc, tasks[ti].start);
                float wdist = cfg().iron_dist_weight * d + cfg().iron_task_len_weight * tasks[ti].length;
                if (wdist >= best_wdist) continue;
                if (!highway_quota_allows(agent, tasks[ti])) continue;
                if (!realtime_ff_allows(agent, tasks[ti], false)) continue;
                chosen_ti = ti;
                best_wdist = wdist;
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
                    if (tasks[ti].ff_blocked) continue;
                    int d = distance_fast(env, agent.loc, tasks[ti].start);
                    float wdist = cfg().iron_dist_weight * d + cfg().iron_task_len_weight * tasks[ti].length;
                    if (wdist >= best_wdist) continue;
                    if (!highway_quota_allows(agent, tasks[ti])) continue;
                    if (!realtime_ff_allows(agent, tasks[ti], true)) continue;
                    chosen_ti = ti;
                    best_wdist = wdist;
                }
                if (chosen_ti >= 0) break;
            }
        }
        if (chosen_ti >= 0) {
            assign_and_add_ff(ai, chosen_ti);
        }
    }

    auto solve_end = std::chrono::steady_clock::now();
    stats.solve_ms = elapsed_ms(solve_start, solve_end);
    if (IRON_ZONE_LOG_ENABLED) {
        double total_ms = elapsed_ms(start_time, solve_end);
        if (total_ms > 500.0) {
            std::fprintf(stderr, " [TIMING] tick=%d solve=%.0fms total=%.0fms\n",
                cur_tick, stats.solve_ms, total_ms);
        }
    }

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
}

} // namespace orz_engine
