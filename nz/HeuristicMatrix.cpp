// optsvr/HeuristicMatrix.cpp — 多线程预计算全量距离矩阵
//
// 两种 BFS 模式根据 turn_aware 参数选择，只计算/存储一份矩阵：
// - 模式 A (turn_aware=false): location-only BFS, cost=1/step
// - 模式 B (turn_aware=true):  (location, orientation) BFS, 转弯和前进各 cost=1
#include "HeuristicMatrix.h"
#include "Config.h"
#include "RuntimeConfig.h"
#include "MapProfile.h"
#include "planner/GameDirectionalPlanner/orz_highway.h"

// ORZ-only stub (was in WarehouseRailConstraints.h)
namespace nz {
struct WarehouseRailConstraintStats { int passage_rows = 0; int passage_cols = 0; int forbidden_edges = 0; };
WarehouseRailConstraintStats build_warehouse_planner_rail_constraints(
    SharedEnvironment*, std::vector<std::array<unsigned char,4>>*, std::vector<int>*);
inline WarehouseRailConstraintStats build_warehouse_planner_rail_constraints(
    SharedEnvironment*, std::vector<std::array<unsigned char,4>>*, std::vector<int>*) {
    return {0, 0, 0};
}
}

#include <thread>
#include <atomic>
#include <deque>
#include <algorithm>
#include <cstdlib>
#include <iostream>
#include <cstdio>

namespace nz {

// ================================================================
// 全局单例
// ================================================================
HeuristicMatrix& heuristic_matrix() {
    static HeuristicMatrix instance;
    return instance;
}

// ================================================================
// get_h_fast — 统一便捷查询接口
// ================================================================
int get_h_fast(SharedEnvironment* env, int source, int target, int source_orient) {
    auto& hm = heuristic_matrix();
    if (hm.is_ready()) {
        if (hm.is_turn_aware() && source_orient >= 0) {
            // 模式 B：精确 action 步数
            uint16_t d = hm.get_oriented(source, source_orient, target);
            if (d != DIST_UNREACHABLE) return static_cast<int>(d);
        } else if (hm.is_turn_aware()) {
            // 模式 B 但未传朝向：取 4 个朝向的最小值
            uint16_t best = DIST_UNREACHABLE;
            for (int o = 0; o < 4; o++) {
                uint16_t d = hm.get_oriented(source, o, target);
                if (d < best) best = d;
            }
            if (best != DIST_UNREACHABLE) return static_cast<int>(best);
        } else {
            // 模式 A：location-only
            uint16_t d = hm.get(source, target);
            if (d != DIST_UNREACHABLE) return static_cast<int>(d);
            if (hm.is_direction_constrained()) return 100000000;
        }
    }
    // 回退：曼哈顿距离
    int src_r = source / env->cols, src_c = source % env->cols;
    int tgt_r = target / env->cols, tgt_c = target % env->cols;
    return std::abs(src_r - tgt_r) + std::abs(src_c - tgt_c);
}

// ================================================================
// 方向辅助常量
// 方向编码：0=E(+1), 1=S(+cols), 2=W(-1), 3=N(-cols)
// ================================================================
static int dir_delta(int d, int cols) {
    static const int dx[4] = {1, 0, -1, 0};  // col delta
    static const int dy[4] = {0, 1, 0, -1};  // row delta
    return dy[d] * cols + dx[d];
}

// 两个方向之间最少转弯次数（0,1,2）
static int turns_between(int d1, int d2) {
    int diff = (d1 - d2 + 4) % 4;
    if (diff == 0) return 0;
    if (diff == 2) return 2;
    return 1;  // diff == 1 or 3
}

static int move_dir_between(int from, int to, int cols) {
    int diff = to - from;
    if (diff == 1) return 0;
    if (diff == cols) return 1;
    if (diff == -1) return 2;
    if (diff == -cols) return 3;
    return -1;
}

bool HeuristicMatrix::forward_allowed(int from, int to) const {
    if (from < 0 || to < 0 || from >= map_size_ || to >= map_size_) return false;
    if (from >= static_cast<int>(cell_to_idx_.size()) ||
        to >= static_cast<int>(cell_to_idx_.size()) ||
        cell_to_idx_[from] < 0 || cell_to_idx_[to] < 0) return false;
    const int fr = from / cols_;
    const int fc = from % cols_;
    const int tr = to / cols_;
    const int tc = to % cols_;
    if (std::abs(fr - tr) + std::abs(fc - tc) != 1) return false;
    if (!direction_constrained_) return true;
    int d = move_dir_between(from, to, cols_);
    if (d < 0 || d >= 4) return false;
    if (from >= static_cast<int>(forbidden_dirs_.size())) return true;
    return forbidden_dirs_[from][d] == 0;
}

void HeuristicMatrix::build_direction_constraints(SharedEnvironment* env) {
    direction_constrained_ = false;
    forbidden_edge_count_ = 0;
    forbidden_dirs_.clear();

    if (env == nullptr || env->rows <= 2 || env->cols <= 2) return;

    const bool use_game_orz_direction =
        cfg().game_heuristic_hard_direction &&
        cfg().planner_mode == 16 &&
        env->map_name.find("orz") != std::string::npos;
    if (use_game_orz_direction) {
        ORZ_Highway::init();
        forbidden_dirs_.assign(map_size_, {0, 0, 0, 0});

        auto forbid_dir = [&](int loc, int dir) {
            if (loc < 0 || loc >= map_size_ || dir < 0 || dir >= 4) return;
            if (env->map[loc] != 0) return;
            if (forbidden_dirs_[loc][dir] == 0) {
                forbidden_dirs_[loc][dir] = 1;
                forbidden_edge_count_++;
            }
        };

        for (int i = 0; i < ORZ_Highway::NUM_FORBID; i++) {
            const int loc = ORZ_Highway::kForbid[i].loc;
            const uint8_t forbid = ORZ_Highway::kForbid[i].forbid;
            if (forbid & 2) forbid_dir(loc, 0);  // E
            if (forbid & 8) forbid_dir(loc, 1);  // S
            if (forbid & 1) forbid_dir(loc, 2);  // W
            if (forbid & 4) forbid_dir(loc, 3);  // N
        }

        direction_constrained_ = forbidden_edge_count_ > 0;
        if (direction_constrained_ && cfg().debug_level >= 1) {
            std::fprintf(stderr,
                "[HeuristicMatrix] game ORZ hard-direction forbidden_edges=%d\n",
                forbidden_edge_count_);
        }
        return;
    }

    return;
}

void HeuristicMatrix::bfs_from(int target_idx) {
    int target_cell = idx_to_cell_[target_idx];
    uint16_t* row = &matrix_[static_cast<size_t>(target_idx) * n_free_];

    std::deque<std::pair<int, uint16_t>> queue;
    queue.push_back({target_cell, 0});
    row[target_idx] = 0;

    while (!queue.empty()) {
        auto [cell, dist] = queue.front();
        queue.pop_front();

        uint16_t next_dist = dist + 1;

        for (int neighbor : reverse_neighbors_[cell]) {
            int nidx = cell_to_idx_[neighbor];
            if (nidx < 0) continue;

            if (next_dist < row[nidx]) {
                row[nidx] = next_dist;
                queue.push_back({neighbor, next_dist});
            }
        }
    }
}

// ================================================================
// bfs_from_oriented — 模式 B：(location, orientation) 反向 BFS
//
// 反向 BFS：从 target 出发，目标是算 "从任意 (source, orient) 出发到 target 的最少 action 数"
//
// 反向展开的状态转移（从 (loc, d) 反推 "谁能一步到达 (loc, d)"）：
//   1. 前驱 forward:  (prev_loc, d) → FW → (loc, d)
//      其中 prev_loc = loc - delta[d]
//      所以 (prev_loc, d) 的距离 = dist(loc, d) + 1
//   2. 前驱 CR:       (loc, (d+3)%4) → CR → (loc, d)
//      所以 (loc, (d+3)%4) 的距离 = dist(loc, d) + 1
//   3. 前驱 CCR:      (loc, (d+1)%4) → CCR → (loc, d)
//      所以 (loc, (d+1)%4) 的距离 = dist(loc, d) + 1
//
// target 的初始状态：所有 4 个方向 (target, d) 距离 = 0
// ================================================================
void HeuristicMatrix::bfs_from_oriented(int target_idx) {
    int target_cell = idx_to_cell_[target_idx];
    uint16_t* row = &matrix_[static_cast<size_t>(target_idx) * n_states_];

    // BFS 队列：(cell, orientation, dist)
    struct BfsEntry { int cell; int orient; uint16_t dist; };
    std::deque<BfsEntry> queue;

    // target 的所有方向距离 = 0（到达 target 位置即可，不关心最终朝向）
    for (int d = 0; d < 4; d++) {
        int state = cell_to_idx_[target_cell] * 4 + d;
        row[state] = 0;
        queue.push_back({target_cell, d, 0});
    }

    while (!queue.empty()) {
        auto [cell, orient, dist] = queue.front();
        queue.pop_front();

        int cell_idx = cell_to_idx_[cell];
        int cur_state = cell_idx * 4 + orient;

        // 如果当前 dist 已不是最优（被更短路径更新过），跳过
        if (dist > row[cur_state]) continue;

        uint16_t next_dist = dist + 1;

        // 反向转移 1: forward 前驱
        // (prev_loc, orient) --FW--> (cell, orient)
        // prev_loc = cell - delta[orient]
        int delta = dir_delta(orient, cols_);
        int prev_cell = cell - delta;
        if (prev_cell >= 0 && prev_cell < map_size_) {
            int prev_idx = cell_to_idx_[prev_cell];
            if (prev_idx >= 0) {
                // 检查 prev_cell → cell 是否是合法移动
                bool valid = false;
                for (int nb : neighbors_[prev_cell]) {
                    if (nb == cell) { valid = true; break; }
                }
                if (valid && !forward_allowed(prev_cell, cell)) {
                    valid = false;
                }
                if (valid) {
                    int prev_state = prev_idx * 4 + orient;
                    if (next_dist < row[prev_state]) {
                        row[prev_state] = next_dist;
                        queue.push_back({prev_cell, orient, next_dist});
                    }
                }
            }
        }

        // 反向转移 2: CR 前驱
        // (cell, (orient+3)%4) --CR--> (cell, orient)
        int cr_prev_orient = (orient + 3) % 4;
        int cr_state = cell_idx * 4 + cr_prev_orient;
        if (next_dist < row[cr_state]) {
            row[cr_state] = next_dist;
            queue.push_back({cell, cr_prev_orient, next_dist});
        }

        // 反向转移 3: CCR 前驱
        // (cell, (orient+1)%4) --CCR--> (cell, orient)
        int ccr_prev_orient = (orient + 1) % 4;
        int ccr_state = cell_idx * 4 + ccr_prev_orient;
        if (next_dist < row[ccr_state]) {
            row[ccr_state] = next_dist;
            queue.push_back({cell, ccr_prev_orient, next_dist});
        }
    }
}

// ================================================================
// get — 模式 A：O(1) location-only 查询
// ================================================================
uint16_t HeuristicMatrix::get(int source, int target) const {
    if (!ready_ || turn_aware_
        || source < 0 || target < 0
        || source >= map_size_ || target >= map_size_) {
        return DIST_UNREACHABLE;
    }

    int si = cell_to_idx_[source];
    int ti = cell_to_idx_[target];
    if (si < 0 || ti < 0) return DIST_UNREACHABLE;

    return matrix_[static_cast<size_t>(ti) * n_free_ + si];
}

// ================================================================
// get_oriented — 模式 B：O(1) (location, orientation) 查询
// ================================================================
uint16_t HeuristicMatrix::get_oriented(int source, int source_orient, int target) const {
    if (!ready_ || !turn_aware_
        || source < 0 || target < 0
        || source >= map_size_ || target >= map_size_
        || source_orient < 0 || source_orient >= 4) {
        return DIST_UNREACHABLE;
    }

    int si = cell_to_idx_[source];
    int ti = cell_to_idx_[target];
    if (si < 0 || ti < 0) return DIST_UNREACHABLE;

    int source_state = si * 4 + source_orient;
    return matrix_[static_cast<size_t>(ti) * n_states_ + source_state];
}

// ================================================================
// build — 预处理阶段调用，根据 turn_aware 选择 BFS 模式
// ================================================================
void HeuristicMatrix::build(SharedEnvironment* env, int n_threads,
                            std::chrono::steady_clock::time_point deadline,
                            bool turn_aware)
{
    auto start_time = std::chrono::steady_clock::now();

    ready_ = false;
    n_computed_ = 0;
    build_time_ms_ = 0;
    turn_aware_ = turn_aware;
    direction_constrained_ = false;
    forbidden_edge_count_ = 0;
    forbidden_dirs_.clear();
    map_size_ = static_cast<int>(env->map.size());
    rows_ = env->rows;
    cols_ = env->cols;

    // ---- Step 1: 建立 cell ↔ compressed index 的映射 ----
    cell_to_idx_.assign(map_size_, -1);
    idx_to_cell_.clear();
    idx_to_cell_.reserve(map_size_ / 2);

    for (int i = 0; i < map_size_; i++) {
        if (env->map[i] == 0) {
            cell_to_idx_[i] = static_cast<int>(idx_to_cell_.size());
            idx_to_cell_.push_back(i);
        }
    }
    n_free_ = static_cast<int>(idx_to_cell_.size());
    n_states_ = n_free_ * 4;
    build_direction_constraints(env);

    // ---- Step 2: 检查是否在可计算范围内 ----
    if (n_free_ == 0) {
        fprintf(stderr, "[HeuristicMatrix] No free cells, skipping\n");
        return;
    }

    // 内存需求
    size_t cols_per_row = turn_aware_ ? static_cast<size_t>(n_states_) : static_cast<size_t>(n_free_);
    size_t mem_bytes = static_cast<size_t>(n_free_) * cols_per_row * sizeof(uint16_t);
    size_t mem_mb = mem_bytes / (1024 * 1024);

    if (n_free_ > MAX_PRECOMPUTE_CELLS) {
        fprintf(stderr,
            "[HeuristicMatrix] n_free=%d exceeds MAX_PRECOMPUTE_CELLS=%d, "
            "skipping precompute (would need ~%zu MB)\n",
            n_free_, MAX_PRECOMPUTE_CELLS, mem_mb);
        return;
    }

    fprintf(stderr,
        "[HeuristicMatrix] mode=%s%s, n_free=%d, matrix size=%.1f MB, starting build...\n",
        turn_aware_ ? "turn_aware" : "location_only",
        direction_constrained_ ? "+dir_hard" : "",
        n_free_, static_cast<float>(mem_mb));

    // ---- Step 3: 分配矩阵内存 ----
    try {
        matrix_.assign(static_cast<size_t>(n_free_) * cols_per_row, DIST_UNREACHABLE);
    } catch (const std::bad_alloc&) {
        fprintf(stderr,
            "[HeuristicMatrix] Failed to allocate %zu MB, skipping\n", mem_mb);
        return;
    }

    // ---- Step 4: 构建邻居表 ----
    neighbors_.clear();
    reverse_neighbors_.clear();
    neighbors_.resize(map_size_);
    reverse_neighbors_.resize(map_size_);
    int rows = env->rows;
    int cols = env->cols;
    for (int r = 0; r < rows; r++) {
        for (int c = 0; c < cols; c++) {
            int loc = r * cols + c;
            if (env->map[loc] != 0) continue;

            neighbors_[loc].clear();
            if (r > 0     && env->map[loc - cols] == 0) neighbors_[loc].push_back(loc - cols);
            if (r < rows-1 && env->map[loc + cols] == 0) neighbors_[loc].push_back(loc + cols);
            if (c > 0     && env->map[loc - 1] == 0)    neighbors_[loc].push_back(loc - 1);
            if (c < cols-1 && env->map[loc + 1] == 0)    neighbors_[loc].push_back(loc + 1);
        }
    }
    for (int loc : idx_to_cell_) {
        for (int nb : neighbors_[loc]) {
            if (forward_allowed(loc, nb)) {
                reverse_neighbors_[nb].push_back(loc);
            }
        }
    }

    // ---- Step 5: 确定线程数 ----
    if (n_threads <= 0) {
        n_threads = static_cast<int>(std::thread::hardware_concurrency());
        if (n_threads <= 0) n_threads = 4;
    }
    n_threads = std::min(n_threads, n_free_);

    fprintf(stderr, "[HeuristicMatrix] Using %d threads for BFS\n", n_threads);

    // ---- Step 6: 多线程并行 BFS ----
    std::atomic<int> next_task{0};
    std::atomic<int> completed{0};
    std::atomic<bool> timed_out{false};

    auto worker = [this, &next_task, &completed, &timed_out, &deadline]() {
        while (!timed_out.load(std::memory_order_relaxed)) {
            int idx = next_task.fetch_add(1, std::memory_order_relaxed);
            if (idx >= n_free_) break;

            if (std::chrono::steady_clock::now() > deadline) {
                timed_out.store(true, std::memory_order_relaxed);
                break;
            }

            if (turn_aware_) {
                bfs_from_oriented(idx);
            } else {
                bfs_from(idx);
            }
            completed.fetch_add(1, std::memory_order_relaxed);
        }
    };

    if (n_threads == 1) {
        worker();
    } else {
        std::vector<std::thread> threads;
        threads.reserve(n_threads);
        for (int t = 0; t < n_threads; t++) {
            threads.emplace_back(worker);
        }
        for (auto& th : threads) {
            th.join();
        }
    }

    n_computed_ = completed.load();

    auto end_time = std::chrono::steady_clock::now();
    build_time_ms_ = static_cast<int>(
        std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count());

    if (n_computed_ >= n_free_) {
        ready_ = true;
        fprintf(stderr,
            "[HeuristicMatrix] Build complete: %d/%d targets in %d ms\n",
            n_computed_, n_free_, build_time_ms_);
    } else {
        fprintf(stderr,
            "[HeuristicMatrix] Build PARTIAL: %d/%d targets in %d ms (timeout)\n",
            n_computed_, n_free_, build_time_ms_);
        matrix_.clear();
        matrix_.shrink_to_fit();
        cell_to_idx_.clear();
        idx_to_cell_.clear();
        neighbors_.clear();
        reverse_neighbors_.clear();
        forbidden_dirs_.clear();
        direction_constrained_ = false;
        forbidden_edge_count_ = 0;
        n_free_ = 0;
        n_states_ = 0;
    }
}

} // namespace nz
