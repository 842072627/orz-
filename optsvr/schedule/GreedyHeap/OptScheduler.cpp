// optsvr/OptScheduler.cpp — 优化任务调度器实现
#include "OptScheduler.h"
#include "MapProfile.h"
#include "MapTuner.h"
#include "HeuristicMatrix.h"
#include "IdleParking.h"
#include "heuristics.h"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <cassert>
#include <unordered_set>

namespace optsvr {

// ================================================================
// initialize — 预处理阶段调用一次
// ================================================================
void OptScheduler::initialize(SharedEnvironment* env, int preprocess_time_limit_ms) {
    map_size_ = env->rows * env->cols;
    rows_ = env->rows;
    cols_ = env->cols;

    // ---- 地图特征识别 + MapTuner 调参 ----
    load_map_tuning(env);
    if (cfg().debug_level >= 1) {
        map_profile().print();
    }

    // skip_test 已由 MapTuner 设置，跳过后续所有预处理
    if (cfg().skip_test) return;

    // 构建 grid 划分（依赖调参后的 grid_cell_size）
    build_grid_layout(env);

    idle_parking().initialize(env);

    // 确保 heuristic 表已初始化（DefaultPlanner 的全局数据）
    DefaultPlanner::init_heuristics(env);

    // ---- 距离矩阵预计算（如已在框架层构建则跳过） ----
    if (cfg().precompute_heuristic && !heuristic_matrix().is_ready()) {
        // 预留一些时间给后续初始化（至少留 500ms）
        auto now = std::chrono::steady_clock::now();
        auto deadline = env->plan_start_time
                      + std::chrono::milliseconds(preprocess_time_limit_ms)
                      - std::chrono::milliseconds(500);

        // 确保 deadline 合理
        if (deadline > now + std::chrono::milliseconds(100)) {
            heuristic_matrix().build(env, cfg().num_threads, deadline,
                                     cfg().turn_aware_heuristic);
        } else {
            fprintf(stderr,
                "[optsvr] Not enough time for heuristic precompute, skipping\n");
        }
    }

    // 初始化匹配器
    matcher_.set_env(env);
    matcher_.set_congestion_func(
        [this](int loc) -> float { return this->get_region_congestion(loc); }
    );

    // 初始化 GreedyHeap 匹配器
    heap_matcher_.set_env(env);
    // 注入拥塞查询函数（供密度惩罚打分使用）
    heap_matcher_.set_congestion_func(
        [this](int loc) -> float { return this->get_region_congestion(loc); }
    );
}

// ================================================================
// build_grid_layout — 预处理：将地图划分为 N×N 的 grid
// ================================================================
void OptScheduler::build_grid_layout(const SharedEnvironment* env) {
    int cell_size = cfg().grid_cell_size;
    if (cell_size <= 0) cell_size = 4;

    grid_rows_ = (rows_ + cell_size - 1) / cell_size;
    grid_cols_ = (cols_ + cell_size - 1) / cell_size;
    grid_count_ = grid_rows_ * grid_cols_;
    grid_congestion_.assign(grid_count_, 0.0f);
    grid_valid_cells_.assign(grid_count_, 0);
    grid_total_cells_.assign(grid_count_, 0);

    if (env == nullptr || static_cast<int>(env->map.size()) < map_size_) return;

    for (int r = 0; r < rows_; ++r) {
        for (int c = 0; c < cols_; ++c) {
            const int loc = r * cols_ + c;
            const int gid = (r / cell_size) * grid_cols_ + (c / cell_size);
            if (gid < 0 || gid >= grid_count_) continue;
            grid_total_cells_[gid]++;
            if (env->map[loc] == 0) {
                grid_valid_cells_[gid]++;
            }
        }
    }
}

// ================================================================
// cell_to_grid — cell 位置 → grid 编号
// ================================================================
int OptScheduler::cell_to_grid(int loc) const {
    int cell_size = cfg().grid_cell_size;
    if (cell_size <= 0) cell_size = 4;
    int row = loc / cols_;
    int col = loc % cols_;
    int gr = row / cell_size;
    int gc = col / cell_size;
    return gr * grid_cols_ + gc;
}

// ================================================================
// make_agent_info_from_snapshot — 从 locations 快照构建 AgentInfo
// ================================================================
AgentInfo OptScheduler::make_agent_info_from_snapshot(int agent_id) const {
    return AgentInfo{agent_id, curr_locations_snapshot_[agent_id]};
}

// ================================================================
// make_task_info_from_snapshot — 从 task_pool 快照中构建 TaskInfo
// 使用快照而非 env->task_pool，避免多线程竞争导致 segfault
// ================================================================
TaskInfo OptScheduler::make_task_info_from_snapshot(int task_id) const {
    const auto it = task_pool_snapshot_.find(task_id);
    if (it == task_pool_snapshot_.end()) {
        // 任务已被主线程回收（快照时不存在），返回空 TaskInfo
        TaskInfo info;
        info.id = task_id;
        info.start_location = -1;
        return info;
    }
    const Task& task = it->second;
    TaskInfo info;
    info.id = task_id;
    info.start_location = task.locations[task.idx_next_loc];
    info.errand_locations.reserve(task.locations.size() - task.idx_next_loc);
    for (size_t i = task.idx_next_loc; i < task.locations.size(); i++) {
        info.errand_locations.push_back(task.locations[i]);
    }
    return info;
}

// ================================================================
// schedule — 每轮调用，分配 free agents 到 free tasks
//
// 流程：
// 1. 分类 agents，收集 assigned_tasks
// 2. 收集 free tasks
// 3. 更新拥塞热力图
// 4. 匹配 free agents ↔ free tasks
//
// 注意：proposed_schedule 是主线程/子线程无锁共享的，
// 子线程只能安全地写 proposed_schedule[i]==-1 的 slot（free agent）。
// ================================================================
void OptScheduler::schedule(int time_limit,
                            std::vector<int>& proposed_schedule,
                            SharedEnvironment* env) {
    auto sched_entry_time = std::chrono::steady_clock::now();
    auto deadline = sched_entry_time +
                    std::chrono::milliseconds(time_limit);

    int n_agents = env->num_of_agents;

    // ---- 安全检查 ----
    if (n_agents <= 0 || env->curr_states.empty() ||
        static_cast<int>(env->curr_states.size()) < n_agents ||
        static_cast<int>(proposed_schedule.size()) < n_agents) {
        std::cerr << "[optsvr] schedule: invalid state, n_agents=" << n_agents
                  << " curr_states.size=" << env->curr_states.size()
                  << " proposed_schedule.size=" << proposed_schedule.size()
                  << ", skipping\n";
        return;
    }

    // ---- Step 0: 拷贝快照 ----
    // 主线程在 planner 线程运行期间会并发修改：
    //   - env->curr_states（Simulator::sync_shared_env 中 vector 整体赋值）
    //   - proposed_schedule（TaskManager::update_tasks 中 assignment[k]=-1）
    //   - env->task_pool（TaskManager::sync_shared_env 中 clear+重建，
    //     但这只在 !started 时调用，即子线程不运行时，所以 task_pool 实际安全）
    //
    // 关键风险：env->curr_states 是 vector<State>，主线程的赋值操作
    // (env->curr_states = predict_states) 可能触发内存重分配，
    // 导致子线程中迭代访问到已释放的内存 → segfault。
    //
    // 防御策略：先整体拷贝 vector（一次 memcpy），再从本地副本读取。
    try {
        task_pool_snapshot_ = env->task_pool;

        // 一次性拷贝整个 curr_states vector，减小竞态窗口
        // （虽然不能完全消除竞态，但单次 vector 拷贝比逐元素遍历安全得多）
        std::vector<State> states_copy = env->curr_states;
        int states_sz = static_cast<int>(states_copy.size());
        curr_locations_snapshot_.resize(n_agents);
        for (int i = 0; i < n_agents; i++) {
            if (i < states_sz) {
                curr_locations_snapshot_[i] = states_copy[i].location;
            } else {
                // 竞态导致 states 变短了，用 -1 标记无效
                curr_locations_snapshot_[i] = -1;
            }
        }

        schedule_snapshot_.assign(proposed_schedule.begin(), proposed_schedule.end());
    } catch (...) {
        // 捕获所有异常（包括 bad_alloc 等），竞态期间拷贝可能失败
        std::cerr << "[optsvr] schedule: snapshot copy failed, skipping\n";
        return;
    }

    // ---- 关键修复：使用本地副本操作，避免竞态 ----
    // proposed_schedule 由主线程（TaskManager::update_tasks）并发读取，
    // 子线程不应中途清零已有分配。所有操作都在 local_schedule 上进行，
    // 最终一次性将变更写回 proposed_schedule。
    std::vector<int> local_schedule = schedule_snapshot_;
    const int schedule_timestep = std::max(env->system_timestep, env->curr_timestep);

    // ---- Step 1: 分类 agents ----
    // GreedyHeap + reassign 走 LNS swap 路径：不释放 reassign_tasks 为 free，
    // 只收集真正的 free agents 做 heap 匹配，然后用 LNS 在所有 non-locked 间做 swap。
    // TaskMatcher 路径保留原有逻辑（释放 + bias 重匹配）。
    bool do_reassign = cfg().reassign_enabled;
    bool use_heap = (cfg().schedule_mode == 1);  // mode=1: GreedyHeap, mode=4: TaskMatcher
    bool use_idle_parking = use_heap && idle_parking().enabled();
    if (use_idle_parking) {
        idle_parking().apply_schedule_filter(local_schedule);
        idle_parking().apply_schedule_filter(proposed_schedule);
    }
    bool heap_lns_reassign = do_reassign && use_heap;
    int reassign_min_dist = cfg().reassign_min_dist;
    float effective_heap_dist_weight = cfg().heap_dist_weight;
    if (use_heap &&
        cfg().heap_dist_decay_interval_steps > 0 &&
        cfg().heap_dist_decay_factor >= 0.0f &&
        schedule_timestep >= cfg().heap_dist_decay_interval_steps) {
        effective_heap_dist_weight *= cfg().heap_dist_decay_factor;
    }

    std::vector<int> free_agents;                // tid==-1 的 agent
    std::vector<int> reassign_agents;            // 未打开任务、可参与 LNS swap 的 agent
    std::unordered_set<int> assigned_tasks;      // 锁定的任务（已打开/距离保护/不参与重分配）
    std::unordered_set<int> reassign_tasks;      // 未打开的任务（可重分配，仅 TaskMatcher 路径使用）

    for (int i = 0; i < n_agents; i++) {
        if (use_idle_parking && idle_parking().is_parked(i)) {
            local_schedule[i] = -1;
            continue;
        }
        int tid = schedule_snapshot_[i];
        if (tid == -1) {
            free_agents.push_back(i);
        } else if (do_reassign) {
            auto it = task_pool_snapshot_.find(tid);
            if (it != task_pool_snapshot_.end() && it->second.idx_next_loc == 0) {
                // 未打开 → 检查距离保护
                bool too_close = false;
                if (reassign_min_dist > 0) {
                    int agent_loc = curr_locations_snapshot_[i];
                    int task_target = it->second.locations[0];
                    if (agent_loc >= 0 && task_target >= 0) {
                        int dist = get_h_fast(env, agent_loc, task_target);
                        if (dist < reassign_min_dist) too_close = true;
                    }
                }
                if (too_close) {
                    assigned_tasks.insert(tid);
                } else if (heap_lns_reassign) {
                    // GreedyHeap + reassign：不释放，保留分配，标记为可 swap
                    reassign_agents.push_back(i);
                    assigned_tasks.insert(tid);  // 任务仍然标记为 assigned（不进入 free_tasks 池）
                } else {
                    // TaskMatcher 路径：释放回池，参与全量重匹配
                    free_agents.push_back(i);
                    reassign_tasks.insert(tid);
                    local_schedule[i] = -1;
                }
            } else {
                assigned_tasks.insert(tid);
            }
        } else {
            assigned_tasks.insert(tid);
        }
    }

    // ---- Step 1.5: max_assign 限制 — 按百分比限制携带任务的 agent 总数 ----
    // heap_max_assign ∈ (0, 1.0]，1.0=全部分配，0.4=最多分配40%的agent。
    // heap_dist_decay_max_assign 复用 decay_interval，在后期额外收紧总分配上限。
    float active_max_assign = cfg().heap_max_assign;
    const bool decay_max_assign_active =
        use_heap &&
        cfg().heap_dist_decay_interval_steps > 0 &&
        schedule_timestep >= cfg().heap_dist_decay_interval_steps &&
        cfg().heap_dist_decay_max_assign < 0.999f;
    if (decay_max_assign_active) {
        active_max_assign = std::min(active_max_assign, cfg().heap_dist_decay_max_assign);
    }
    if (use_heap && active_max_assign < 0.999f) {
        int max_total = static_cast<int>(active_max_assign * n_agents);
        if (max_total < 1) max_total = 1;
        int already_assigned = 0;
        for (int tid : local_schedule) {
            if (tid != -1) already_assigned++;
        }
        int max_new = max_total - already_assigned;
        if (max_new < 0) max_new = 0;
        if (decay_max_assign_active && already_assigned >= max_total && !reassign_agents.empty()) {
            if (cfg().debug_level >= 1) {
                fprintf(stderr,
                        "[optsvr] max_assign after_decay: disabling %d reassign agents while assigned=%d cap=%d\n",
                        static_cast<int>(reassign_agents.size()), already_assigned, max_total);
            }
            reassign_agents.clear();
        }
        if (static_cast<int>(free_agents.size()) > max_new) {
            if (cfg().debug_level >= 1) {
                fprintf(stderr, "[optsvr] max_assign=%.1f%%%s: trimming free_agents from %d to %d (cap %d/%d assigned=%d)\n",
                        active_max_assign * 100.0f,
                        decay_max_assign_active ? " after_decay" : "",
                        static_cast<int>(free_agents.size()), max_new,
                        max_total, n_agents, already_assigned);
            }
            free_agents.resize(max_new);
        }
    }

    // 记录重分配前的旧分配关系（用于 TaskMatcher bias 和验证日志）
    std::unordered_map<int, int> old_assignment;  // agent_id -> old_task_id
    if (do_reassign) {
        if (heap_lns_reassign && !reassign_agents.empty()) {
            // GreedyHeap 路径：记录 reassign_agents 的旧分配
            for (int a : reassign_agents) {
                old_assignment[a] = schedule_snapshot_[a];
            }
            if (cfg().debug_level >= 1) {
                std::cerr << "[optsvr] reassign(LNS): " << reassign_agents.size()
                          << " agents eligible for LNS swap\n";
            }
        } else if (!reassign_tasks.empty()) {
            // TaskMatcher 路径：记录旧分配
            for (int i = 0; i < n_agents; i++) {
                int old_tid = schedule_snapshot_[i];
                if (old_tid != -1 && reassign_tasks.count(old_tid)) {
                    old_assignment[i] = old_tid;
                }
            }
            if (cfg().debug_level >= 1) {
                std::cerr << "[optsvr] reassign: " << reassign_tasks.size()
                          << " unopened tasks released back to pool\n";
            }
        }
    }

    // ---- Step 2: 收集 free tasks ----
    std::vector<int> free_tasks;
    for (const auto& [tid, task] : task_pool_snapshot_) {
        if (assigned_tasks.find(tid) == assigned_tasks.end()) {
            free_tasks.push_back(tid);
        }
    }

    // ---- Step 2.5: 预替换 — 用本轮新下发的 task 替换 reassign agents 的旧任务 ----
    // 只扫描 env->new_tasks（本轮新下发），避免对全部 free_tasks 重复判断。
    // 新 task 如果比旧配对（含 bias）更优，则替换；旧 task 放回 free_tasks 池。
    if (heap_lns_reassign && !reassign_agents.empty() && !env->new_tasks.empty()) {
        // 收集本轮新下发的 task ids（取交集：new_tasks ∩ free_tasks）
        std::unordered_set<int> free_tasks_set(free_tasks.begin(), free_tasks.end());
        std::vector<int> new_free_tasks;
        new_free_tasks.reserve(env->new_tasks.size());
        for (int tid : env->new_tasks) {
            if (free_tasks_set.count(tid)) {
                new_free_tasks.push_back(tid);
            }
        }

        if (!new_free_tasks.empty()) {
            // keep_bias 根据地图规模折算
            float base_bias = cfg().reassign_keep_bias;
            float scale_factor = static_cast<float>(rows_ + cols_) / 200.0f;
            if (scale_factor < 0.1f) scale_factor = 0.1f;
            float keep_bias = base_bias * scale_factor;

            // 打分 lambda：score = dist_weight * dist + task_length，旧配对减 keep_bias
            auto compute_replace_score = [&](int agent_loc, int task_id, bool is_old_pair) -> float {
                constexpr int H_CAP = 100000;
                auto it = task_pool_snapshot_.find(task_id);
                if (it == task_pool_snapshot_.end()) return 1e9f;
                const Task& task = it->second;
                int task_start = task.locations[task.idx_next_loc < static_cast<int>(task.locations.size()) ? task.idx_next_loc : 0];
                int dist = get_h_fast(env, agent_loc, task_start);
                if (dist >= H_CAP) dist = H_CAP;
                float task_len = 0.0f;
                if (task.locations.size() > 1) {
                    size_t start_idx = task.idx_next_loc;
                    if (start_idx < task.locations.size()) {
                        int prev = task.locations[start_idx];
                        for (size_t k = start_idx + 1; k < task.locations.size(); k++) {
                            int next = task.locations[k];
                            int h = get_h_fast(env, prev, next);
                            if (h >= H_CAP) h = H_CAP;
                            task_len += h;
                            prev = next;
                        }
                    }
                }
                float score = effective_heap_dist_weight * static_cast<float>(dist) + task_len;
                if (is_old_pair) {
                    score -= keep_bias;
                }
                return score;
            };

            // 为每个新 task 建立 free_tasks 中的位置索引（用于替换回原位）
            std::unordered_map<int, int> tid_to_free_idx;
            for (int fi = 0; fi < static_cast<int>(free_tasks.size()); fi++) {
                tid_to_free_idx[free_tasks[fi]] = fi;
            }

            int pre_replaced = 0;
            for (int a : reassign_agents) {
                int agent_loc = curr_locations_snapshot_[a];
                if (agent_loc < 0) continue;

                int old_tid = local_schedule[a];
                float old_score = compute_replace_score(agent_loc, old_tid, true);

                // 只在本轮新下发的 tasks 中找更优替换
                int best_ni = -1;
                float best_score = old_score;
                for (int ni = 0; ni < static_cast<int>(new_free_tasks.size()); ni++) {
                    float ns = compute_replace_score(agent_loc, new_free_tasks[ni], false);
                    if (ns < best_score) {
                        best_score = ns;
                        best_ni = ni;
                    }
                }

                if (best_ni >= 0) {
                    int new_tid = new_free_tasks[best_ni];
                    auto fi_it = tid_to_free_idx.find(new_tid);
                    if (fi_it == tid_to_free_idx.end()) continue;  // safety
                    int fi = fi_it->second;

                    // 替换：agent 拿新 task，旧 task 放回 free_tasks 池
                    assigned_tasks.erase(old_tid);
                    free_tasks[fi] = old_tid;
                    tid_to_free_idx.erase(new_tid);
                    tid_to_free_idx[old_tid] = fi;

                    assigned_tasks.insert(new_tid);
                    local_schedule[a] = new_tid;
                    old_assignment[a] = new_tid;

                    // 已被使用的 new task 从候选中移除
                    new_free_tasks[best_ni] = new_free_tasks.back();
                    new_free_tasks.pop_back();

                    pre_replaced++;
                }
            }

            if (cfg().debug_level >= 1) {
                fprintf(stderr, "[optsvr] pre-replace: %d/%d reassign agents replaced "
                        "(new_tasks=%d, keep_bias=%.1f)\n",
                        pre_replaced, static_cast<int>(reassign_agents.size()),
                        static_cast<int>(env->new_tasks.size()), keep_bias);
            }
        }
    }

    // ---- Step 3: 更新拥塞热力图 ----
    update_congestion_from_snapshot(n_agents);

    // 无 free task 或无 free agent → 跳过匹配（但 LNS reassign 仍可能在已有分配上 swap）
    if (free_tasks.empty() && free_agents.empty() && reassign_agents.empty()) return;

    // ---- Step 4: 构建匹配列表 ----
    // free agents/tasks 在前，reassign agents/tasks 追加在尾部
    std::vector<AgentInfo> agents_info;
    std::vector<TaskInfo>  tasks_info;

    agents_info.reserve(free_agents.size() + reassign_agents.size());
    for (int a : free_agents) {
        AgentInfo ai = make_agent_info_from_snapshot(a);
        if (ai.location < 0 || ai.location >= map_size_) continue;
        agents_info.push_back(ai);
    }
    int n_free_agents_info = static_cast<int>(agents_info.size());

    if (use_heap && cfg().heap_task_filter_enabled &&
        n_free_agents_info > 0 && !free_tasks.empty()) {
        const int free_task_count_before = static_cast<int>(free_tasks.size());
        const float redundancy_pct = std::max(0.0f, cfg().heap_task_filter_redundancy_pct);
        int base_keep = static_cast<int>(std::ceil(
            static_cast<float>(n_free_agents_info) * (1.0f + redundancy_pct / 100.0f)));
        base_keep = std::max(n_free_agents_info, base_keep);

        if (free_task_count_before > base_keep) {
            constexpr int H_CAP = 100000;
            struct TaskFilterItem {
                int tid;
                int nearest_dist;
                float task_length;
            };

            auto remaining_task_length = [&](const Task& task) -> float {
                if (task.locations.empty()) return 0.0f;
                size_t start_idx = task.idx_next_loc;
                if (start_idx >= task.locations.size()) return 0.0f;
                int prev = task.locations[start_idx];
                int length = 0;
                for (size_t k = start_idx + 1; k < task.locations.size(); k++) {
                    int next = task.locations[k];
                    int h = get_h_fast(env, prev, next);
                    if (h >= H_CAP) h = H_CAP;
                    length += h;
                    prev = next;
                }
                return static_cast<float>(length);
            };

            std::vector<TaskFilterItem> items;
            items.reserve(free_tasks.size());
            for (int tid : free_tasks) {
                const auto it = task_pool_snapshot_.find(tid);
                if (it == task_pool_snapshot_.end()) continue;
                const Task& task = it->second;
                if (task.locations.empty() || task.idx_next_loc < 0 ||
                    task.idx_next_loc >= static_cast<int>(task.locations.size())) {
                    continue;
                }

                const int task_start = task.locations[task.idx_next_loc];
                int nearest_dist = H_CAP;
                for (const AgentInfo& ai : agents_info) {
                    int h = get_h_fast(env, ai.location, task_start);
                    if (h < nearest_dist) nearest_dist = h;
                    if (nearest_dist == 0) break;
                }
                if (nearest_dist >= H_CAP) nearest_dist = H_CAP;
                items.push_back({tid, nearest_dist, remaining_task_length(task)});
            }

            if (static_cast<int>(items.size()) > base_keep) {
                std::sort(items.begin(), items.end(),
                          [](const TaskFilterItem& a, const TaskFilterItem& b) {
                              if (a.nearest_dist != b.nearest_dist) {
                                  return a.nearest_dist < b.nearest_dist;
                              }
                              if (a.task_length != b.task_length) {
                                  return a.task_length < b.task_length;
                              }
                              return a.tid < b.tid;
                          });

                base_keep = std::min(base_keep, static_cast<int>(items.size()));
                float base_len_sum = 0.0f;
                for (int i = 0; i < base_keep; i++) {
                    base_len_sum += items[i].task_length;
                }
                const float base_avg_len =
                    base_keep > 0 ? base_len_sum / static_cast<float>(base_keep) : 0.0f;
                const int length_slack = std::max(0, cfg().heap_task_filter_length_slack);
                const float length_threshold = base_avg_len + static_cast<float>(length_slack);

                std::vector<int> filtered_tasks;
                filtered_tasks.reserve(items.size());
                for (int i = 0; i < base_keep; i++) {
                    filtered_tasks.push_back(items[i].tid);
                }
                for (int i = base_keep; i < static_cast<int>(items.size()); i++) {
                    if (items[i].task_length <= length_threshold) {
                        filtered_tasks.push_back(items[i].tid);
                    }
                }

                if (cfg().debug_level >= 1) {
                    fprintf(stderr,
                            "[optsvr] heap task filter: %d -> %d free tasks "
                            "(agents=%d, base=%d, extra=%.1f%%, avg_len=%.1f, slack=%d)\n",
                            free_task_count_before,
                            static_cast<int>(filtered_tasks.size()),
                            n_free_agents_info, base_keep, redundancy_pct,
                            base_avg_len, length_slack);
                }
                free_tasks.swap(filtered_tasks);
            }
        }
    }

    tasks_info.reserve(free_tasks.size() + reassign_agents.size());
    for (int t : free_tasks) {
        TaskInfo ti = make_task_info_from_snapshot(t);
        if (ti.start_location < 0) continue;
        tasks_info.push_back(std::move(ti));
    }

    // GreedyHeap + reassign：追加 reassign agents 和它们的旧 tasks 到尾部
    if (heap_lns_reassign) {
        for (int a : reassign_agents) {
            AgentInfo ai = make_agent_info_from_snapshot(a);
            if (ai.location < 0 || ai.location >= map_size_) continue;
            agents_info.push_back(ai);

            int tid = local_schedule[a];  // 使用 local_schedule（可能被 pre-replace 更新过）
            TaskInfo ti = make_task_info_from_snapshot(tid);
            if (ti.start_location >= 0) {
                tasks_info.push_back(std::move(ti));
            }
        }
    }

    // ---- Step 4.5: 根据匹配策略分流 ----
    MatchResult matches;

    // ---- 为 score_mode=2 预计算 avg_task_length ----
    float avg_task_length = 1.0f;
    if (cfg().score_mode == 2 && !tasks_info.empty()) {
        constexpr int H_CAP = 100000;
        float total = 0.0f;
        for (const auto& ti : tasks_info) {
            int len = 0;
            if (ti.errand_locations.size() > 1) {
                int prev = ti.errand_locations[0];
                for (size_t k = 1; k < ti.errand_locations.size(); k++) {
                    int next = ti.errand_locations[k];
                    int h = get_h_fast(env, prev, next);
                    if (h >= H_CAP) h = H_CAP;
                    len += h;
                    prev = next;
                }
            }
            total += static_cast<float>(len);
        }
        avg_task_length = total / static_cast<float>(tasks_info.size());
        if (avg_task_length < 1.0f) avg_task_length = 1.0f;
    }
    matcher_.set_context(schedule_timestep, avg_task_length);
    heap_matcher_.set_timestep_info(schedule_timestep, cfg().game_total_steps);

    if (use_heap) {
        // ---- GreedyHeap 路径 (SCHEDULE_MODE=1) ----
        if (cfg().debug_level >= 1) {
            std::cerr << "[optsvr] using GreedyHeapMatcher"
                      << ", reassign=" << (do_reassign ? "on(LNS)" : "off") << "\n";
        }

        if (heap_lns_reassign) {
            // reassign 模式：传入 n_free_agents_info，match 内部 Phase 1+2 只处理 free，
            // Phase 3 LNS 合并 reassign agents 一起 swap（由 HEAP_LNS_PCT 控制）
            matches = heap_matcher_.match(agents_info, tasks_info, deadline,
                                          n_free_agents_info, old_assignment);
        } else {
            if (!agents_info.empty() && !tasks_info.empty()) {
                matches = heap_matcher_.match(agents_info, tasks_info, deadline);
            }
        }
    } else {
        // ---- TaskMatcher 路径 (mode 0~3, 5) ----
        const float keep_bias = cfg().reassign_keep_bias;
        if (!old_assignment.empty()) {
            std::unordered_map<int, int> aid_to_idx;
            for (int i = 0; i < static_cast<int>(agents_info.size()); i++) {
                aid_to_idx[agents_info[i].id] = i;
            }
            std::unordered_map<int, int> tid_to_idx;
            for (int j = 0; j < static_cast<int>(tasks_info.size()); j++) {
                tid_to_idx[tasks_info[j].id] = j;
            }
            std::unordered_map<int, int> old_pair_map;
            for (auto& [aid, old_tid] : old_assignment) {
                auto ait = aid_to_idx.find(aid);
                auto tit = tid_to_idx.find(old_tid);
                if (ait != aid_to_idx.end() && tit != tid_to_idx.end()) {
                    old_pair_map[ait->second] = tit->second;
                }
            }
            auto biased_score = [this, &agents_info, &tasks_info, &old_pair_map, keep_bias](int ai, int ti) -> float {
                float base = matcher_.score_dist_only(agents_info[ai], tasks_info[ti]);
                auto it = old_pair_map.find(ai);
                if (it != old_pair_map.end() && it->second == ti) {
                    base -= keep_bias;
                }
                return base;
            };
            matches = matcher_.match(agents_info, tasks_info, biased_score, deadline);
        } else {
            if (!agents_info.empty() && !tasks_info.empty()) {
                matches = matcher_.match(agents_info, tasks_info, deadline);
            }
        }
    }

    // ---- Step 6: 应用匹配结果到本地副本 ----
    std::unordered_set<int> matched_tasks;
    for (auto& [aid, tid] : matches) {
        local_schedule[aid] = tid;
        matched_tasks.insert(tid);
    }

    // ---- Step 6.5: 兜底分配 ----
    if (cfg().schedule_fallback_enabled) {
        auto task_it = tasks_info.begin();
        int fallback_count = 0;
        for (int a : free_agents) {
            if (local_schedule[a] != -1) continue;
            while (task_it != tasks_info.end() &&
                   matched_tasks.count(task_it->id)) {
                ++task_it;
            }
            if (task_it == tasks_info.end()) break;
            local_schedule[a] = task_it->id;
            matched_tasks.insert(task_it->id);
            ++task_it;
            ++fallback_count;
        }
        if (fallback_count > 0 && cfg().debug_level >= 1) {
            std::cerr << "[optsvr] fallback assigned: " << fallback_count
                      << " agents got sequential task\n";
        }
    } else if (cfg().debug_level >= 1) {
        int unmatched = 0;
        for (int a : free_agents) {
            if (local_schedule[a] == -1) ++unmatched;
        }
        if (unmatched > 0) {
            std::cerr << "[optsvr] fallback disabled: " << unmatched
                      << " free agents remain unassigned\n";
        }
    }

    // ---- Step 7: 一次性写回 proposed_schedule ----
    for (int a : free_agents) {
        proposed_schedule[a] = local_schedule[a];
    }
    // reassign_agents 的分配可能被 LNS swap 改变
    for (int a : reassign_agents) {
        proposed_schedule[a] = local_schedule[a];
    }
    if (use_idle_parking) {
        idle_parking().apply_schedule_filter(proposed_schedule);
    }

    // ---- 验证日志：输出重分配详情 ----
    if (do_reassign && !old_assignment.empty()) {
        int kept = 0, changed = 0, lost = 0;
        for (auto& [aid, old_tid] : old_assignment) {
            int new_tid = local_schedule[aid];
            if (new_tid == old_tid) {
                kept++;
            } else if (new_tid == -1) {
                lost++;
                if (cfg().debug_level >= 2) {
                    std::cerr << "[optsvr] reassign WARN: agent " << aid
                              << " lost task " << old_tid << " → unassigned\n";
                }
            } else {
                changed++;
                if (cfg().debug_level >= 2 && changed <= 5) {
                    std::cerr << "[optsvr] reassign: agent " << aid
                              << " task " << old_tid << " → " << new_tid << "\n";
                }
            }
        }
        if (cfg().debug_level >= 1) {
            std::cerr << "[optsvr] reassign summary: "
                      << old_assignment.size() << " candidates, "
                      << kept << " kept, " << changed << " changed, "
                      << lost << " lost\n";
        }
    }
}

// ================================================================
// update_congestion_from_snapshot — 从 locations 快照更新拥塞热力图
// ================================================================
void OptScheduler::update_congestion_from_snapshot(int n_agents) {
    bool use_history = cfg().congestion_use_history;

    if (use_history) {
        float decay = cfg().congestion_decay;
        for (int i = 0; i < grid_count_; i++) {
            grid_congestion_[i] *= decay;
        }
    } else {
        std::fill(grid_congestion_.begin(), grid_congestion_.end(), 0.0f);
    }

    for (int i = 0; i < n_agents; i++) {
        int loc = curr_locations_snapshot_[i];
        if (loc >= 0 && loc < map_size_) {
            int gid = cell_to_grid(loc);
            grid_congestion_[gid] += 1.0f;
        }
    }
}

// ================================================================
// get_region_congestion — 区域平均拥塞度（grid-based）
// ================================================================
float OptScheduler::get_region_congestion(int loc) const {
    if (loc < 0 || loc >= map_size_ || grid_count_ <= 0) return 0.0f;

    int cell_size = cfg().grid_cell_size;
    if (cell_size <= 0) cell_size = 4;
    int radius = cfg().grid_radius;
    if (radius < 0) radius = 0;

    int row = loc / cols_;
    int col = loc % cols_;
    int gr = row / cell_size;
    int gc = col / cell_size;

    int gr_start = std::max(0, gr - radius);
    int gr_end   = std::min(grid_rows_ - 1, gr + radius);
    int gc_start = std::max(0, gc - radius);
    int gc_end   = std::min(grid_cols_ - 1, gc + radius);

    float agent_sum = 0.0f;
    int grid_count = 0;
    int valid_cells = 0;
    int total_cells = 0;
    for (int r = gr_start; r <= gr_end; r++) {
        for (int c = gc_start; c <= gc_end; c++) {
            int gid = r * grid_cols_ + c;
            agent_sum += grid_congestion_[gid];
            grid_count++;
            if (gid >= 0 && gid < static_cast<int>(grid_valid_cells_.size())) {
                valid_cells += grid_valid_cells_[gid];
            }
            if (gid >= 0 && gid < static_cast<int>(grid_total_cells_.size())) {
                total_cells += grid_total_cells_[gid];
            }
        }
    }
    if (grid_count <= 0) return 0.0f;

    const float avg_agents_per_grid = agent_sum / static_cast<float>(grid_count);
    if (valid_cells <= 0 || total_cells <= 0) {
        return avg_agents_per_grid;
    }

    const float valid_ratio = static_cast<float>(valid_cells) /
                              static_cast<float>(total_cells);
    if (valid_ratio <= 1e-6f) return 0.0f;
    return avg_agents_per_grid / valid_ratio;
}

} // namespace optsvr
