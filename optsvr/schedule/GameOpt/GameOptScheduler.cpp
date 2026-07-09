// optsvr/schedule/GameOpt/GameOptScheduler.cpp — Game 赛道定制调度器实现
// 基于 OptScheduler 核心逻辑，使用独立 GAME_* 参数集
#include "GameOptScheduler.h"
#include "HeuristicMatrix.h"
#include "heuristics.h"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <unordered_set>

namespace optsvr {

// ================================================================
// load_game_params — 从 cfg().game_* 加载 GameOpt 专属参数
// ================================================================
void GameOptScheduler::load_game_params() {
    dist_weight_  = cfg().game_dist_weight;
    rebuild_pct_  = cfg().game_rebuild_pct;
    lns_pct_      = cfg().game_lns_pct;
    sort_k_       = cfg().game_sort_k;
    reassign_on_  = cfg().game_reassign;
    keep_bias_    = cfg().game_keep_bias;
    min_dist_     = cfg().game_min_dist;
    cong_decay_   = cfg().game_cong_decay;
    grid_cell_    = cfg().game_grid_cell;
    grid_radius_  = cfg().game_grid_radius;
    dest_cong_weight_ = cfg().game_dest_cong_weight;
    dest_cong_cap_    = cfg().game_dest_cong_cap;

    if (cfg().debug_level >= 1) {
        fprintf(stderr, "[GameOpt] params: dist_w=%.1f rb=%d%% lns=%d%% sk=%d "
                "re=%d bias=%.1f md=%d decay=%.2f cell=%d r=%d dest_cw=%.1f cap=%.1f\n",
                dist_weight_, rebuild_pct_, lns_pct_, sort_k_,
                (int)reassign_on_, keep_bias_, min_dist_,
                cong_decay_, grid_cell_, grid_radius_,
                dest_cong_weight_, dest_cong_cap_);
    }
}

// ================================================================
// initialize
// ================================================================
void GameOptScheduler::initialize(SharedEnvironment* env, int /*preprocess_time_limit_ms*/) {
    load_game_params();

    // mode=3 独占：一次性覆盖 cfg() 中 heap 匹配器读取的字段
    cfg().heap_dist_weight = dist_weight_;
    cfg().heap_rebuild_pct = rebuild_pct_;
    cfg().heap_lns_pct     = lns_pct_;
    cfg().heap_sort_k      = sort_k_;
    // game_cong_weight 和 game_score_mode 已直接在 cfg() 中（不需额外覆盖）

    map_size_ = env->rows * env->cols;
    rows_ = env->rows;
    cols_ = env->cols;

    build_grid_layout();

    // 初始化 GreedyHeap 匹配器
    heap_matcher_.set_env(env);
    heap_matcher_.set_congestion_func(
        [this](int loc) -> float {
            float cong = this->get_region_congestion(loc);
            if (this->dest_cong_weight_ > 0.0f) {
                cong += this->dest_cong_weight_ * this->get_dest_congestion(loc);
            }
            return cong;
        }
    );
}

// ================================================================
// build_grid_layout
// ================================================================
void GameOptScheduler::build_grid_layout() {
    int cell_size = grid_cell_;
    if (cell_size <= 0) cell_size = 4;
    grid_rows_ = (rows_ + cell_size - 1) / cell_size;
    grid_cols_ = (cols_ + cell_size - 1) / cell_size;
    grid_count_ = grid_rows_ * grid_cols_;
    grid_congestion_.assign(grid_count_, 0.0f);
    grid_dest_load_.assign(grid_count_, 0.0f);
}

// ================================================================
// cell_to_grid
// ================================================================
int GameOptScheduler::cell_to_grid(int loc) const {
    int cell_size = grid_cell_;
    if (cell_size <= 0) cell_size = 4;
    int row = loc / cols_;
    int col = loc % cols_;
    return (row / cell_size) * grid_cols_ + (col / cell_size);
}

// ================================================================
// make_agent_info / make_task_info
// ================================================================
AgentInfo GameOptScheduler::make_agent_info(int agent_id) const {
    return AgentInfo{agent_id, curr_locations_snapshot_[agent_id]};
}

TaskInfo GameOptScheduler::make_task_info(int task_id) const {
    const auto it = task_pool_snapshot_.find(task_id);
    if (it == task_pool_snapshot_.end()) {
        TaskInfo info; info.id = task_id; info.start_location = -1; return info;
    }
    const Task& task = it->second;
    TaskInfo info;
    info.id = task_id;
    info.start_location = task.locations[task.idx_next_loc];
    info.t_revealed = task.t_revealed;
    info.errand_locations.reserve(task.locations.size() - task.idx_next_loc);
    for (size_t i = task.idx_next_loc; i < task.locations.size(); i++) {
        info.errand_locations.push_back(task.locations[i]);
    }
    return info;
}

// ================================================================
// update_congestion
// ================================================================
void GameOptScheduler::update_congestion(int n_agents) {
    // 使用历史衰减模式（Game 地图上效果更稳定）
    float decay = cong_decay_;
    for (int i = 0; i < grid_count_; i++) {
        grid_congestion_[i] *= decay;
    }
    for (int i = 0; i < n_agents; i++) {
        int loc = curr_locations_snapshot_[i];
        if (loc >= 0 && loc < map_size_) {
            grid_congestion_[cell_to_grid(loc)] += 1.0f;
        }
    }
}

// ================================================================
// get_region_congestion
// ================================================================
float GameOptScheduler::get_region_congestion(int loc) const {
    int cell_size = grid_cell_;
    if (cell_size <= 0) cell_size = 4;
    int row = loc / cols_;
    int col = loc % cols_;
    int gr = row / cell_size;
    int gc = col / cell_size;

    int gr_start = std::max(0, gr - grid_radius_);
    int gr_end   = std::min(grid_rows_ - 1, gr + grid_radius_);
    int gc_start = std::max(0, gc - grid_radius_);
    int gc_end   = std::min(grid_cols_ - 1, gc + grid_radius_);

    float sum = 0.0f;
    int count = 0;
    for (int r = gr_start; r <= gr_end; r++) {
        for (int c = gc_start; c <= gc_end; c++) {
            sum += grid_congestion_[r * grid_cols_ + c];
            count++;
        }
    }
    return count > 0 ? sum / count : 0.0f;
}

// ================================================================
// get_dest_congestion — 查询目的地区域的负载
// ================================================================
float GameOptScheduler::get_dest_congestion(int loc) const {
    if (dest_cong_weight_ <= 0.0f) return 0.0f;

    int cell_size = grid_cell_;
    if (cell_size <= 0) cell_size = 4;
    int row = loc / cols_;
    int col = loc % cols_;
    int gr = row / cell_size;
    int gc = col / cell_size;

    int gr_start = std::max(0, gr - grid_radius_);
    int gr_end   = std::min(grid_rows_ - 1, gr + grid_radius_);
    int gc_start = std::max(0, gc - grid_radius_);
    int gc_end   = std::min(grid_cols_ - 1, gc + grid_radius_);

    float sum = 0.0f;
    int count = 0;
    for (int r = gr_start; r <= gr_end; r++) {
        for (int c = gc_start; c <= gc_end; c++) {
            sum += grid_dest_load_[r * grid_cols_ + c];
            count++;
        }
    }
    float avg_load = count > 0 ? sum / count : 0.0f;

    // Cap 模式：超过上限后指数惩罚
    if (dest_cong_cap_ > 0.0f && avg_load > dest_cong_cap_) {
        float excess = avg_load - dest_cong_cap_;
        return avg_load + excess * excess;  // 超限后二次方惩罚
    }
    return avg_load;
}

// ================================================================
// update_dest_load — 统计已分配 agent 的目标区域负载
// ================================================================
void GameOptScheduler::update_dest_load(int n_agents) {
    if (dest_cong_weight_ <= 0.0f) return;

    // 清零
    std::fill(grid_dest_load_.begin(), grid_dest_load_.end(), 0.0f);

    // 遍历所有已分配的 agent，将其目标 task 起点累加到 grid
    for (int i = 0; i < n_agents; i++) {
        int tid = schedule_snapshot_[i];
        if (tid == -1) continue;
        auto it = task_pool_snapshot_.find(tid);
        if (it == task_pool_snapshot_.end()) continue;
        const Task& task = it->second;
        if (task.idx_next_loc >= (int)task.locations.size()) continue;
        int target_loc = task.locations[task.idx_next_loc];
        if (target_loc >= 0 && target_loc < map_size_) {
            grid_dest_load_[cell_to_grid(target_loc)] += 1.0f;
        }
    }
}

// ================================================================
// schedule — 每轮调度
// ================================================================
void GameOptScheduler::schedule(int time_limit,
                                std::vector<int>& proposed_schedule,
                                SharedEnvironment* env) {
    auto sched_entry = std::chrono::steady_clock::now();
    auto deadline = sched_entry + std::chrono::milliseconds(time_limit);

    int n_agents = env->num_of_agents;
    if (n_agents <= 0 || env->curr_states.empty() ||
        (int)env->curr_states.size() < n_agents ||
        (int)proposed_schedule.size() < n_agents) return;

    // ---- Step 0: 快照 ----
    try {
        task_pool_snapshot_ = env->task_pool;
        std::vector<State> states_copy = env->curr_states;
        int states_sz = (int)states_copy.size();
        curr_locations_snapshot_.resize(n_agents);
        for (int i = 0; i < n_agents; i++) {
            curr_locations_snapshot_[i] = (i < states_sz) ? states_copy[i].location : -1;
        }
        schedule_snapshot_.assign(proposed_schedule.begin(), proposed_schedule.end());
    } catch (...) {
        return;
    }

    std::vector<int> local_schedule = schedule_snapshot_;

    // ---- Step 1: 分类 agents ----
    std::vector<int> free_agents;
    std::vector<int> reassign_agents;
    std::unordered_set<int> assigned_tasks;

    for (int i = 0; i < n_agents; i++) {
        int tid = schedule_snapshot_[i];
        if (tid == -1) {
            free_agents.push_back(i);
        } else if (reassign_on_) {
            auto it = task_pool_snapshot_.find(tid);
            if (it != task_pool_snapshot_.end() && it->second.idx_next_loc == 0) {
                // 未打开 → 距离保护
                bool too_close = false;
                if (min_dist_ > 0) {
                    int aloc = curr_locations_snapshot_[i];
                    int tloc = it->second.locations[0];
                    if (aloc >= 0 && tloc >= 0) {
                        int dist = get_h_fast(env, aloc, tloc);
                        if (dist < min_dist_) too_close = true;
                    }
                }
                if (too_close) {
                    assigned_tasks.insert(tid);
                } else {
                    reassign_agents.push_back(i);
                    assigned_tasks.insert(tid);
                }
            } else {
                assigned_tasks.insert(tid);
            }
        } else {
            assigned_tasks.insert(tid);
        }
    }

    // 记录旧分配
    std::unordered_map<int, int> old_assignment;
    for (int a : reassign_agents) {
        old_assignment[a] = schedule_snapshot_[a];
    }

    // ---- Step 2: free tasks ----
    std::vector<int> free_tasks;
    for (const auto& [tid, task] : task_pool_snapshot_) {
        if (!assigned_tasks.count(tid)) {
            free_tasks.push_back(tid);
        }
    }

    // ---- Step 2.5: 预替换（新下发 task 替换 reassign agents 旧任务） ----
    if (!reassign_agents.empty() && !env->new_tasks.empty()) {
        std::unordered_set<int> free_set(free_tasks.begin(), free_tasks.end());
        std::vector<int> new_free;
        new_free.reserve(env->new_tasks.size());
        for (int tid : env->new_tasks) {
            if (free_set.count(tid)) new_free.push_back(tid);
        }

        if (!new_free.empty()) {
            float scale = static_cast<float>(rows_ + cols_) / 200.0f;
            if (scale < 0.1f) scale = 0.1f;
            float keep_bias = keep_bias_ * scale;

            auto score_fn = [&](int aloc, int task_id, bool is_old) -> float {
                constexpr int H_CAP = 100000;
                auto it = task_pool_snapshot_.find(task_id);
                if (it == task_pool_snapshot_.end()) return 1e9f;
                const Task& task = it->second;
                int ts = task.locations[task.idx_next_loc < (int)task.locations.size() ? task.idx_next_loc : 0];
                int dist = get_h_fast(env, aloc, ts);
                if (dist >= H_CAP) dist = H_CAP;
                float tlen = 0.0f;
                if (task.locations.size() > 1) {
                    size_t si = task.idx_next_loc;
                    if (si < task.locations.size()) {
                        int prev = task.locations[si];
                        for (size_t k = si+1; k < task.locations.size(); k++) {
                            int next = task.locations[k];
                            int h = get_h_fast(env, prev, next);
                            if (h >= H_CAP) h = H_CAP;
                            tlen += h;
                            prev = next;
                        }
                    }
                }
                float s = dist_weight_ * (float)dist + tlen;
                if (is_old) s -= keep_bias;
                return s;
            };

            std::unordered_map<int, int> tid_fi;
            for (int fi = 0; fi < (int)free_tasks.size(); fi++) tid_fi[free_tasks[fi]] = fi;

            for (int a : reassign_agents) {
                int aloc = curr_locations_snapshot_[a];
                if (aloc < 0) continue;
                int old_tid = local_schedule[a];
                float old_s = score_fn(aloc, old_tid, true);
                int best_ni = -1; float best_s = old_s;
                for (int ni = 0; ni < (int)new_free.size(); ni++) {
                    float ns = score_fn(aloc, new_free[ni], false);
                    if (ns < best_s) { best_s = ns; best_ni = ni; }
                }
                if (best_ni >= 0) {
                    int new_tid = new_free[best_ni];
                    auto fi_it = tid_fi.find(new_tid);
                    if (fi_it == tid_fi.end()) continue;
                    int fi = fi_it->second;
                    assigned_tasks.erase(old_tid);
                    free_tasks[fi] = old_tid;
                    tid_fi.erase(new_tid); tid_fi[old_tid] = fi;
                    assigned_tasks.insert(new_tid);
                    local_schedule[a] = new_tid;
                    old_assignment[a] = new_tid;
                    new_free[best_ni] = new_free.back(); new_free.pop_back();
                }
            }
        }
    }

    // ---- Step 3: 更新拥塞 ----
    update_congestion(n_agents);
    update_dest_load(n_agents);

    if (free_tasks.empty() && free_agents.empty() && reassign_agents.empty()) return;

    // ---- Step 4: 构建匹配列表（全员参与，堆阶段限制匹配数量） ----
    std::vector<AgentInfo> agents_info;
    std::vector<TaskInfo>  tasks_info;

    // 所有 free agents 全部参与 rebuild + heap，由堆自然选出全局 top-K 最优配对
    std::vector<int> selected_free = free_agents;

    // 计算堆匹配上限：assign_ratio 控制最多匹配多少个 free agent
    float assign_ratio = cfg().game_assign_ratio;
    int max_matches = -1;  // -1 = 不限制
    if (assign_ratio < 0.999f && !free_agents.empty()) {
        max_matches = std::max(1, (int)(assign_ratio * (float)free_agents.size()));
    }

    agents_info.reserve(selected_free.size() + reassign_agents.size());
    for (int a : selected_free) {
        AgentInfo ai = make_agent_info(a);
        if (ai.location < 0 || ai.location >= map_size_) continue;
        agents_info.push_back(ai);
    }
    int n_free_info = (int)agents_info.size();

    tasks_info.reserve(free_tasks.size() + reassign_agents.size());
    for (int t : free_tasks) {
        TaskInfo ti = make_task_info(t);
        if (ti.start_location < 0) continue;
        tasks_info.push_back(std::move(ti));
    }

    // 追加 reassign agents + 旧 tasks
    for (int a : reassign_agents) {
        AgentInfo ai = make_agent_info(a);
        if (ai.location < 0 || ai.location >= map_size_) continue;
        agents_info.push_back(ai);
        int tid = local_schedule[a];
        TaskInfo ti = make_task_info(tid);
        if (ti.start_location >= 0) tasks_info.push_back(std::move(ti));
    }

    // ---- Step 5: GreedyHeap 匹配 ----
    heap_matcher_.set_timestep_info(std::max(env->system_timestep, env->curr_timestep),
                                    cfg().game_total_steps);
    MatchResult matches;
    if (!reassign_agents.empty()) {
        matches = heap_matcher_.match(agents_info, tasks_info, deadline,
                                      n_free_info, old_assignment, max_matches);
    } else if (!agents_info.empty() && !tasks_info.empty()) {
        matches = heap_matcher_.match(agents_info, tasks_info, deadline,
                                      -1, {}, max_matches);
    }

    // ---- Step 6: 应用结果 ----
    std::unordered_set<int> matched_tasks;
    for (auto& [aid, tid] : matches) {
        local_schedule[aid] = tid;
        matched_tasks.insert(tid);
    }

    // 兜底：仅在不限制匹配数时为超时未匹配的 agent 顺序分配
    if (max_matches < 0 && cfg().schedule_fallback_enabled) {
        auto task_it = tasks_info.begin();
        for (int a : selected_free) {
            if (local_schedule[a] != -1) continue;
            while (task_it != tasks_info.end() && matched_tasks.count(task_it->id)) ++task_it;
            if (task_it == tasks_info.end()) break;
            local_schedule[a] = task_it->id;
            matched_tasks.insert(task_it->id);
            ++task_it;
        }
    } else if (max_matches < 0 && cfg().debug_level >= 1) {
        int unmatched = 0;
        for (int a : selected_free) {
            if (local_schedule[a] == -1) ++unmatched;
        }
        if (unmatched > 0) {
            std::cerr << "[GameOpt] fallback disabled: " << unmatched
                      << " free agents remain unassigned\n";
        }
    }

    // ---- Step 7: 写回 ----
    for (int a : selected_free) proposed_schedule[a] = local_schedule[a];
    for (int a : reassign_agents) proposed_schedule[a] = local_schedule[a];
}

} // namespace optsvr
