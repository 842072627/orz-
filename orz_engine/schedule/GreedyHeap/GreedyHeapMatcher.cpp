// optsvr/GreedyHeapMatcher.cpp — LORR24-style Greedy Heap Matcher 实现
//
// 三阶段匹配流程：
//   1. rebuild_candidates  — 多线程构建每个 robot 的排序候选列表
//   2. lazy_heap_solve     — 全局最小堆贪心匹配（lazy skip 已占用的 task）
//   3. lns_refine          — swap 优化（首轮 SA，后续轮次贪心）
#include "GreedyHeapMatcher.h"
#include "HeuristicMatrix.h"

#include <algorithm>
#include <cmath>
#include <queue>
#include <unordered_map>
#include <iostream>
#include <cassert>
#include <random>
#include <cstdio>
#include <thread>

namespace orz_engine {

void GreedyHeapMatcher::set_pair_extra_cost_func(
    std::function<float(int, int)> func) {
    pair_extra_cost_func_ = std::move(func);
    candidates_.clear();
    timestep_updated_.clear();
}

// ================================================================
// compute_task_length — 计算 task 自身长度（所有 errand 段距离之和）
// ================================================================
float GreedyHeapMatcher::compute_task_length(const TaskInfo& task) const {
    constexpr int H_CAP = 100000;
    int length = 0;
    if (task.errand_locations.size() > 1) {
        int prev = task.errand_locations[0];
        for (size_t i = 1; i < task.errand_locations.size(); i++) {
            int next = task.errand_locations[i];
            int h = get_h_fast(env_, prev, next);
            if (h >= H_CAP) h = H_CAP;
            length += h;
            prev = next;
        }
    }
    return static_cast<float>(length);
}

// ================================================================
// compute_score_fast — 内置打分
//   score = dist_weight * dist^dist_pow + task_length + cong_weight * congestion
//         - age_weight * age + comp_weight * completion_penalty
// ================================================================
float GreedyHeapMatcher::compute_score_fast(int agent_loc, int task_idx) const {
    constexpr int H_CAP = 100000;

    int task_start = task_start_locs_[task_idx];
    int dist = get_h_fast(env_, agent_loc, task_start);
    if (dist >= H_CAP) dist = H_CAP;

    float dist_score;
    if (cached_dist_pow_ >= 0.99f && cached_dist_pow_ <= 1.01f) {
        dist_score = static_cast<float>(dist) * cached_dist_weight_;
    } else {
        dist_score = std::pow(static_cast<float>(dist), cached_dist_pow_) * cached_dist_weight_;
    }

    float score = dist_score + task_lengths_[task_idx];

    if (pair_extra_cost_func_) {
        score += pair_extra_cost_func_(agent_loc, task_idx);
    }

    if (cached_cong_weight_ > 0.0f) {
        score += cached_cong_weight_ * task_congestion_[task_idx];
    }

    // Task Age: 越久没人接的 task 得分越低（越优先）
    if (cached_age_weight_ > 0.0f) {
        score -= cached_age_weight_ * static_cast<float>(task_ages_[task_idx]);
    }

    // Completion Potential: 后期对"来不及完成"的 task 施加惩罚
    if (cached_comp_weight_ > 0.0f) {
        int remaining = total_steps_ - curr_timestep_;
        float total_cost = static_cast<float>(dist) + task_lengths_[task_idx];
        if (remaining > 0 && total_cost > static_cast<float>(remaining)) {
            // 超出剩余时间的部分按比例惩罚
            float excess = (total_cost - static_cast<float>(remaining)) / static_cast<float>(remaining);
            score += cached_comp_weight_ * excess * task_lengths_[task_idx];
        }
    }

    return score;
}

// ================================================================
// rebuild_single — 为 agent_idx 重建候选列表（前 sort_k 有序）
//
// 优化策略：
//   1. 使用 compute_score_fast（缓存 task_length + 连续内存访问）
//   2. 使用 agent_locs_ 缓存避免间接寻址 agents[idx].location
//   3. 排序深度由 cfg().heap_sort_k 控制：
//      - sort_k > 0 且 < m：nth_element + partial_sort 只排前 sort_k
//        O(N) + O(K log K)，lazy_heap_solve 在前 sort_k 内匹配成功
//        超出 sort_k 的候选无序但 score >= cand[sort_k-1]
//      - sort_k <= 0 或 >= m：全量排序
//   4. 无外部 score_func，全部走内置快速路径（多线程安全）
// ================================================================
void GreedyHeapMatcher::rebuild_single(int agent_idx) {
    const auto& tasks  = *tasks_ptr_;
    int m = static_cast<int>(tasks.size());

    auto& cand = candidates_[agent_idx];
    cand.clear();
    cand.reserve(m);

    // ---- 打分阶段 ----
    // 全部走快速路径：连续数组 + 缓存查表
    int agent_loc = agent_locs_[agent_idx];
    for (int j = 0; j < m; j++) {
        float s = compute_score_fast(agent_loc, j);
        cand.emplace_back(s, j);
    }

    // ---- 排序阶段 ----
    int sort_k = cfg().heap_sort_k;
    if (sort_k > 0 && sort_k < m) {
        // 前 sort_k 个有序：nth_element O(N) + partial_sort O(N + K log K)
        std::nth_element(cand.begin(), cand.begin() + sort_k, cand.end());
        std::partial_sort(cand.begin(), cand.begin() + sort_k, cand.begin() + sort_k);
    } else {
        // 全量排序
        std::sort(cand.begin(), cand.end());
    }

    // 记录更新时间步
    timestep_updated_[agent_idx] = curr_timestep_ + 1;
}

// ================================================================
// rebuild_chunk — 线程内批量重建一段连续的 agent
//
// 每个线程处理 order[begin..end) 范围内的 agent：
//   - order 中连续的 agent 索引 → candidates_[i] 在内存中也趋向连续
//   - 线程内的 task 遍历走同一份只读缓存 (task_start_locs_, task_lengths_)
//   - 写入 candidates_[i] 各线程独立，无竞争
//
// 返回实际重建的数量（遇到 deadline 会提前停止）
// ================================================================
int GreedyHeapMatcher::rebuild_chunk(
    const std::vector<int>& order, int begin, int end,
    std::chrono::steady_clock::time_point deadline)
{
    int count = 0;
    for (int idx = begin; idx < end; idx++) {
        if (std::chrono::steady_clock::now() > deadline) break;
        rebuild_single(order[idx]);
        count++;
    }
    return count;
}

// ================================================================
// prepare — 保存 agents/tasks 指针，调整缓存大小
// ================================================================
void GreedyHeapMatcher::prepare(
    const std::vector<AgentInfo>& agents,
    const std::vector<TaskInfo>& tasks)
{
    agents_ptr_ = &agents;
    tasks_ptr_  = &tasks;

    int n = static_cast<int>(agents.size());
    int m = static_cast<int>(tasks.size());

    // 如果 agents 数量变小了，截断缓存
    if (static_cast<int>(candidates_.size()) > n) {
        candidates_.resize(n);
        timestep_updated_.resize(n);
    } else {
        candidates_.resize(n);
        timestep_updated_.resize(n, 0);
    }

    // 关键：清除候选列表中 task index 已越界的旧缓存
    // （上一轮 tasks 可能更多，旧缓存中的 task_idx >= 当前 m 就无效了）
    for (int i = 0; i < n; i++) {
        if (!candidates_[i].empty()) {
            // 快速检查：如果最大的 task_idx >= m，整个列表需要重建
            // candidates_ 按 score 排序，task_idx 分散在各处，无法快速修补
            // 直接清除，强制重建
            bool has_invalid = false;
            for (auto& [s, tj] : candidates_[i]) {
                if (tj >= m) { has_invalid = true; break; }
            }
            if (has_invalid) {
                candidates_[i].clear();
                timestep_updated_[i] = 0;  // 标记为需要重建
            }
        }
    }

    // ---- 预计算 task_length 缓存（所有 agent 共享，每轮只算一次） ----
    // 原来在 rebuild_single 中每个 agent 对每个 task 都调用 compute_task_length，
    // 5000 agents × 7500 tasks = 37,500,000 次重复调用。
    // 现在只需计算 7500 次。
    task_lengths_.resize(m);
    task_start_locs_.resize(m);
    for (int j = 0; j < m; j++) {
        task_lengths_[j] = compute_task_length(tasks[j]);
        task_start_locs_[j] = tasks[j].start_location;
    }
    task_lengths_valid_ = true;

    // ---- 预计算 agent_locs_ 连续数组 ----
    // rebuild_single 中按 agent_idx 访问位置，提取到连续数组后
    // 线程内顺序访问 agent_locs_[i], agent_locs_[i+1], ... 具有良好的 cache 行为
    agent_locs_.resize(n);
    for (int i = 0; i < n; i++) {
        agent_locs_[i] = agents[i].location;
    }

    // ---- 缓存配置参数 ----
    cached_dist_weight_ = cfg().heap_dist_weight;
    cached_decay_interval_ = cfg().heap_dist_decay_interval_steps;
    cached_decay_factor_ = cfg().heap_dist_decay_factor;
    cached_decay_steps_ = 0;
    if (cached_decay_interval_ > 0 &&
        cached_decay_factor_ >= 0.0f &&
        curr_timestep_ >= cached_decay_interval_) {
        cached_dist_weight_ *= cached_decay_factor_;
        cached_decay_steps_ = 1;
    }
    // 如果 game_cong_weight > 0 或 game_dest_cong_weight > 0，则激活拥塞评分
    // 实际权重已在回调函数内部组合
    cached_cong_weight_ = cfg().game_cong_weight;
    if (cached_cong_weight_ <= 0.0f && cfg().game_dest_cong_weight > 0.0f) {
        cached_cong_weight_ = 1.0f;  // 激活拥塞路径，权重由回调内部处理
    }
    cached_dist_pow_    = cfg().game_dist_pow;
    cached_age_weight_  = cfg().game_age_weight;
    cached_comp_weight_ = cfg().game_comp_weight;
    if (total_steps_ <= 0) total_steps_ = cfg().game_total_steps;

    // ---- 预计算 task 起点拥塞缓存 ----
    task_congestion_.resize(m);
    if (congestion_func_) {
        for (int j = 0; j < m; j++) {
            task_congestion_[j] = congestion_func_(task_start_locs_[j]);
        }
    } else {
        std::fill(task_congestion_.begin(), task_congestion_.end(), 0.0f);
    }

    // ---- 预计算 task age 缓存 ----
    task_ages_.resize(m);
    for (int j = 0; j < m; j++) {
        int age = curr_timestep_ - tasks[j].t_revealed;
        task_ages_[j] = (age > 0) ? age : 0;
    }
}

// ================================================================
// rebuild_candidates — 重建候选列表
//
// get_h_fast() 始终线程安全（矩阵 O(1) 或曼哈顿距离），可以多线程并行 rebuild。
//
// 策略：优先重建 timestep_updated 最旧的 robot（最久没更新的优先）
// 时间到了就停止，未重建的 robot 使用旧缓存
// ================================================================
void GreedyHeapMatcher::rebuild_candidates(
    std::chrono::steady_clock::time_point deadline)
{
    const auto& agents = *agents_ptr_;
    int n = static_cast<int>(agents.size());
    if (n == 0) return;

    // 按 timestep_updated 升序排列（最旧的优先重建）
    std::vector<int> order(n);
    for (int i = 0; i < n; i++) order[i] = i;
    std::stable_sort(order.begin(), order.end(), [this](int a, int b) {
        return timestep_updated_[a] < timestep_updated_[b];
    });

    int rebuilt_count = 0;

    // get_h_fast 无论走矩阵查表还是曼哈顿距离，都是无状态纯计算，线程安全
    bool can_parallel = true;

    if (can_parallel) {
        // ---- 多线程并行 rebuild（连续块分配，cache-friendly） ----
        //
        // 改进：原来用 atomic fetch_add 逐个抢活，每个线程访问的 agent 在内存中
        // 是分散的（线程 A 处理 agent 0, 线程 B 处理 agent 1, ...），
        // candidates_[i] 的写入在线程间交错，cache line 冲突严重（false sharing）。
        //
        // 现在改为静态连续块分配：
        //   线程 0 → order[0..chunk), 线程 1 → order[chunk..2*chunk), ...
        // 每个线程的 candidates_ 写入集中在连续内存区域，
        // 且读取 task_start_locs_/task_lengths_ 共享只读缓存的 cache line 不冲突。
        //
        int n_threads = cfg().num_threads;
        if (n_threads <= 0) {
            n_threads = static_cast<int>(std::thread::hardware_concurrency());
            if (n_threads <= 0) n_threads = 4;
        }
        n_threads = std::min(n_threads, n);

        // 均匀分块
        int chunk_size = (n + n_threads - 1) / n_threads;
        std::vector<int> chunk_results(n_threads, 0);

        std::vector<std::thread> threads;
        threads.reserve(n_threads);
        for (int t = 0; t < n_threads; t++) {
            int begin = t * chunk_size;
            int end   = std::min(begin + chunk_size, n);
            if (begin >= n) break;

            threads.emplace_back([this, &order, &chunk_results, begin, end, t, deadline]() {
                chunk_results[t] = rebuild_chunk(order, begin, end, deadline);
            });
        }
        for (auto& th : threads) {
            th.join();
        }

        for (int t = 0; t < static_cast<int>(chunk_results.size()); t++) {
            rebuilt_count += chunk_results[t];
        }
    } else {
        // ---- 单线程执行 ----
        rebuilt_count = rebuild_chunk(order, 0, n, deadline);
    }

    if (cfg().debug_level >= 1) {
        int pct = n > 0 ? rebuilt_count * 100 / n : 100;
        int n_tasks = tasks_ptr_ ? static_cast<int>(tasks_ptr_->size()) : 0;
        int n_threads_used = can_parallel ?
            (cfg().num_threads > 0 ? cfg().num_threads
             : static_cast<int>(std::thread::hardware_concurrency())) : 1;
        int chunk = can_parallel ? (n + n_threads_used - 1) / n_threads_used : n;
        int sort_k = cfg().heap_sort_k;
        fprintf(stderr, "[optsvr] heap rebuild: %d/%d agents (%d%%), %d tasks, %s (%d threads, chunk=%d), "
                "sort_k=%d, dist_w=%.3f, decay_after=%d, decay_factor=%.3f, decayed=%d, tick=%d\n",
                rebuilt_count, n, pct, n_tasks,
                can_parallel ? "parallel" : "serial",
                n_threads_used, chunk, sort_k, cached_dist_weight_,
                cached_decay_interval_, cached_decay_factor_,
                cached_decay_steps_, curr_timestep_);
    }
}

// ================================================================
// lazy_heap_solve — 全局最小堆贪心匹配
//
// 算法：
//   1. 初始化堆：每个 robot 把候选列表的第一个 (score, robot_idx, 0) 入堆
//   2. 循环弹出堆顶 (score, robot_idx, cand_index)：
//      - 如果 task 已被占用 → cand_index++，push 下一个候选回堆
//      - 如果 task 可用 → 确认匹配，标记 task 已占用
//   3. 直到堆空或超时
//
// 这是一种 "lazy" 的全局贪心：不需要 N×M 代价矩阵，只需要每个 robot
// 的排序候选列表，堆保证全局按 score 升序处理。
// ================================================================
MatchResult GreedyHeapMatcher::lazy_heap_solve(
    std::chrono::steady_clock::time_point deadline, int max_matches)
{
    const auto& agents = *agents_ptr_;
    const auto& tasks  = *tasks_ptr_;
    int n = static_cast<int>(agents.size());
    int m = static_cast<int>(tasks.size());

    if (n == 0 || m == 0) return {};

    // max_matches <= 0 表示不限制
    int match_limit = (max_matches > 0) ? max_matches : std::min(n, m);

    // (score, agent_idx, candidate_index)
    using HeapItem = std::tuple<float, int, int>;
    std::priority_queue<HeapItem, std::vector<HeapItem>, std::greater<HeapItem>> heap;

    // 初始化：每个 robot 放入候选列表的第一个
    for (int i = 0; i < n; i++) {
        if (!candidates_[i].empty()) {
            heap.push({candidates_[i][0].first, i, 0});
        }
    }

    std::vector<bool> task_used(m, false);
    std::vector<bool> agent_matched(n, false);
    MatchResult result;
    result.reserve(std::min(match_limit, std::min(n, m)));

    int heap_pops = 0;       // 总 pop 次数
    int lazy_skips = 0;      // task 被占用导致的 skip 次数

    while (!heap.empty()) {
        // 达到匹配上限 → 提前停止（全局 score 最低的 top-K 已确认）
        if (static_cast<int>(result.size()) >= match_limit) break;
        if (std::chrono::steady_clock::now() > deadline) break;

        auto [score, ai, ci] = heap.top();
        heap.pop();
        heap_pops++;

        // robot 已经匹配了（不应发生，但防御性检查）
        if (agent_matched[ai]) continue;

        // 安全检查：ci 超出候选列表
        if (ci >= static_cast<int>(candidates_[ai].size())) continue;

        int ti = candidates_[ai][ci].second;

        // 防御性检查：task index 越界（旧缓存残留）→ 跳到下一个候选
        if (ti < 0 || ti >= m) {
            ci++;
            if (ci < static_cast<int>(candidates_[ai].size())) {
                heap.push({candidates_[ai][ci].first, ai, ci});
            }
            continue;
        }

        // task 已被占用 → lazy skip：推入下一个候选
        if (task_used[ti]) {
            lazy_skips++;
            ci++;
            if (ci < static_cast<int>(candidates_[ai].size())) {
                heap.push({candidates_[ai][ci].first, ai, ci});
            }
            continue;
        }

        // 匹配成功
        result.push_back({agents[ai].id, tasks[ti].id});
        task_used[ti] = true;
        agent_matched[ai] = true;
    }

    if (cfg().debug_level >= 1) {
        int pct = n > 0 ? static_cast<int>(result.size()) * 100 / n : 100;
        fprintf(stderr, "[optsvr] heap solve: matched %d/%d agents (%d%%), "
                "%d pops, %d lazy_skips (skip ratio %.1f%%), max_matches=%d\n",
                static_cast<int>(result.size()), n, pct,
                heap_pops, lazy_skips,
                heap_pops > 0 ? lazy_skips * 100.0 / heap_pops : 0.0,
                match_limit);
    }

    return result;
}

// ================================================================
// lns_refine — swap 优化
//
// 首轮（old_assignment 为空）：SA swap（概率接受差解，更好探索全局）
// 后续轮次：贪心 swap（只接受 delta < 0，含 bias 保护旧配对）
// keep_bias 根据地图规模折算：effective = base * (rows+cols) / 200
// ================================================================
MatchResult GreedyHeapMatcher::lns_refine(
    MatchResult initial,
    const std::vector<AgentInfo>& agents,
    const std::vector<TaskInfo>& tasks,
    std::chrono::steady_clock::time_point deadline,
    const std::unordered_map<int, int>& old_assignment)
{
    int sz = static_cast<int>(initial.size());
    if (sz < 2) return initial;

    // 构建 id → index 映射
    std::unordered_map<int, int> aid_to_ai;
    for (int i = 0; i < static_cast<int>(agents.size()); i++) {
        aid_to_ai[agents[i].id] = i;
    }
    std::unordered_map<int, int> tid_to_ti;
    for (int i = 0; i < static_cast<int>(tasks.size()); i++) {
        tid_to_ti[tasks[i].id] = i;
    }

    // keep_bias 根据地图规模折算（首轮 old_assignment 为空 → bias=0）
    float base_bias = old_assignment.empty() ? 0.0f : cfg().reassign_keep_bias;
    float scale_factor = 1.0f;
    // if (env_ && base_bias > 0.0f) {
    //     scale_factor = static_cast<float>(env_->rows + env_->cols) / 200.0f;
    //     if (scale_factor < 0.1f) scale_factor = 0.1f;
    // }
    const float keep_bias = base_bias * scale_factor;

    bool use_sa = old_assignment.empty();  // 首轮用 SA，后续轮次贪心

    // 打分函数：旧配对减去 keep_bias
    auto get_pair_score = [&](int agent_id, int task_id) -> float {
        auto ait = aid_to_ai.find(agent_id);
        auto tit = tid_to_ti.find(task_id);
        if (ait == aid_to_ai.end() || tit == tid_to_ti.end()) return 1e9f;
        float score = compute_score_fast(agents[ait->second].location, tit->second);
        if (keep_bias > 0.0f) {
            auto oit = old_assignment.find(agent_id);
            if (oit != old_assignment.end() && oit->second == task_id) {
                score -= keep_bias;
            }
        }
        return score;
    };

    std::mt19937 rng(42);
    std::uniform_int_distribution<int> dist(0, sz - 1);
    int swap_steps = 0, swap_improved = 0;

    // SA 参数（仅首轮使用）
    double total_score = 0;
    double temp = 0.5, decay = 0.999;
    auto sa_start = std::chrono::steady_clock::now();
    if (use_sa) {
        for (auto& [aid, tid] : initial)
            total_score += get_pair_score(aid, tid);
    }

    while (std::chrono::steady_clock::now() < deadline) {
        int i = dist(rng);
        int j = dist(rng);
        if (i == j) { swap_steps++; if (use_sa) temp *= decay; continue; }

        int aid_i = initial[i].first, aid_j = initial[j].first;
        int tid_i = initial[i].second, tid_j = initial[j].second;

        double delta = (get_pair_score(aid_i, tid_j) + get_pair_score(aid_j, tid_i))
                     - (get_pair_score(aid_i, tid_i) + get_pair_score(aid_j, tid_j));

        bool accept = (delta < 0);
        if (use_sa && !accept && delta > 0 && total_score > 0 && temp > 0.001) {
            accept = std::uniform_real_distribution<double>(0.0, 1.0)(rng)
                   < std::exp(-delta / (total_score * temp));
        }

        if (accept) {
            initial[i].second = tid_j;
            initial[j].second = tid_i;
            if (use_sa) total_score += delta;
            if (delta < 0) swap_improved++;
        }
        swap_steps++;

        if (use_sa) {
            // 100 步后自动校准衰减率
            if (swap_steps == 100) {
                auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                    std::chrono::steady_clock::now() - sa_start).count();
                auto budget_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                    deadline - sa_start).count();
                if (elapsed_ms > 0 && budget_ms > 0) {
                    double est_total = (swap_steps / static_cast<double>(elapsed_ms)) * budget_ms;
                    decay = std::pow(0.001, 1.0 / est_total);
                }
            }
            temp *= decay;
        }
    }

    if (cfg().debug_level >= 1) {
        fprintf(stderr, "[optsvr] heap LNS(%s): %d swap_steps (%d improved), "
                "keep_bias=%.1f (base=%.1f, scale=%.2f)\n",
                use_sa ? "SA" : "greedy",
                swap_steps, swap_improved,
                keep_bias, base_bias, scale_factor);
    }

    return initial;
}

// ================================================================
// match — 一站式入口：自动分配三阶段时间预算
//
// 时间预算分配（参考 LORR24 的 200ms + 500ms + 0ms）：
//   rebuild:    heap_rebuild_pct% 的总时间
//   lns_refine: heap_lns_pct% 的总时间（默认 0 = 关闭）
//   lazy_solve: 剩余时间
// ================================================================
MatchResult GreedyHeapMatcher::match(
    const std::vector<AgentInfo>& agents,
    const std::vector<TaskInfo>& tasks,
    std::chrono::steady_clock::time_point deadline,
    int n_free_agents,
    const std::unordered_map<int, int>& old_assignment,
    int max_matches)
{
    // n_free_agents < 0 → 所有 agents 都是 free（无 reassign）
    // n_free_agents >= 0 → agents[0..n_free_agents) 是 free，
    //   agents[n_free_agents..end) 是 reassign agents（不参与 rebuild/solve，只参与 LNS）
    //   对应地 tasks[0..n_free_tasks) 是 free tasks，tasks[n_free_tasks..end) 是 reassign tasks
    int total_agents = static_cast<int>(agents.size());
    int total_tasks  = static_cast<int>(tasks.size());
    bool has_reassign = (n_free_agents >= 0 && n_free_agents < total_agents);
    int n_fa = has_reassign ? n_free_agents : total_agents;
    // reassign agents 的 task 也追加在 tasks 尾部，数量相同
    int n_ft = has_reassign ? (total_tasks - (total_agents - n_free_agents)) : total_tasks;
    if (n_ft < 0) n_ft = total_tasks;  // safety

    if (n_fa == 0 && !has_reassign) return {};
    if (n_fa == 0 && total_tasks == 0) return {};

    auto now = std::chrono::steady_clock::now();
    auto total_budget = std::chrono::duration_cast<std::chrono::milliseconds>(deadline - now);
    if (total_budget.count() <= 0) return {};

    // 时间预算分配（均按百分比，LNS 最多用 lns_ms，剩余留给 planner）
    int total_ms = static_cast<int>(total_budget.count());
    int rebuild_ms, lns_ms, solve_ms;
    if (n_fa == 0) {
        // 无 free agents → rebuild/solve 无用，LNS swap 仍可用于 reassign agents
        rebuild_ms = 0;
        solve_ms = 0;
        lns_ms = total_ms * cfg().heap_lns_pct / 100;
    } else {
        rebuild_ms = total_ms * cfg().heap_rebuild_pct / 100;
        lns_ms     = total_ms * cfg().heap_lns_pct / 100;
        solve_ms   = total_ms - rebuild_ms - lns_ms;
    }

    auto rebuild_deadline = now + std::chrono::milliseconds(rebuild_ms);
    auto solve_deadline   = now + std::chrono::milliseconds(rebuild_ms + solve_ms);
    // LNS 在 Phase 2 结束后立即开始，最多用 lns_ms，不占用 planner 时间

    if (cfg().debug_level >= 1) {
        fprintf(stderr, "[optsvr] GreedyHeap match: %d free_agents, %d free_tasks"
                ", %d reassign_agents, budget %d ms "
                "(rebuild %d ms, solve %d ms, lns %d ms)\n",
                n_fa, n_ft,
                total_agents - n_fa,
                total_ms, rebuild_ms, solve_ms, lns_ms);
    }

    MatchResult result;

    // Phase 1+2 只在有 free agents 和 free tasks 时执行
    auto t1_start = std::chrono::steady_clock::now();
    auto t1_prepare = t1_start;
    auto t1_end = t1_start;
    auto t2_start = t1_start;
    auto t2_end = t1_start;

    if (n_fa > 0 && n_ft > 0) {
        // 构造 free-only 的子视图（用 span 思路，但为兼容性直接构造子 vector）
        // 注意：prepare/rebuild/solve 操作的是 agents_ptr_/tasks_ptr_，
        // 所以需要用 free 子集来调用
        std::vector<AgentInfo> free_agents_vec(agents.begin(), agents.begin() + n_fa);
        std::vector<TaskInfo>  free_tasks_vec(tasks.begin(), tasks.begin() + n_ft);

        // Phase 1: 准备 + 重建候选列表
        t1_start = std::chrono::steady_clock::now();
        prepare(free_agents_vec, free_tasks_vec);
        t1_prepare = std::chrono::steady_clock::now();
        rebuild_candidates(rebuild_deadline);
        t1_end = std::chrono::steady_clock::now();

        // Phase 2: Lazy heap 贪心匹配
        t2_start = std::chrono::steady_clock::now();
        result = lazy_heap_solve(solve_deadline, max_matches);
        t2_end = std::chrono::steady_clock::now();
    }

    // Phase 3: LNS swap 优化
    // 合并 reassign agents 的旧配对，然后对所有 agents 做 swap
    auto t3_start = std::chrono::steady_clock::now();
    // LNS 最多用 lns_ms，不超过总 deadline
    auto lns_deadline = std::min(t3_start + std::chrono::milliseconds(lns_ms), deadline);
    if (lns_ms > 0) {
        if (has_reassign) {
            // 合并 reassign agents 的旧配对到 result 中
            for (int i = n_fa; i < total_agents; i++) {
                auto oit = old_assignment.find(agents[i].id);
                if (oit != old_assignment.end()) {
                    result.push_back({agents[i].id, oit->second});
                }
            }
        }

        if (result.size() >= 2) {
            prepare(agents, tasks);
            result = lns_refine(std::move(result), agents, tasks, lns_deadline,
                                old_assignment);
        }
    } else if (has_reassign) {
        // LNS 关闭但有 reassign agents → 旧分配原样保留写回
        for (int i = n_fa; i < total_agents; i++) {
            auto oit = old_assignment.find(agents[i].id);
            if (oit != old_assignment.end()) {
                result.push_back({agents[i].id, oit->second});
            }
        }
    }
    auto t3_end = std::chrono::steady_clock::now();

    if (cfg().debug_level >= 1) {
        auto ms = [](std::chrono::steady_clock::time_point a,
                     std::chrono::steady_clock::time_point b) {
            return std::chrono::duration_cast<std::chrono::microseconds>(b - a).count() / 1000.0;
        };
        double prepare_t = ms(t1_start, t1_prepare);
        double rebuild_t = ms(t1_prepare, t1_end);
        double phase2 = ms(t2_start, t2_end);
        double phase3 = ms(t3_start, t3_end);
        double total  = ms(now, t3_end);
        fprintf(stderr, "[optsvr] GreedyHeap timing: prepare=%.1fms, rebuild=%.1fms, "
                "solve=%.1fms, lns=%.1fms, total=%.1fms (budget %dms)\n",
                prepare_t, rebuild_t, phase2, phase3, total, total_ms);
    }

    return result;
}

} // namespace orz_engine
