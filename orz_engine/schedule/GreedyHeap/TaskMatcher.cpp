// optsvr/TaskMatcher.cpp — 无状态任务匹配器实现
#include "TaskMatcher.h"
#include "HeuristicMatrix.h"
#include <algorithm>
#include <cmath>
#include <cassert>
#include <iostream>

namespace orz_engine {

// ================================================================
// score_dist_only — mode 0 打分：score = dist_to_start
//   只看 agent 到任务起点的距离，不考虑任务自身长度
// ================================================================
float TaskMatcher::score_dist_only(const AgentInfo& agent, const TaskInfo& task) const {
    assert(env_ != nullptr);

    int map_size = static_cast<int>(env_->map.size());

    if (agent.location < 0 || agent.location >= map_size) return 1e9f;
    if (task.start_location < 0 || task.start_location >= map_size) return 1e9f;

    constexpr int H_CAP = 100000;
    int dist_to_start = get_h_fast(env_, agent.location, task.start_location);
    if (dist_to_start >= H_CAP) dist_to_start = H_CAP;

    return static_cast<float>(dist_to_start);
}

// ================================================================
// score — mode 1 打分：score = dist_to_start + task_length
// ================================================================
float TaskMatcher::score(const AgentInfo& agent, const TaskInfo& task) const {
    assert(env_ != nullptr);

    int map_size = static_cast<int>(env_->map.size());

    // 位置合法性检查
    if (agent.location < 0 || agent.location >= map_size) {
        std::cerr << "[optsvr] BUG: agent " << agent.id << " location=" << agent.location
                  << " out of range [0, " << map_size << ")\n";
        return 1e9f;
    }
    if (task.start_location < 0 || task.start_location >= map_size) {
        std::cerr << "[optsvr] BUG: task " << task.id << " start_location=" << task.start_location
                  << " out of range [0, " << map_size << ")\n";
        return 1e9f;
    }

    // (1) agent 到任务起点的距离（clamp 不可达值，防止溢出）
    constexpr int H_CAP = 100000;  // 大于任何合理地图距离，远小于 MAX_TIMESTEP
    int dist_to_start = get_h_fast(env_, agent.location, task.start_location);
    if (dist_to_start >= H_CAP) dist_to_start = H_CAP;

    // (2) 任务自身长度：所有 errand 依次走完的距离
    int task_length = 0;
    if (task.errand_locations.size() > 1) {
        int prev = task.errand_locations[0];
        for (size_t i = 1; i < task.errand_locations.size(); i++) {
            int next = task.errand_locations[i];
            int h = get_h_fast(env_, prev, next);
            if (h >= H_CAP) h = H_CAP;
            task_length += h;
            prev = next;
        }
    }

    // 综合打分：makespan = dist_to_start + task_length
    float s = static_cast<float>(dist_to_start + task_length);

    return s;
}

// ================================================================
// score_weighted — mode 3 打分：
//   score = dist_to_start + difficulty_weight * (task_length + congestion)
//   之前带权重和拥塞惩罚的方案
// ================================================================
float TaskMatcher::score_weighted(const AgentInfo& agent, const TaskInfo& task) const {
    assert(env_ != nullptr);

    int map_size = static_cast<int>(env_->map.size());

    if (agent.location < 0 || agent.location >= map_size) return 1e9f;
    if (task.start_location < 0 || task.start_location >= map_size) return 1e9f;

    constexpr int H_CAP = 100000;

    // (1) agent 到任务起点的距离
    int dist_to_start = get_h_fast(env_, agent.location, task.start_location);
    if (dist_to_start >= H_CAP) dist_to_start = H_CAP;

    // (2) 任务自身长度
    int task_length = 0;
    if (task.errand_locations.size() > 1) {
        int prev = task.errand_locations[0];
        for (size_t i = 1; i < task.errand_locations.size(); i++) {
            int next = task.errand_locations[i];
            int h = get_h_fast(env_, prev, next);
            if (h >= H_CAP) h = H_CAP;
            task_length += h;
            prev = next;
        }
    }

    // (3) 拥塞惩罚
    float congestion = 0.0f;
    if (congestion_func_) {
        congestion = congestion_func_(task.start_location);
    }

    // 综合打分：dist_to_start + difficulty_weight * (task_length + congestion)
    float dw = cfg().difficulty_weight;
    float s = static_cast<float>(dist_to_start) + dw * (static_cast<float>(task_length) + congestion);

    return s;
}

// ================================================================
// score_mode5 — mode 5 打分：score = 5 * dist_to_start + task_length
//   与 GreedyHeap (mode 4) 相同的加权公式，但走匈牙利/Regret/TopK 路径
// ================================================================
float TaskMatcher::score_mode5(const AgentInfo& agent, const TaskInfo& task) const {
    assert(env_ != nullptr);

    int map_size = static_cast<int>(env_->map.size());

    if (agent.location < 0 || agent.location >= map_size) return 1e9f;
    if (task.start_location < 0 || task.start_location >= map_size) return 1e9f;

    constexpr int H_CAP = 100000;

    // (1) agent 到任务起点的距离
    int dist_to_start = get_h_fast(env_, agent.location, task.start_location);
    if (dist_to_start >= H_CAP) dist_to_start = H_CAP;

    // (2) 任务自身长度
    int task_length = 0;
    if (task.errand_locations.size() > 1) {
        int prev = task.errand_locations[0];
        for (size_t i = 1; i < task.errand_locations.size(); i++) {
            int next = task.errand_locations[i];
            int h = get_h_fast(env_, prev, next);
            if (h >= H_CAP) h = H_CAP;
            task_length += h;
            prev = next;
        }
    }

    // 综合打分：5 * dist + task_length
    float s = 5.0f * static_cast<float>(dist_to_start) + static_cast<float>(task_length);

    return s;
}

// ================================================================
// score_mode6 — mode 6 打分：score = heap_dist_weight * dist + task_length
//   与 mode5 相比，dist 的系数从 OPT_HEAP_DIST_WEIGHT 读取（可通过环境变量调节）
// ================================================================
float TaskMatcher::score_mode6(const AgentInfo& agent, const TaskInfo& task) const {
    assert(env_ != nullptr);

    int map_size = static_cast<int>(env_->map.size());

    if (agent.location < 0 || agent.location >= map_size) return 1e9f;
    if (task.start_location < 0 || task.start_location >= map_size) return 1e9f;

    constexpr int H_CAP = 100000;

    int dist_to_start = get_h_fast(env_, agent.location, task.start_location);
    if (dist_to_start >= H_CAP) dist_to_start = H_CAP;

    int task_length = 0;
    if (task.errand_locations.size() > 1) {
        int prev = task.errand_locations[0];
        for (size_t i = 1; i < task.errand_locations.size(); i++) {
            int next = task.errand_locations[i];
            int h = get_h_fast(env_, prev, next);
            if (h >= H_CAP) h = H_CAP;
            task_length += h;
            prev = next;
        }
    }

    float dw = cfg().heap_dist_weight;
    float s = dw * static_cast<float>(dist_to_start) + static_cast<float>(task_length);
    return s;
}

// ================================================================
// score_mode7 — mode 7 打分：score = 10 * sqrt(dist) + task_length
//   凹函数：对远距离的惩罚增长减缓，让"中远距 agent"抢任务的能力更均衡。
//   动机：maze 场景 dist_to_start 分布长尾，线性项让最近 agent 垄断；
//        sqrt 把长尾压缩，避免近处 agent 被同一组任务绑死。
// ================================================================
float TaskMatcher::score_mode7(const AgentInfo& agent, const TaskInfo& task) const {
    assert(env_ != nullptr);

    int map_size = static_cast<int>(env_->map.size());

    if (agent.location < 0 || agent.location >= map_size) return 1e9f;
    if (task.start_location < 0 || task.start_location >= map_size) return 1e9f;

    constexpr int H_CAP = 100000;

    int dist_to_start = get_h_fast(env_, agent.location, task.start_location);
    if (dist_to_start >= H_CAP) dist_to_start = H_CAP;

    int task_length = 0;
    if (task.errand_locations.size() > 1) {
        int prev = task.errand_locations[0];
        for (size_t i = 1; i < task.errand_locations.size(); i++) {
            int next = task.errand_locations[i];
            int h = get_h_fast(env_, prev, next);
            if (h >= H_CAP) h = H_CAP;
            task_length += h;
            prev = next;
        }
    }

    float s = 10.0f * std::sqrt(static_cast<float>(dist_to_start))
            + static_cast<float>(task_length);
    return s;
}

// ================================================================
// score_mode8 — mode 8 打分：heap_dist_weight * dist + task_length
// ================================================================
float TaskMatcher::score_mode8(const AgentInfo& agent, const TaskInfo& task) const {
    assert(env_ != nullptr);

    int map_size = static_cast<int>(env_->map.size());

    if (agent.location < 0 || agent.location >= map_size) return 1e9f;
    if (task.start_location < 0 || task.start_location >= map_size) return 1e9f;

    constexpr int H_CAP = 100000;

    int dist_to_start = get_h_fast(env_, agent.location, task.start_location);
    if (dist_to_start >= H_CAP) dist_to_start = H_CAP;

    int task_length = 0;
    if (task.errand_locations.size() > 1) {
        int prev = task.errand_locations[0];
        for (size_t i = 1; i < task.errand_locations.size(); i++) {
            int next = task.errand_locations[i];
            int h = get_h_fast(env_, prev, next);
            if (h >= H_CAP) h = H_CAP;
            task_length += h;
            prev = next;
        }
    }

    float dw = cfg().heap_dist_weight;
    return dw * static_cast<float>(dist_to_start)
         + static_cast<float>(task_length);
}

// ================================================================
// score_mode2 — 新思路：距离比例评分
//
// 思路：任务是持续完成的，应优先完成附近的任务。
// 给每个任务一个系数 = len / avg_len（len = task_length，avg_len = 当前 free tasks 的平均长度）
// 这样短任务的系数 < 1（更优先），长任务的系数 > 1（惩罚）
//
// 后期策略：当剩余 tick <= 2 * avg_len 时，直接用 dist_to_start + task_length
// 因为此时短任务更有可能在截止前完成，应该大力惩罚长任务
// ================================================================
float TaskMatcher::score_mode2(const AgentInfo& agent, const TaskInfo& task) const {
    assert(env_ != nullptr);

    int map_size = static_cast<int>(env_->map.size());

    // 位置合法性检查
    if (agent.location < 0 || agent.location >= map_size) return 1e9f;
    if (task.start_location < 0 || task.start_location >= map_size) return 1e9f;

    constexpr int H_CAP = 100000;

    // (1) agent 到任务起点的距离
    int dist_to_start = get_h_fast(env_, agent.location, task.start_location);
    if (dist_to_start >= H_CAP) dist_to_start = H_CAP;

    // (2) 任务自身长度
    int task_length = 0;
    if (task.errand_locations.size() > 1) {
        int prev = task.errand_locations[0];
        for (size_t i = 1; i < task.errand_locations.size(); i++) {
            int next = task.errand_locations[i];
            int h = get_h_fast(env_, prev, next);
            if (h >= H_CAP) h = H_CAP;
            task_length += h;
            prev = next;
        }
    }

    // 总完成距离 = dist_to_start + task_length（即 makespan）
    int total_len = dist_to_start + task_length;

    // 计算剩余 tick
    int remaining_ticks = 1000 - curr_timestep_;  // TOTAL_STEPS=1000
    if (remaining_ticks < 1) remaining_ticks = 1;

    float avg_len = avg_task_length_;
    if (avg_len < 1.0f) avg_len = 1.0f;

    // 后期判断：剩余 tick <= 2 * avg_len → 直接用 makespan（大力惩罚长任务）
    bool is_late_stage = (static_cast<float>(remaining_ticks) <= 4.0f * avg_len);

    if (is_late_stage) {
        // 后期：直接用 makespan = dist_to_start + task_length
        return static_cast<float>(total_len);
    } else {
        // 正常期：dist_to_start + (len / avg_len) * task_length
        // 其中 len = total_len（agent 到起点 + 任务本身长度），系数 = len / avg_len
        // 这样短任务的 difficulty 系数 < 1，优先被选；长任务系数 > 1，被惩罚
        //float ratio = static_cast<float>(total_len) / avg_len;
        //float s = static_cast<float>(dist_to_start) + ratio * static_cast<float>(task_length);
        float s = static_cast<float>(dist_to_start);
        return s;
    }
}

// ================================================================
// match（内置打分版本）— 根据 score_mode 选择打分函数，全部走匈牙利匹配
//
// score_mode = 0: 匈牙利 + score = dist_to_start（只看距离）
// score_mode = 1: 匈牙利 + score = dist_to_start + task_length
// score_mode = 2: 匈牙利 + 距离比例评分（新思路）
// score_mode = 3: 匈牙利 + dist + difficulty_weight * (task_len + congestion)
// score_mode = 5: 匈牙利 + 5*dist + task_length
// score_mode = 6: 匈牙利 + heap_dist_weight * dist + task_length（dw 可调）
// score_mode = 7: 匈牙利 + 10 * sqrt(dist) + task_length（凹函数）
// score_mode = 8: 匈牙利 + heap_dist_weight * dist + task_length + density_w * congestion
// ================================================================
MatchResult TaskMatcher::match(
    const std::vector<AgentInfo>& agents,
    const std::vector<TaskInfo>& tasks,
    std::chrono::steady_clock::time_point deadline) {

    int mode = cfg().score_mode;

    // 根据 mode 构造对应的打分函数，全部走匈牙利
    ScoreFunc score_func;
    if (mode == 0) {
        score_func = [this, &agents, &tasks](int ai, int ti) -> float {
            return this->score_dist_only(agents[ai], tasks[ti]);
        };
    } else if (mode == 1) {
        score_func = [this, &agents, &tasks](int ai, int ti) -> float {
            return this->score(agents[ai], tasks[ti]);
        };    
    } else if (mode == 2) {
        score_func = [this, &agents, &tasks](int ai, int ti) -> float {
            return this->score_mode2(agents[ai], tasks[ti]);
        };
    } else if (mode == 3) {
        score_func = [this, &agents, &tasks](int ai, int ti) -> float {
            return this->score_weighted(agents[ai], tasks[ti]);
        };
    } else if (mode == 5) {
        score_func = [this, &agents, &tasks](int ai, int ti) -> float {
            return this->score_mode5(agents[ai], tasks[ti]);
        };
    } else if (mode == 6) {
        score_func = [this, &agents, &tasks](int ai, int ti) -> float {
            return this->score_mode6(agents[ai], tasks[ti]);
        };
    } else if (mode == 7) {
        score_func = [this, &agents, &tasks](int ai, int ti) -> float {
            return this->score_mode7(agents[ai], tasks[ti]);
        };
    } else if (mode == 8) {
        score_func = [this, &agents, &tasks](int ai, int ti) -> float {
            return this->score_mode8(agents[ai], tasks[ti]);
        };
    } else {
        // mode 0 (default)
        score_func = [this, &agents, &tasks](int ai, int ti) -> float {
            return this->score_dist_only(agents[ai], tasks[ti]);
        };
    }

    return match(agents, tasks, score_func, deadline);
}

// ================================================================
// match（自定义打分版本）— 根据规模自动选择策略
// ================================================================
MatchResult TaskMatcher::match(
    const std::vector<AgentInfo>& agents,
    const std::vector<TaskInfo>& tasks,
    const ScoreFunc& score_func,
    std::chrono::steady_clock::time_point deadline) {

    if (agents.empty() || tasks.empty()) {
        return {};
    }

    int n = static_cast<int>(agents.size());
    int m = static_cast<int>(tasks.size());

    // 策略选择：根据规模选择匹配算法
    // 代价矩阵大小 = n * m * sizeof(float)，超过阈值时用 greedy_topk 避免 OOM/超时
    constexpr long long MAX_MATRIX_ELEMENTS = 2000000LL; // ~8MB，安全阈值
    long long matrix_size = static_cast<long long>(n) * m;

    if (matrix_size > MAX_MATRIX_ELEMENTS) {
        // 大规模：用 TopK 贪心匹配（O(n * m * log(topk))），不分配巨型矩阵
        if (cfg().debug_level >= 1) {
            std::cerr << "[optsvr] match: n=" << n << " m=" << m
                      << " matrix=" << matrix_size << " > " << MAX_MATRIX_ELEMENTS
                      << ", using greedy_topk\n";
        }
        return greedy_topk_match(agents, tasks, score_func, deadline);
    }

    // 小规模用 KM 匈牙利；超时 fallback 到 regret
    return hungarian_match(agents, tasks, score_func, deadline);
}

// ================================================================
// hungarian_match — KM 算法（Kuhn-Munkres）求最小权完美匹配
//
// 求解 N agents → M tasks 的最小代价二部匹配（N <= M）
// 时间复杂度 O(N^2 * M)，适合 N <= 500 的小规模场景
//
// 使用"shortest augmenting path"变体：
//   - 代价矩阵 cost[i][j] = score(agent_i, task_j)，越低越好（最小化）
//   - 维护 u[i]（agent 势）、v[j]（task 势），满足 u[i]+v[j] <= cost[i][j]
//   - 对每个 agent 用 Dijkstra 风格找最短增广路
//   - 沿增广路翻转匹配并更新势
//
// 参考：https://cp-algorithms.com/graph/hungarian-algorithm.html
// ================================================================
MatchResult TaskMatcher::hungarian_match(
    const std::vector<AgentInfo>& agents,
    const std::vector<TaskInfo>& tasks,
    const ScoreFunc& score_func,
    std::chrono::steady_clock::time_point deadline) {

    int n = static_cast<int>(agents.size());
    int m = static_cast<int>(tasks.size());

    // KM 要求 n <= m
    if (n > m) {
        return regret_match(agents, tasks, score_func, deadline);
    }

    // ---- Step 1: 构建代价矩阵（一维，行优先） ----
    // cost_matrix[i * m + j] = score(agent_i, task_j)，最小化
    constexpr float INF = 1e18f;
    std::vector<float> cost_matrix(n * m);

    for (int i = 0; i < n; i++) {
        if (std::chrono::steady_clock::now() > deadline) {
            return regret_match(agents, tasks, score_func, deadline);
        }
        for (int j = 0; j < m; j++) {
            cost_matrix[i * m + j] = score_func(i, j);
        }
    }

    // ---- Step 2: KM（最小化版本） ----
    // u[i] = agent 势（i=0..n-1），v[j] = task 势（j=0..m-1）
    // 不变量：u[i] + v[j] <= cost[i][j]（对所有 i,j）
    // 匹配边上等号成立：u[i] + v[matched_j] == cost[i][matched_j]
    std::vector<float> u(n + 1, 0.0f), v(m + 1, 0.0f);

    // p[j] = task j 匹配的 agent（0 表示未匹配，使用 1-indexed agent）
    // 我们用 0 号 agent 作为虚拟起点
    std::vector<int> p(m + 1, 0);   // p[j] = matched agent (1-indexed), 0 = free
    std::vector<int> way(m + 1, 0); // way[j] = 增广路中 task j 的前驱 task

    // 辅助数组
    std::vector<float> dist(m + 1);    // shortest reduced cost to each task
    std::vector<bool> used(m + 1);     // task visited in current Dijkstra

    for (int i = 1; i <= n; i++) {
        if (std::chrono::steady_clock::now() > deadline) {
            if (cfg().debug_level >= 1) {
                std::cerr << "[optsvr] hungarian: timeout at agent " << i << "/" << n
                          << ", fallback to regret\n";
            }
            return regret_match(agents, tasks, score_func, deadline);
        }

        // 将 agent i 虚拟分配到 task 0（哨兵）
        p[0] = i;
        int j0 = 0; // 当前从 task 0（虚拟）开始

        std::fill(dist.begin(), dist.end(), INF);
        std::fill(used.begin(), used.end(), false);

        // Dijkstra 风格找最短增广路
        bool augment_ok = true;
        do {
            used[j0] = true;
            int a0 = p[j0]; // a0 是 task j0 当前匹配的 agent（1-indexed）
            float delta = INF;
            int j1 = -1;

            for (int j = 1; j <= m; j++) {
                if (used[j]) continue;
                // reduced cost: cost[a0-1][j-1] - u[a0] - v[j]
                float cur = cost_matrix[(a0 - 1) * m + (j - 1)] - u[a0] - v[j];
                if (cur < dist[j]) {
                    dist[j] = cur;
                    way[j] = j0;
                }
                if (dist[j] < delta) {
                    delta = dist[j];
                    j1 = j;
                }
            }

            // 安全检查：如果没找到未访问的 task（所有列都被 used 了），
            // 说明出现异常（例如 cost 全为 INF），跳出避免 p[-1] 越界
            if (j1 < 0) {
                if (cfg().debug_level >= 1) {
                    std::cerr << "[optsvr] hungarian: j1=-1 at agent " << i
                              << ", breaking (possible unreachable tasks)\n";
                }
                augment_ok = false;
                break;
            }

            // 更新势
            for (int j = 0; j <= m; j++) {
                if (used[j]) {
                    u[p[j]] += delta;
                    v[j] -= delta;
                } else {
                    dist[j] -= delta;
                }
            }

            j0 = j1;
        } while (p[j0] != 0); // 直到找到一个空闲 task

        // 沿增广路翻转匹配（仅在成功找到增广路时执行）
        if (augment_ok) {
            do {
                int j_prev = way[j0];
                p[j0] = p[j_prev];
                j0 = j_prev;
            } while (j0 != 0);
        }
    }

    // ---- Step 3: 提取结果 ----
    MatchResult result;
    result.reserve(n);
    for (int j = 1; j <= m; j++) {
        if (p[j] > 0 && p[j] <= n) {
            int ai = p[j] - 1;  // 转回 0-indexed
            int ti = j - 1;
            result.push_back({agents[ai].id, tasks[ti].id});
        }
    }

    return result;
}

// ================================================================
// regret_match — Regret-based 小规模精确匹配
// ================================================================
MatchResult TaskMatcher::regret_match(
    const std::vector<AgentInfo>& agents,
    const std::vector<TaskInfo>& tasks,
    const ScoreFunc& score_func,
    std::chrono::steady_clock::time_point deadline) {

    int n = static_cast<int>(agents.size());
    int m = static_cast<int>(tasks.size());

    // 构建代价矩阵 cost[i][j] = score_func(i, j)
    std::vector<std::vector<float>> cost(n, std::vector<float>(m, 1e9f));

    for (int i = 0; i < n; i++) {
        if (std::chrono::steady_clock::now() > deadline) break;
        for (int j = 0; j < m; j++) {
            cost[i][j] = score_func(i, j);
        }
    }

    // Regret-based 分配：按 regret 降序优先分配
    struct AgentBest {
        int agent_idx;
        int best_task_idx;
        float best_score;
        float second_best_score;
        float regret;
    };

    std::vector<AgentBest> agent_bests(n);
    for (int i = 0; i < n; i++) {
        float best = 1e9f, second = 1e9f;
        int best_j = 0;
        for (int j = 0; j < m; j++) {
            if (cost[i][j] < best) {
                second = best;
                best = cost[i][j];
                best_j = j;
            } else if (cost[i][j] < second) {
                second = cost[i][j];
            }
        }
        agent_bests[i] = {i, best_j, best, second, second - best};
    }

    std::sort(agent_bests.begin(), agent_bests.end(),
              [](const AgentBest& a, const AgentBest& b) {
                  return a.regret > b.regret;
              });

    std::vector<bool> task_used(m, false);
    std::vector<int> assignment(n, -1);

    for (auto& ab : agent_bests) {
        if (std::chrono::steady_clock::now() > deadline) break;

        int i = ab.agent_idx;
        if (!task_used[ab.best_task_idx]) {
            assignment[i] = ab.best_task_idx;
            task_used[ab.best_task_idx] = true;
        } else {
            float best = 1e9f;
            int best_j = -1;
            for (int j = 0; j < m; j++) {
                if (!task_used[j] && cost[i][j] < best) {
                    best = cost[i][j];
                    best_j = j;
                }
            }
            if (best_j >= 0) {
                assignment[i] = best_j;
                task_used[best_j] = true;
            }
        }
    }

    // 构建结果
    MatchResult result;
    result.reserve(n);
    for (int i = 0; i < n; i++) {
        if (assignment[i] >= 0) {
            result.push_back({agents[i].id, tasks[assignment[i]].id});
        }
    }
    return result;
}

// ================================================================
// greedy_topk_match — 大规模 TopK 候选贪心匹配
// ================================================================
MatchResult TaskMatcher::greedy_topk_match(
    const std::vector<AgentInfo>& agents,
    const std::vector<TaskInfo>& tasks,
    const ScoreFunc& score_func,
    std::chrono::steady_clock::time_point deadline) {

    assert(env_ != nullptr);

    int n = static_cast<int>(agents.size());
    int m = static_cast<int>(tasks.size());
    int topk = std::min(cfg().candidate_topk, m);
    int cols = env_->cols;

    // 预计算 task 起点位置的行列
    struct TaskLoc {
        int row, col;
    };
    std::vector<TaskLoc> task_locs(m);
    for (int j = 0; j < m; j++) {
        int loc = tasks[j].start_location;
        task_locs[j] = {loc / cols, loc % cols};
    }

    // 按每个 agent 到最近 task 的距离排序（远的优先选）
    struct AgentOrder {
        int agent_idx;
        float min_dist;
    };
    std::vector<AgentOrder> agent_order;
    agent_order.reserve(n);

    for (int i = 0; i < n; i++) {
        if (std::chrono::steady_clock::now() > deadline) break;
        int loc = agents[i].location;
        int a_row = loc / cols;
        int a_col = loc % cols;

        float min_dist = 1e9f;
        for (int j = 0; j < std::min(m, topk * 2); j++) {
            float dist = std::abs(a_row - task_locs[j].row)
                       + std::abs(a_col - task_locs[j].col);
            min_dist = std::min(min_dist, dist);
        }
        agent_order.push_back({i, min_dist});
    }

    // 距离远的 agent 优先（选择更受限）
    std::sort(agent_order.begin(), agent_order.end(),
              [](const AgentOrder& a, const AgentOrder& b) {
                  return a.min_dist > b.min_dist;
              });

    // 依次为每个 agent 从候选中选最优 task
    std::vector<bool> task_used(m, false);
    MatchResult result;
    result.reserve(n);

    for (auto& ao : agent_order) {
        if (std::chrono::steady_clock::now() > deadline) break;

        int i = ao.agent_idx;
        int loc = agents[i].location;
        int a_row = loc / cols;
        int a_col = loc % cols;

        // 快速曼哈顿距离粗筛 TopK
        std::vector<std::pair<float, int>> candidates;
        for (int j = 0; j < m; j++) {
            if (task_used[j]) continue;
            float dist = std::abs(a_row - task_locs[j].row)
                       + std::abs(a_col - task_locs[j].col);
            candidates.push_back({dist, j});
        }

        int k = std::min(topk, static_cast<int>(candidates.size()));
        if (k == 0) continue;

        std::partial_sort(candidates.begin(), candidates.begin() + k, candidates.end());

        // 对 topk 候选用精确打分
        float best_score = 1e9f;
        int best_j = -1;

        for (int c = 0; c < k; c++) {
            if (std::chrono::steady_clock::now() > deadline) break;
            int j = candidates[c].second;
            float s = score_func(i, j);
            if (s < best_score) {
                best_score = s;
                best_j = j;
            }
        }

        if (best_j >= 0) {
            result.push_back({agents[i].id, tasks[best_j].id});
            task_used[best_j] = true;
        }
    }

    return result;
}

} // namespace orz_engine
