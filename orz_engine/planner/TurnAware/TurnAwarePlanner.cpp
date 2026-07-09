// optsvr/planner/TurnAware/TurnAwarePlanner.cpp
// 转弯感知版 Planner v2：
//   核心改动：
//   1. A* guide path 在 (loc, orient) 状态空间搜索，输出仅保留位置序列给 flow
//   2. PIBT 保持"位置模型"（与 FlowDecay 完全一致的 causalPIBT 架构）
//      但启发式换成 ta_heuristic_.get(loc, orient, goal) — 感知当前朝向
//   3. decided_ 和 moveCheck 与 FlowDecay 完全一致，无状态不同步问题

#include "TurnAwarePlanner.h"
#include "RuntimeConfig.h"
#include "../FlowDecay/const.h"
#include "../FlowDecay/flow.h"
#include "../FlowDecay/heuristics.h"
#include "../FlowDecay/pibt.h"
#include "../FlowDecay/utils.h"

#include <algorithm>
#include <numeric>
#include <chrono>
#include <iostream>
#include <cmath>
#include <cassert>
#include <cstring>
#include <climits>

#ifdef _OPENMP
#include <omp.h>
#endif

using namespace OptPlan;

namespace orz_engine {

static inline int unscale_flow(int raw) {
    int s = cfg().flow_decay_scale;
    if (s <= 1) return raw;
    return (raw + s / 2) / s;
}

// ═══════════════════════════════════════════════════════════════════════════════
// TurnAwareHeuristic: 预计算带转弯的精确距离
// ═══════════════════════════════════════════════════════════════════════════════

void TurnAwarePlanner::TurnAwareHeuristic::build(SharedEnvironment* env) {
    rows = env->rows;
    cols = env->cols;
    map_size = env->rows * env->cols;

    cell_to_idx.assign(map_size, -1);
    idx_to_cell.clear();
    for (int loc = 0; loc < map_size; ++loc) {
        if (env->map[loc] == 0) {
            cell_to_idx[loc] = static_cast<int>(idx_to_cell.size());
            idx_to_cell.push_back(loc);
        }
    }
    n_free = static_cast<int>(idx_to_cell.size());
    n_states = 4 * n_free;

    size_t bytes = static_cast<size_t>(n_free) * n_states * sizeof(uint16_t);
    if (bytes > 4ull * 1024 * 1024 * 1024) {
        std::cerr << "[TurnAware] TA heuristic too large (" << bytes / (1024*1024) << " MB), skip\n";
        ready = false;
        return;
    }

    matrix.assign(static_cast<size_t>(n_free) * n_states, 0xFFFF);

    std::cerr << "[TurnAware] Building turn-aware heuristic: n_free=" << n_free
              << " states=" << n_states << " mem=" << bytes / (1024*1024) << "MB\n";

    auto start = std::chrono::steady_clock::now();

    int moves[4] = {1, cols, -1, -cols};

#ifdef _OPENMP
    int n_threads = std::min(32, n_free);
#pragma omp parallel for num_threads(n_threads) schedule(dynamic, 1)
#endif
    for (int tidx = 0; tidx < n_free; ++tidx) {
        uint16_t* row = &matrix[static_cast<size_t>(tidx) * n_states];

        std::vector<int> q;
        q.reserve(n_states);

        // target 初始化：到达 target 后朝向任意，距离为 0
        for (int o = 0; o < 4; ++o) {
            int s = tidx * 4 + o;
            row[s] = 0;
            q.push_back(s);
        }

        for (size_t qi = 0; qi < q.size(); ++qi) {
            int cur_state = q[qi];
            int cur_idx = cur_state / 4;
            int cur_orient = cur_state % 4;
            int cur_loc = idx_to_cell[cur_idx];
            uint16_t cur_dist = row[cur_state];

            // 反向 FW: 前驱 (prev_loc, cur_orient)，前驱面向 cur_orient 执行 FW 到达 cur_loc
            int prev_loc = cur_loc - moves[cur_orient];
            if (prev_loc >= 0 && prev_loc < map_size && cell_to_idx[prev_loc] >= 0) {
                int pr = prev_loc / cols, pc = prev_loc % cols;
                int cr = cur_loc / cols, cc = cur_loc % cols;
                if (std::abs(pr - cr) + std::abs(pc - cc) == 1) {
                    int prev_idx = cell_to_idx[prev_loc];
                    int prev_state = prev_idx * 4 + cur_orient;
                    if (row[prev_state] > cur_dist + 1) {
                        row[prev_state] = cur_dist + 1;
                        q.push_back(prev_state);
                    }
                }
            }

            // 反向 CR: 前驱 (cur_loc, (cur_orient+3)%4) 执行 CR 变为 cur_orient
            {
                int prev_orient = (cur_orient + 3) % 4;
                int prev_state = cur_idx * 4 + prev_orient;
                if (row[prev_state] > cur_dist + 1) {
                    row[prev_state] = cur_dist + 1;
                    q.push_back(prev_state);
                }
            }

            // 反向 CCR: 前驱 (cur_loc, (cur_orient+1)%4) 执行 CCR 变为 cur_orient
            {
                int prev_orient = (cur_orient + 1) % 4;
                int prev_state = cur_idx * 4 + prev_orient;
                if (row[prev_state] > cur_dist + 1) {
                    row[prev_state] = cur_dist + 1;
                    q.push_back(prev_state);
                }
            }
        }
    }

    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - start).count();
    std::cerr << "[TurnAware] TA heuristic built in " << elapsed << "ms\n";
    ready = true;
}

int TurnAwarePlanner::TurnAwareHeuristic::get(int source, int source_orient, int target) const {
    if (!ready) return -1;
    int si = cell_to_idx[source];
    int ti = cell_to_idx[target];
    if (si < 0 || ti < 0) return 0xFFFF;
    int state = si * 4 + source_orient;
    return matrix[static_cast<size_t>(ti) * n_states + state];
}

int TurnAwarePlanner::TurnAwareHeuristic::get_min(int source, int target) const {
    if (!ready) return -1;
    int si = cell_to_idx[source];
    int ti = cell_to_idx[target];
    if (si < 0 || ti < 0) return 0xFFFF;
    int base_state = si * 4;
    size_t base = static_cast<size_t>(ti) * n_states;
    int mn = 0xFFFF;
    for (int o = 0; o < 4; ++o) {
        int d = matrix[base + base_state + o];
        if (d < mn) mn = d;
    }
    return mn;
}

// ═══════════════════════════════════════════════════════════════════════════════
// Turn-Aware A* Guide Path
// ═══════════════════════════════════════════════════════════════════════════════

bool TurnAwarePlanner::ta_astar(SharedEnvironment* env, int start_loc, int start_orient,
                                int goal_loc, std::vector<Int4>& flow, Traj& traj) {
    if (start_loc == goal_loc) {
        traj.clear();
        traj.push_back(start_loc);
        return true;
    }

    int map_size = env->rows * env->cols;
    int n_states = map_size * 4;
    int moves[4] = {1, env->cols, -1, -env->cols};

    struct ASNode {
        int state;
        int g;
        int h;
        int op_flow;
        int all_vertex_flow;
        int parent_state;
    };

    auto cmp = [](const ASNode& a, const ASNode& b) {
        int ca = a.op_flow + a.all_vertex_flow + a.g + a.h;
        int cb = b.op_flow + b.all_vertex_flow + b.g + b.h;
        if (ca != cb) return ca > cb;
        int fa = a.g + a.h, fb = b.g + b.h;
        if (fa != fb) return fa > fb;
        return a.g < b.g;
    };
    std::priority_queue<ASNode, std::vector<ASNode>, decltype(cmp)> open(cmp);

    std::vector<int> dist(n_states, INT_MAX);
    std::vector<int> parent(n_states, -1);
    std::vector<bool> closed(n_states, false);

    int start_state = start_loc * 4 + start_orient;
    int h0 = ta_heuristic_.ready ? ta_heuristic_.get(start_loc, start_orient, goal_loc) : 0;
    dist[start_state] = 0;
    open.push({start_state, 0, h0, 0, 0, -1});

    int goal_state = -1;

    while (!open.empty()) {
        auto cur = open.top(); open.pop();
        int cs = cur.state;
        if (closed[cs]) continue;
        closed[cs] = true;
        parent[cs] = cur.parent_state;

        int cur_loc = cs / 4;
        int cur_orient = cs % 4;

        if (cur_loc == goal_loc) {
            goal_state = cs;
            break;
        }

        // FW: move in facing direction
        int next_loc = cur_loc + moves[cur_orient];
        if (next_loc >= 0 && next_loc < map_size && env->map[next_loc] == 0) {
            int nr = next_loc / env->cols, nc = next_loc % env->cols;
            int cr = cur_loc / env->cols, cc = cur_loc % env->cols;
            if (std::abs(nr - cr) + std::abs(nc - cc) == 1) {
                int ns = next_loc * 4 + cur_orient;
                if (!closed[ns]) {
                    int ng = cur.g + 1;
                    if (ng < dist[ns]) {
                        dist[ns] = ng;
                        int nh = ta_heuristic_.ready ? ta_heuristic_.get(next_loc, cur_orient, goal_loc) : 0;
                        int d = get_d(next_loc - cur_loc, env);
                        int op = (unscale_flow(flow[cur_loc].d[d]) + 1) * unscale_flow(flow[next_loc].d[(d + 2) % 4]);
                        int vf = 1;
                        for (int j = 0; j < 4; j++) vf += unscale_flow(flow[next_loc].d[j]);
                        vf = (vf - 1) / 2;
                        open.push({ns, ng, nh, cur.op_flow + op, cur.all_vertex_flow + vf, cs});
                    }
                }
            }
        }

        // CR: turn clockwise, stay at same location
        {
            int no = (cur_orient + 1) % 4;
            int ns = cur_loc * 4 + no;
            if (!closed[ns]) {
                int ng = cur.g + 1;
                if (ng < dist[ns]) {
                    dist[ns] = ng;
                    int nh = ta_heuristic_.ready ? ta_heuristic_.get(cur_loc, no, goal_loc) : 0;
                    open.push({ns, ng, nh, cur.op_flow, cur.all_vertex_flow, cs});
                }
            }
        }

        // CCR: turn counter-clockwise, stay at same location
        {
            int no = (cur_orient + 3) % 4;
            int ns = cur_loc * 4 + no;
            if (!closed[ns]) {
                int ng = cur.g + 1;
                if (ng < dist[ns]) {
                    dist[ns] = ng;
                    int nh = ta_heuristic_.ready ? ta_heuristic_.get(cur_loc, no, goal_loc) : 0;
                    open.push({ns, ng, nh, cur.op_flow, cur.all_vertex_flow, cs});
                }
            }
        }
    }

    if (goal_state < 0) return false;

    // 回溯：保留位置序列（连续相同 loc 去重），给 flow 系统用
    std::vector<int> state_path;
    for (int s = goal_state; s != -1; s = parent[s]) {
        state_path.push_back(s);
    }
    std::reverse(state_path.begin(), state_path.end());

    traj.clear();
    for (int s : state_path) {
        int loc = s / 4;
        if (traj.empty() || traj.back() != loc) {
            traj.push_back(loc);
        }
    }
    return true;
}

// ═══════════════════════════════════════════════════════════════════════════════
// PIBT — 位置模型 + turn-aware 启发式
// 与 FlowDecay causalPIBT 结构完全一致，只替换启发式
// ═══════════════════════════════════════════════════════════════════════════════

int TurnAwarePlanner::ta_get_gp_h(TrajLNS& lns, int ai, int loc, int orient) {
    // 优先使用 guide path 距离表
    if (!lns.traj_dists.empty() && !lns.traj_dists[ai].empty()) {
        return get_dist_2_path(lns.traj_dists[ai], lns.env, loc, &neighbors_);
    }
    // 使用 turn-aware 精确距离（感知当前朝向）
    if (ta_heuristic_.ready) {
        return ta_heuristic_.get(loc, orient, lns.tasks.at(ai));
    }
    return manhattanDistance(loc, lns.tasks.at(ai), lns.env);
}

bool TurnAwarePlanner::ta_causalPIBT(int curr_id, int higher_id,
                                      std::vector<State>& prev_states,
                                      std::vector<State>& next_states,
                                      std::vector<int>& prev_decision,
                                      std::vector<int>& decision,
                                      std::vector<bool>& occupied,
                                      TrajLNS& lns) {
    // 与 FlowDecay::causalPIBT 完全一致的"位置模型"
    // 唯一区别：启发式使用 ta_get_gp_h 感知当前朝向

    assert(next_states[curr_id].location == -1);
    int prev_loc = prev_states[curr_id].location;
    int prev_orient = prev_states[curr_id].orientation;

    assert(prev_loc >= 0 && prev_loc < (int)env_->map.size());

    if (higher_id == -1) {
        occupied[prev_loc] = true;
    }

    // 生成邻居位置 + 等待（与 FlowDecay 完全一致的位置候选集）
    std::vector<int> neighbor_locs;
    getNeighborLocs(&neighbors_, neighbor_locs, prev_loc);

    // 朝向前方的位置（用于 tie-break）
    int moves[4] = {1, env_->cols, -1, -env_->cols};
    int orien_next_v = prev_loc + moves[prev_orient];

    struct Candidate {
        int location;
        int heuristic;
        int tie_breaker;
    };
    std::vector<Candidate> successors;
    successors.reserve(neighbor_locs.size() + 1);

    for (int nb : neighbor_locs) {
        assert(validateMove(prev_loc, nb, env_));
        // 计算如果移动到 nb，agent 的新朝向
        int diff = nb - prev_loc;
        int move_orient;
        if (diff == 1) move_orient = 0;
        else if (diff == env_->cols) move_orient = 1;
        else if (diff == -1) move_orient = 2;
        else move_orient = 3; // diff == -env_->cols

        // 启发式：从 nb 面向 move_orient（到达 nb 后面朝的方向）到 goal 的距离
        int h = ta_get_gp_h(lns, curr_id, nb, move_orient);
        successors.push_back({nb, h, rand()});
    }

    // Wait: 留在原地，朝向不变
    {
        int h = ta_get_gp_h(lns, curr_id, prev_loc, prev_orient);
        successors.push_back({prev_loc, h, rand()});
    }

    // 排序：h 小优先，tie-break 偏好面朝方向，最后随机
    std::sort(successors.begin(), successors.end(),
        [&](const Candidate& a, const Candidate& b) {
            if (a.heuristic != b.heuristic) return a.heuristic < b.heuristic;
            // tie-break: prefer moving forward (面朝方向)
            if (a.location == orien_next_v && b.location != orien_next_v) return true;
            if (a.location != orien_next_v && b.location == orien_next_v) return false;
            return a.tie_breaker < b.tie_breaker;
        });

    for (auto& succ : successors) {
        if (occupied[succ.location] && !(higher_id == -1 && prev_loc == succ.location))
            continue;
        if (succ.location == -1) continue;
        if (decision[succ.location] != -1) continue;
        if (higher_id != -1 && prev_decision[succ.location] == higher_id) continue;

        next_states.at(curr_id) = State(succ.location, -1, -1);
        decision.at(succ.location) = curr_id;

        // push-chain: 如果目标格子上有上一轮的 agent 且还未决策
        if (prev_decision.at(succ.location) != -1 &&
            next_states.at(prev_decision.at(succ.location)).location == -1) {
            int lower_id = prev_decision.at(succ.location);
            if (!ta_causalPIBT(lower_id, curr_id, prev_states, next_states,
                               prev_decision, decision, occupied, lns)) {
                next_states.at(curr_id) = State(-1, -1, -1);
                continue;
            }
        }
        return true;
    }

    // 无法移动，原地等待
    next_states.at(curr_id) = State(prev_loc, -1, -1);
    decision.at(prev_loc) = curr_id;
    return false;
}

// ═══════════════════════════════════════════════════════════════════════════════
// frank_wolfe (turn-aware 版)
// ═══════════════════════════════════════════════════════════════════════════════

void TurnAwarePlanner::ta_frank_wolfe(TrajLNS& lns, TimePoint timelimit) {
    update_fw_metrics(lns);
    std::vector<FW_Metric> replan_order = lns.fw_metrics;

    std::sort(replan_order.begin(), replan_order.end(),
        [](FW_Metric& a, FW_Metric& b) {
            if (a.deviation != b.deviation) return a.deviation > b.deviation;
            if (a.last_replan_t != b.last_replan_t) return a.last_replan_t < b.last_replan_t;
            return a.rand > b.rand;
        });

    int count = 0;
    while (std::chrono::steady_clock::now() < timelimit) {
        int index = count % lns.env->num_of_agents;
        int a = replan_order[index].id;
        count++;
        if (lns.traj_dists[a].empty() || lns.trajs[a].empty()) continue;

        std::vector<int> old_traj = lns.trajs[a];
        remove_traj(lns, a);

        int start = lns.env->curr_states[a].location;
        int orient = lns.env->curr_states[a].orientation;
        int goal = lns.tasks[a];

        Traj new_traj;
        if (ta_astar(lns.env, start, orient, goal, lns.flow, new_traj) && !new_traj.empty()) {
            lns.trajs[a] = new_traj;
            add_traj(lns, a);
            update_dist_2_path(lns, a);
        } else {
            lns.trajs[a] = old_traj;
            add_traj(lns, a);
            break;
        }
    }
}

// ═══════════════════════════════════════════════════════════════════════════════
// rollout_next_state
// ═══════════════════════════════════════════════════════════════════════════════

State TurnAwarePlanner::rollout_next_state(const State& state, Action action, const SharedEnvironment* env) {
    State next = state;
    next.timestep = state.timestep + 1;
    if (action == Action::CR) { next.orientation = (state.orientation + 1) % 4; return next; }
    if (action == Action::CCR) { next.orientation = (state.orientation + 3) % 4; return next; }
    if (action != Action::FW) return next;

    int delta = 0;
    if (state.orientation == 0) delta = 1;
    if (state.orientation == 1) delta = env->cols;
    if (state.orientation == 2) delta = -1;
    if (state.orientation == 3) delta = -env->cols;
    int next_loc = state.location + delta;
    if (next_loc >= 0 && next_loc < (int)env->map.size() && validateMove(state.location, next_loc, env))
        next.location = next_loc;
    return next;
}

// ═══════════════════════════════════════════════════════════════════════════════
// initialize
// ═══════════════════════════════════════════════════════════════════════════════

void TurnAwarePlanner::initialize(int preprocess_time_limit, SharedEnvironment* env) {
    env_ = env;
    assert(env->num_of_agents != 0);

    // 构建邻居表
    neighbors_.resize(env->map.size());
    for (int row = 0; row < env->rows; row++) {
        for (int col = 0; col < env->cols; col++) {
            int loc = row * env->cols + col;
            if (env->map[loc] != 0) continue;
            if (row > 0 && env->map[loc - env->cols] == 0) neighbors_[loc].push_back(loc - env->cols);
            if (row < env->rows - 1 && env->map[loc + env->cols] == 0) neighbors_[loc].push_back(loc + env->cols);
            if (col > 0 && env->map[loc - 1] == 0) neighbors_[loc].push_back(loc - 1);
            if (col < env->cols - 1 && env->map[loc + 1] == 0) neighbors_[loc].push_back(loc + 1);
        }
    }

    // 预计算 turn-aware 启发式
    ta_heuristic_.build(env);

    // 初始化 FlowDecay 的全局启发式
    init_heuristics(env);

    // 分配状态（与 OptPlanner 完全一致）
    p_.resize(env->num_of_agents);
    decision_.resize(env->map.size(), -1);
    prev_states_.resize(env->num_of_agents);
    next_states_.resize(env->num_of_agents);
    decided_.resize(env->num_of_agents, DCR({-1, DONE::DONE}));
    occupied_.resize(env->map.size(), false);
    checked_.resize(env->num_of_agents, false);
    ids_.resize(env->num_of_agents);
    require_guide_path_.resize(env->num_of_agents, false);

    for (int i = 0; i < (int)ids_.size(); i++) ids_[i] = i;

    mt1_.seed(0);
    srand(0);

    new (&trajLNS_) TrajLNS(env, global_heuristictable, neighbors_);
    trajLNS_.init_mem();

    std::shuffle(ids_.begin(), ids_.end(), mt1_);
    for (int i = 0; i < (int)ids_.size(); i++) {
        p_[ids_[i]] = ((double)(ids_.size() - i)) / ((double)(ids_.size() + 1));
    }
    p_copy_ = p_;
    initialized_ = true;

    std::cerr << "[TurnAware] initialized (v2: location-model PIBT + TA heuristic)\n";
}

// ═══════════════════════════════════════════════════════════════════════════════
// plan 主入口
// ═══════════════════════════════════════════════════════════════════════════════

void TurnAwarePlanner::initialize_dummy_goals(SharedEnvironment* env) {
    if (env->curr_timestep != 0) return;
    dummy_goals_.resize(env->num_of_agents);
    for (int i = 0; i < env->num_of_agents; i++) {
        dummy_goals_[i] = env->curr_states[i].location;
    }
}

void TurnAwarePlanner::setup_episode(SharedEnvironment* env, TimePoint flow_end,
                                      std::vector<double>& local_priority) {
    prev_decision_.clear();
    prev_decision_.resize(env->map.size(), -1);

    for (int i = 0; i < env->num_of_agents; i++) {
        if (env->goal_locations[i].empty()) {
            trajLNS_.tasks[i] = dummy_goals_.at(i);
            local_priority[i] = p_copy_[i];
        } else {
            trajLNS_.tasks[i] = env->goal_locations[i].front().first;
        }

        require_guide_path_[i] = false;
        if (trajLNS_.trajs[i].empty()) require_guide_path_[i] = true;
        else if (trajLNS_.trajs[i].back() != trajLNS_.tasks[i]) require_guide_path_[i] = true;

        assert(env->curr_states[i].location >= 0);
        prev_states_[i] = env->curr_states[i];
        next_states_[i] = State();
        prev_decision_[env->curr_states[i].location] = i;

        if (decided_[i].loc == -1) {
            decided_[i].loc = env->curr_states[i].location;
            assert(decided_[i].state == DONE::DONE);
        }
        if (prev_states_[i].location == decided_[i].loc) decided_[i].state = DONE::DONE;
        if (decided_[i].state == DONE::NOT_DONE) {
            decision_.at(decided_[i].loc) = i;
            next_states_[i] = State(decided_[i].loc, -1, -1);
        }

        if (require_guide_path_[i]) local_priority[i] = p_copy_[i];
        else if (!env->goal_locations[i].empty()) local_priority[i] += 1;

        if (!env->goal_locations[i].empty() &&
            neighbors_[env->curr_states[i].location].size() == 1)
            local_priority[i] += 10;
    }
}

void TurnAwarePlanner::update_guide_paths(SharedEnvironment* env, TimePoint flow_end) {
    for (int i = 0; i < env->num_of_agents; i++) {
        if (std::chrono::steady_clock::now() > flow_end) break;
        if (!require_guide_path_[i]) continue;

        if (!trajLNS_.trajs[i].empty()) remove_traj(trajLNS_, i);

        int start = env->curr_states[i].location;
        int orient = env->curr_states[i].orientation;
        int goal = trajLNS_.tasks[i];

        Traj new_traj;
        if (ta_astar(env, start, orient, goal, trajLNS_.flow, new_traj) && !new_traj.empty()) {
            trajLNS_.trajs[i] = new_traj;
            add_traj(trajLNS_, i);
            update_dist_2_path(trajLNS_, i);
        }
    }

    ta_frank_wolfe(trajLNS_, flow_end);
}

void TurnAwarePlanner::refresh_step(SharedEnvironment* env, std::vector<double>& local_priority) {
    prev_decision_.clear();
    prev_decision_.resize(env->map.size(), -1);

    for (int i = 0; i < env->num_of_agents; i++) {
        if (env->goal_locations[i].empty()) {
            trajLNS_.tasks[i] = dummy_goals_.at(i);
            local_priority[i] = p_copy_[i];
        } else {
            trajLNS_.tasks[i] = env->goal_locations[i].front().first;
            local_priority[i] += 1;
        }

        assert(env->curr_states[i].location >= 0);
        prev_states_[i] = env->curr_states[i];
        next_states_[i] = State();
        prev_decision_[env->curr_states[i].location] = i;

        if (prev_states_[i].location == decided_[i].loc) decided_[i].state = DONE::DONE;
        if (decided_[i].state == DONE::NOT_DONE) {
            decision_.at(decided_[i].loc) = i;
            next_states_[i] = State(decided_[i].loc, -1, -1);
        }

        if (!env->goal_locations[i].empty() &&
            neighbors_[env->curr_states[i].location].size() == 1)
            local_priority[i] += 10;
    }
}

void TurnAwarePlanner::run_pibt_step(SharedEnvironment* env, std::vector<double>& local_priority,
                                      std::vector<Action>& one_step_actions) {
    std::sort(ids_.begin(), ids_.end(), [&](int a, int b) {
        return local_priority.at(a) > local_priority.at(b);
    });

    std::fill(occupied_.begin(), occupied_.end(), false);

    for (int i : ids_) {
        if (decided_[i].state == DONE::NOT_DONE) continue;
        if (next_states_[i].location == -1) {
            assert(prev_states_[i].location >= 0 && prev_states_[i].location < (int)env->map.size());
            ta_causalPIBT(i, -1, prev_states_, next_states_,
                          prev_decision_, decision_, occupied_, trajLNS_);
        }
    }

    // 与 FlowDecay 完全一致的 action 生成：getAction(prev, decided_.loc, env)
    one_step_actions.resize(env->num_of_agents);
    for (int id : ids_) {
        if (next_states_.at(id).location != -1)
            decision_.at(next_states_.at(id).location) = -1;
        if (next_states_.at(id).location >= 0)
            decided_.at(id) = DCR({next_states_.at(id).location, DONE::NOT_DONE});

        // 使用 FlowDecay 的 getAction: 根据 prev 朝向和目标位置决定动作
        one_step_actions.at(id) = OptPlan::getAction(prev_states_.at(id), decided_.at(id).loc, env_);
        checked_.at(id) = false;
    }

    // moveCheck: 与 FlowDecay 完全一致
    for (int id = 0; id < env->num_of_agents; id++) {
        if (!checked_.at(id) && one_step_actions.at(id) == Action::FW)
            OptPlan::moveCheck(id, checked_, decided_, one_step_actions, prev_decision_);
    }

    prev_states_ = next_states_;
}

void TurnAwarePlanner::append_actions(SharedEnvironment* env,
                                       std::vector<std::vector<Action>>& actions,
                                       const std::vector<Action>& one_step_actions) {
    for (int aid = 0; aid < env->num_of_agents; aid++)
        actions[aid].push_back(one_step_actions[aid]);

    std::vector<State> rolled(env->num_of_agents);
    for (int aid = 0; aid < env->num_of_agents; aid++)
        rolled[aid] = rollout_next_state(env->curr_states[aid], one_step_actions[aid], env);
    env->curr_states = rolled;
    env->curr_timestep += 1;
}

void TurnAwarePlanner::plan(int time_limit, std::vector<std::vector<Action>>& actions,
                             SharedEnvironment* env, int num_steps) {
    actions.clear();
    if (!env || env->num_of_agents <= 0) return;
    actions.resize(env->num_of_agents);
    if (num_steps <= 0) return;

    env_ = env;
    const auto episode_start = std::chrono::steady_clock::now();

    int pibt_time = PIBT_RUNTIME_PER_100_AGENTS * env->num_of_agents / 100;
    if (pibt_time <= 0) pibt_time = 1;
    int flow_budget = std::max(0, time_limit - pibt_time * num_steps - TRAFFIC_FLOW_ASSIGNMENT_END_TIME_TOLERANCE);
    TimePoint flow_end = episode_start + std::chrono::milliseconds(flow_budget);
    TimePoint hard_deadline = episode_start + std::chrono::milliseconds(std::max(0, time_limit - 5));

    std::vector<double> local_priority = p_;
    const std::vector<State> original_states = env->curr_states;
    const int original_timestep = env->curr_timestep;
    const std::vector<DCR> original_decided = decided_;

    decided_.assign(env->num_of_agents, DCR({-1, DONE::DONE}));

    initialize_dummy_goals(env);
    setup_episode(env, flow_end, local_priority);
    update_guide_paths(env, flow_end);
    p_ = local_priority;

    for (int step = 0; step < num_steps; step++) {
        if (step > 0 && std::chrono::steady_clock::now() >= hard_deadline) break;

        std::vector<Action> one_step_actions;
        if (step > 0) refresh_step(env, local_priority);
        run_pibt_step(env, local_priority, one_step_actions);
        append_actions(env, actions, one_step_actions);
    }

    env->curr_states = original_states;
    env->curr_timestep = original_timestep;
    decided_ = original_decided;
}

} // namespace orz_engine
