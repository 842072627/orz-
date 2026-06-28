// optsvr/planner/EPIBT/EPIBTPlanner.cpp
// 来源: IDEA/GameOpt/MAPFPlanner.cpp
// EPIBT: 在 causalPIBT 排序中加入 turn_cost，优先选不需要转弯的方向
// 结构与 OptPlanner/TurnAwarePlanner 一致，完全独立的状态

#include "EPIBTPlanner.h"
#include "RuntimeConfig.h"
#include "MapProfile.h"
#include "../FlowDecay/const.h"
#include "../FlowDecay/flow.h"
#include "../FlowDecay/FlowVertexCost.h"
#include "../FlowDecay/heuristics.h"
#include "../FlowDecay/pibt.h"
#include "../FlowDecay/utils.h"

#include <algorithm>
#include <chrono>
#include <cassert>
#include <cstring>
#include <climits>
#include <cstdlib>
#include <iostream>
#include <thread>
#include <utility>

using namespace OptPlan;

namespace optsvr {

namespace {

struct ParallelGuideResult {
    int agent = -1;
    bool success = false;
    Traj traj;
    s_node goal_node;
};

} // namespace

// ═══════════════════════════════════════════════════════════════════════════════
// EPIBT Core: turn-cost aware PIBT
// ═══════════════════════════════════════════════════════════════════════════════

bool EPIBTPlanner::epibt_causal(int curr_id, int higher_id) {
    assert(next_states_[curr_id].location == -1);
    int prev_loc = prev_states_[curr_id].location;
    int prev_ori = prev_states_[curr_id].orientation;
    int target = trajLNS_.tasks.at(curr_id);

    assert(prev_loc >= 0 && prev_loc < (int)env_->map.size());

    if (higher_id == -1) {
        occupied_[prev_loc] = true;
    }

    // Generate successors
    std::vector<int> neighbors;
    getNeighborLocs(&neighbors_, neighbors, prev_loc);

    struct EPIBT_C {
        int location;
        double effective_h;
        bool is_forward;
        int tie_breaker;
    };

    std::vector<EPIBT_C> successors;
    successors.reserve(neighbors.size() + 1);
    int current_h = get_gp_h(trajLNS_, curr_id, prev_loc);

    for (auto& nb : neighbors) {
        assert(validateMove(prev_loc, nb, env_));
        int base_h = get_gp_h(trajLNS_, curr_id, nb);
        int move_ori = get_move_orientation(prev_loc, nb, env_->cols);
        double t_cost = (move_ori != -1) ? turn_cost(prev_ori, move_ori) * turn_cost_mult_ : 0.0;
        bool is_forward = (move_ori == prev_ori);
        double score = static_cast<double>(base_h) + t_cost + flow_vertex_adjustment(nb);
        successors.push_back({nb, score, is_forward, rand()});
    }

    // Wait option
    {
        double score = static_cast<double>(current_h) + static_cast<double>(wait_penalty_) +
            flow_vertex_adjustment(prev_loc);
        successors.push_back({prev_loc, score, false, rand()});
    }

    // Sort: effective_h ascending, tie-break forward, then random
    std::sort(successors.begin(), successors.end(),
        [](const EPIBT_C& a, const EPIBT_C& b) {
            if (a.effective_h != b.effective_h) return a.effective_h < b.effective_h;
            if (a.is_forward && !b.is_forward) return true;
            if (!a.is_forward && b.is_forward) return false;
            return a.tie_breaker < b.tie_breaker;
        });

    // Try each successor
    for (auto& next : successors) {
        if (occupied_[next.location] && !(higher_id == -1 && prev_loc == next.location))
            continue;
        if (next.location == -1) continue;
        if (decision_[next.location] != -1) continue;
        if (higher_id != -1 && prev_decision_[next.location] == higher_id) continue;

        next_states_.at(curr_id) = State(next.location, -1, -1);
        decision_.at(next.location) = curr_id;

        if (prev_decision_.at(next.location) != -1 &&
            next_states_.at(prev_decision_.at(next.location)).location == -1) {
            int lower_id = prev_decision_.at(next.location);
            if (!epibt_causal(lower_id, curr_id)) {
                next_states_.at(curr_id) = State(-1, -1, -1);
                continue;
            }
        }
        return true;
    }

    // All failed: stay
    next_states_.at(curr_id) = State(prev_loc, -1, -1);
    decision_.at(prev_loc) = curr_id;
    return false;
}

// ═══════════════════════════════════════════════════════════════════════════════
// rollout_next_state
// ═══════════════════════════════════════════════════════════════════════════════

State EPIBTPlanner::rollout_next_state(const State& state, Action action, const SharedEnvironment* env) {
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

double EPIBTPlanner::flow_vertex_adjustment(int loc) {
    if (flow_vertex_params_.weight <= 0.0 && flow_vertex_params_.base_weight <= 0) return 0.0;
    if (loc < 0 || loc >= static_cast<int>(trajLNS_.flow.size())) return 0.0;

    const int temp_vertex = vertex_flow_count_at(trajLNS_.flow, loc);
    return vertex_flow_cost_double(temp_vertex, 0, flow_vertex_decay_table_, flow_vertex_params_);
}

// ═══════════════════════════════════════════════════════════════════════════════
// initialize
// ═══════════════════════════════════════════════════════════════════════════════

void EPIBTPlanner::initialize(int preprocess_time_limit, SharedEnvironment* env) {
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

    // 初始化 FlowDecay 全局启发式（如尚未初始化）
    init_heuristics(env);

    // 分配状态
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

    // 加载环境变量可调参数
    if (const char* v = std::getenv("OPT_EPIBT_TURN_MULT"))
        turn_cost_mult_ = std::max(0.0, std::atof(v));
    if (const char* v = std::getenv("OPT_EPIBT_WAIT_PENALTY"))
        wait_penalty_ = std::max(0, std::atoi(v));

    flow_vertex_params_.mode = std::max(0, cfg().epibt_flow_vertex_mode);
    flow_vertex_params_.threshold = std::max(0, cfg().epibt_flow_vertex_threshold);
    flow_vertex_params_.weight = std::max(0.0, static_cast<double>(cfg().epibt_flow_vertex_weight));
    flow_vertex_params_.cap = std::max(0, cfg().epibt_flow_vertex_cap);
    flow_vertex_params_.exp_base = std::max(0.0f, cfg().epibt_flow_vertex_exp_base);
    flow_vertex_params_.linear_end = std::max(0, cfg().epibt_flow_vertex_linear_end);
    flow_vertex_params_.sens_decay = 1.0f;
    flow_vertex_params_.base_weight = std::max(0, cfg().epibt_flow_vertex_base_weight);
    flow_vertex_decay_table_.assign(1, 1.0);

    if (cfg().planner_mode == 10 &&
        cfg().ware_direct &&
        map_profile().type == MapType::WAREHOUSE_DOMAIN) {
        build_row_column_highway(env);
    }

    std::cerr << "[EPIBT] initialized (turn_mult=" << turn_cost_mult_
              << " wait_pen=" << wait_penalty_
              << " flow_vertex_mode=" << flow_vertex_params_.mode
              << " flow_vertex_threshold=" << flow_vertex_params_.threshold
              << " flow_vertex_w=" << flow_vertex_params_.weight
              << " flow_vertex_exp_base=" << flow_vertex_params_.exp_base
              << " flow_vertex_linear_end=" << flow_vertex_params_.linear_end
              << " flow_vertex_base_w=" << flow_vertex_params_.base_weight
              << ")\n";
}

void EPIBTPlanner::build_row_column_highway(SharedEnvironment* env) {
    if (env == nullptr || env->rows <= 2 || env->cols <= 2) {
        return;
    }

    static constexpr int ROW_PENALTY = 250;
    static constexpr int COL_PENALTY = 150;

    extra_weights_.assign(env->rows * env->cols, std::vector<int>(4, 0));

    std::vector<std::vector<int>> grid(env->rows, std::vector<int>(env->cols, 1));
    for (int row = 0; row < env->rows; row++) {
        for (int col = 0; col < env->cols; col++) {
            grid[row][col] = env->map[row * env->cols + col];
        }
    }

    std::vector<int> row_free_count(env->rows, 0);
    for (int row = 0; row < env->rows; row++) {
        for (int col = 0; col < env->cols; col++) {
            if (grid[row][col] == 0) row_free_count[row]++;
        }
    }

    const int max_row_free = *std::max_element(row_free_count.begin(), row_free_count.end());
    const int row_threshold = max_row_free * 80 / 100;

    std::vector<int> passage_rows;
    for (int row = 1; row < env->rows - 1; row++) {
        if (row_free_count[row] > row_threshold &&
            row_free_count[row - 1] <= row_threshold &&
            row_free_count[row + 1] <= row_threshold) {
            passage_rows.emplace_back(row);
        }
    }

    for (int i = 0; i < static_cast<int>(passage_rows.size()); i++) {
        const int row = passage_rows[i];
        for (int col = 1; col < env->cols - 1; col++) {
            const int loc = row * env->cols + col;
            if (i % 2 == 0) {
                extra_weights_[loc][2] = ROW_PENALTY;  // discourage west, bias east
            } else {
                extra_weights_[loc][0] = ROW_PENALTY;  // discourage east, bias west
            }
        }
    }

    std::vector<int> col_free_count(env->cols, 0);
    for (int col = 0; col < env->cols; col++) {
        for (int row = 0; row < env->rows; row++) {
            if (grid[row][col] == 0) col_free_count[col]++;
        }
    }

    const int max_col_free = *std::max_element(col_free_count.begin(), col_free_count.end());
    const int col_threshold = max_col_free * 80 / 100;

    std::vector<int> passage_columns;
    for (int col = 1; col < env->cols - 1; col++) {
        if (col_free_count[col] > col_threshold &&
            col_free_count[col - 1] <= col_threshold &&
            col_free_count[col + 1] <= col_threshold) {
            passage_columns.emplace_back(col);
        }
    }

    for (int i = 0; i < static_cast<int>(passage_columns.size()); i++) {
        const int col = passage_columns[i];
        for (int row = 1; row < env->rows - 1; row++) {
            const int loc = row * env->cols + col;
            if (i % 2 == 0) {
                extra_weights_[loc][3] = COL_PENALTY;  // discourage north, bias south
            } else {
                extra_weights_[loc][1] = COL_PENALTY;  // discourage south, bias north
            }
        }
    }

    if (cfg().debug_level >= 1) {
        std::cerr << "[EPIBT] WARE_DIRECT highway (P10): passage_rows="
                  << passage_rows.size() << ", passage_columns="
                  << passage_columns.size()
                  << ", row_penalty=" << ROW_PENALTY
                  << ", col_penalty=" << COL_PENALTY << "\n";
    }
}

void EPIBTPlanner::update_guide_paths_for_plan(SharedEnvironment* env, TimePoint flow_end,
                                               int& guide_required, int& guide_completed,
                                               int& guide_failed, bool& guide_stopped_by_time) {
    guide_required = 0;
    for (bool need_update : require_guide_path_) {
        if (need_update) guide_required++;
    }
    guide_completed = 0;
    guide_failed = 0;
    guide_stopped_by_time = false;

    if (guide_required <= 0) return;

    if (cfg().planner_mode == 10 && cfg().epibt_parallel_guide) {
        update_guide_paths_parallel_rounds(env, flow_end, guide_completed, guide_failed,
                                           guide_stopped_by_time);
    } else {
        update_guide_paths_serial(env, flow_end, guide_completed, guide_failed,
                                  guide_stopped_by_time);
    }
}

void EPIBTPlanner::update_guide_paths_serial(SharedEnvironment* env, TimePoint flow_end,
                                             int& guide_completed, int& guide_failed,
                                             bool& guide_stopped_by_time) {
    for (int i = 0; i < env->num_of_agents; i++) {
        if (std::chrono::steady_clock::now() > flow_end) {
            guide_stopped_by_time = true;
            break;
        }
        if (!require_guide_path_[i]) continue;
        std::vector<int> old_traj = trajLNS_.trajs[i];
        if (!trajLNS_.trajs[i].empty()) remove_traj(trajLNS_, i);
        if (!update_traj(trajLNS_, i, &flow_end, guide_extra_weights())) {
            trajLNS_.trajs[i] = old_traj;
            if (!old_traj.empty()) add_traj(trajLNS_, i);
            guide_failed++;
            break;
        }
        guide_completed++;
    }
}

int EPIBTPlanner::run_parallel_guide_round(SharedEnvironment* env, TimePoint flow_end,
                                           const std::vector<int>& round_agents,
                                           int max_threads, int& guide_failed,
                                           bool& guide_stopped_by_time,
                                           std::unordered_set<int>* updated_agents) {
    const int count = static_cast<int>(round_agents.size());
    if (count <= 0) return 0;

    for (int agent : round_agents) {
        const int goal = trajLNS_.tasks[agent];
        if (trajLNS_.heuristics.find(goal) == trajLNS_.heuristics.end()) {
            trajLNS_.heuristics.emplace(goal, HeuristicTable{});
        }
    }

    std::vector<HeuristicTable*> heuristics(count, nullptr);
    for (int j = 0; j < count; j++) {
        heuristics[j] = &trajLNS_.heuristics.find(trajLNS_.tasks[round_agents[j]])->second;
    }

    std::vector<ParallelGuideResult> results(count);
    const int n_threads = std::max(1, std::min(max_threads, count));
    const auto* extra_weights = guide_extra_weights();
    std::vector<std::thread> threads;
    threads.reserve(n_threads);

    for (int worker = 0; worker < n_threads; worker++) {
        threads.emplace_back([&, worker]() {
            MemoryPool mem;
            mem.init(static_cast<int>(env->map.size()));
            for (int j = worker; j < count; j += n_threads) {
                const int agent = round_agents[j];
                ParallelGuideResult result;
                result.agent = agent;

                Traj candidate;
                s_node goal_node = astar(env, trajLNS_.flow, *heuristics[j],
                                         candidate, mem,
                                         env->curr_states[agent].location,
                                         trajLNS_.tasks[agent],
                                         &neighbors_, &flow_end, extra_weights,
                                         env->curr_states[agent].orientation);
                if (goal_node.id != -1) {
                    result.success = true;
                    result.traj = std::move(candidate);
                    result.goal_node = goal_node;
                }
                results[j] = std::move(result);
            }
        });
    }
    for (auto& thread : threads) thread.join();

    int completed = 0;
    int round_failed = 0;
    for (const auto& result : results) {
        if (!result.success) {
            round_failed++;
            continue;
        }

        const int agent = result.agent;
        // Commit only after the whole parallel round has finished searching.
        if (!trajLNS_.trajs[agent].empty()) remove_traj(trajLNS_, agent);
        trajLNS_.trajs[agent] = result.traj;
        trajLNS_.goal_nodes[agent] = result.goal_node;
        add_traj(trajLNS_, agent);
        update_dist_2_path(trajLNS_, agent);
        if (updated_agents != nullptr) updated_agents->insert(agent);
        completed++;
    }

    if (round_failed > 0) {
        guide_failed += round_failed;
        if (std::chrono::steady_clock::now() >= flow_end) {
            guide_stopped_by_time = true;
        }
    }
    return completed;
}

void EPIBTPlanner::update_guide_paths_parallel_rounds(SharedEnvironment* env, TimePoint flow_end,
                                                      int& guide_completed, int& guide_failed,
                                                      bool& guide_stopped_by_time) {
    int max_threads = cfg().num_threads;
    if (max_threads <= 0) {
        max_threads = static_cast<int>(std::thread::hardware_concurrency());
        if (max_threads > 4) max_threads = 4;
    }
    if (max_threads <= 0) max_threads = 1;
    const int parallel_agents = max_threads;

    std::vector<int> pending_agents;
    pending_agents.reserve(env->num_of_agents);
    for (int i = 0; i < env->num_of_agents; i++) {
        if (require_guide_path_[i]) pending_agents.push_back(i);
    }

    for (int offset = 0; offset < static_cast<int>(pending_agents.size()); offset += parallel_agents) {
        if (std::chrono::steady_clock::now() > flow_end) {
            guide_stopped_by_time = true;
            break;
        }

        const int end = std::min(static_cast<int>(pending_agents.size()), offset + parallel_agents);
        std::vector<int> round_agents;
        round_agents.reserve(end - offset);
        for (int j = offset; j < end; j++) round_agents.push_back(pending_agents[j]);

        guide_completed += run_parallel_guide_round(env, flow_end, round_agents, max_threads,
                                                    guide_failed, guide_stopped_by_time);
        if (guide_failed > 0) break;
    }
}

void EPIBTPlanner::improve_guide_paths_parallel_rounds(SharedEnvironment* env, TimePoint flow_end,
                                                       int& improve_completed, int& improve_failed,
                                                       bool& improve_stopped_by_time,
                                                       int* unique_improved_agents) {
    improve_completed = 0;
    improve_failed = 0;
    improve_stopped_by_time = false;

    int max_threads = cfg().num_threads;
    if (max_threads <= 0) {
        max_threads = static_cast<int>(std::thread::hardware_concurrency());
        if (max_threads > 4) max_threads = 4;
    }
    if (max_threads <= 0) max_threads = 1;
    const int parallel_agents = max_threads;

    update_fw_metrics(trajLNS_);
    std::vector<FW_Metric> replan_order = trajLNS_.fw_metrics;
    std::sort(replan_order.begin(), replan_order.end(),
        [](const FW_Metric& a, const FW_Metric& b) {
            if (a.deviation != b.deviation) return a.deviation > b.deviation;
            if (a.last_replan_t != b.last_replan_t) return a.last_replan_t < b.last_replan_t;
            return a.rand > b.rand;
        });

    int cursor = 0;
    int total_completed = 0;
    std::unordered_set<int> improved_agents;
    while (std::chrono::steady_clock::now() < flow_end) {
        std::vector<int> round_agents;
        round_agents.reserve(parallel_agents);

        int scanned = 0;
        while (static_cast<int>(round_agents.size()) < parallel_agents &&
               scanned < env->num_of_agents) {
            const int agent = replan_order[cursor % env->num_of_agents].id;
            cursor++;
            scanned++;
            if (trajLNS_.traj_dists[agent].empty() || trajLNS_.trajs[agent].empty()) {
                continue;
            }
            round_agents.push_back(agent);
        }

        if (round_agents.empty()) break;

        const int failed_before = improve_failed;
        total_completed += run_parallel_guide_round(env, flow_end, round_agents,
                                                    max_threads, improve_failed,
                                                    improve_stopped_by_time,
                                                    &improved_agents);
        if (improve_failed > failed_before) break;
    }

    improve_completed = total_completed;
    if (unique_improved_agents != nullptr) {
        *unique_improved_agents = static_cast<int>(improved_agents.size());
    }

    if (std::chrono::steady_clock::now() >= flow_end) {
        improve_stopped_by_time = true;
    }
}

// ═══════════════════════════════════════════════════════════════════════════════
// plan 主入口
// ═══════════════════════════════════════════════════════════════════════════════

void EPIBTPlanner::plan(int time_limit, std::vector<std::vector<Action>>& actions,
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

    // Dummy goals
    if (env->curr_timestep == 0) {
        dummy_goals_.resize(env->num_of_agents);
        for (int i = 0; i < env->num_of_agents; i++)
            dummy_goals_[i] = env->curr_states[i].location;
    }

    // Setup episode
    prev_decision_.clear();
    prev_decision_.resize(env->map.size(), -1);
    for (int i = 0; i < env->num_of_agents; i++) {
        // Heuristic 初始化
        if (std::chrono::steady_clock::now() < flow_end) {
            for (size_t j = 0; j < env->goal_locations[i].size(); j++) {
                int goal_loc = env->goal_locations[i][j].first;
                if (trajLNS_.heuristics.find(goal_loc) == trajLNS_.heuristics.end() ||
                    trajLNS_.heuristics[goal_loc].empty()) {
                    init_heuristic(trajLNS_.heuristics[goal_loc], env, goal_loc);
                }
            }
        }

        if (env->goal_locations[i].empty()) {
            trajLNS_.tasks[i] = dummy_goals_.at(i);
            local_priority[i] = p_copy_[i];
        } else {
            trajLNS_.tasks[i] = env->goal_locations[i].front().first;
        }

        require_guide_path_[i] = false;
        if (trajLNS_.trajs[i].empty() || trajLNS_.trajs[i].back() != trajLNS_.tasks[i])
            require_guide_path_[i] = true;

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

    // Update guide paths
    int guide_required = 0;
    int guide_completed = 0;
    int guide_failed = 0;
    bool guide_stopped_by_time = false;
    const auto flow_phase_start = std::chrono::steady_clock::now();
    update_guide_paths_for_plan(env, flow_end, guide_required, guide_completed,
                                guide_failed, guide_stopped_by_time);

    const bool parallel_flow_guide = (cfg().planner_mode == 10 && cfg().epibt_parallel_guide);
    int guide_improve_completed = 0;
    int guide_improve_failed = 0;
    int guide_extra_agents = 0;
    int guide_extra_updates = 0;
    bool guide_improve_stopped_by_time = false;
    if (parallel_flow_guide) {
        improve_guide_paths_parallel_rounds(env, flow_end, guide_improve_completed,
                                            guide_improve_failed,
                                            guide_improve_stopped_by_time,
                                            &guide_extra_agents);
        guide_extra_updates = guide_improve_completed;
    } else {
        std::unordered_set<int> updated;
        guide_extra_agents = frank_wolfe(trajLNS_, updated, flow_end, guide_extra_weights(),
                                         &guide_extra_updates);
        guide_improve_completed = guide_extra_updates;
    }
    const int flow_ms = static_cast<int>(std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - flow_phase_start).count());
    const auto vflow_stats = compute_vertex_flow_hot_stats(trajLNS_.flow);

    const int guide_ready = env->num_of_agents - guide_required + guide_completed;
    const int guide_threads = std::max(1, (cfg().num_threads > 0 ? cfg().num_threads
        : std::min(4, static_cast<int>(std::thread::hardware_concurrency()))));
    std::cerr << "[EPIBT] guide=" << guide_completed << "/" << guide_required
              << " ready=" << guide_ready << "/" << env->num_of_agents
              << " extra=" << guide_extra_agents << "a/" << guide_extra_updates << "u"
              << " flow_ms=" << flow_ms
              << " vflow=a:" << vflow_stats.active
              << " max:" << vflow_stats.max
              << " hot1_avg:" << vflow_stats.hot1_avg
              << " hot10_avg:" << vflow_stats.hot10_avg
              << " hot50_avg:" << vflow_stats.hot50_avg;
    if (guide_failed > 0) std::cerr << " fail=" << guide_failed;
    if (guide_improve_failed > 0) std::cerr << " improve_fail=" << guide_improve_failed;
    if (guide_stopped_by_time || guide_improve_stopped_by_time) {
        std::cerr << " stopped=" << (guide_stopped_by_time ? "guide" : "improve");
    }
    if (parallel_flow_guide) std::cerr << " parallel_threads=" << guide_threads;
    std::cerr << "\n";

    p_ = local_priority;

    const bool deadline_fill_wait = (cfg().planner_mode == 10);
    const int step_safety_ms = std::max(5, env->num_of_agents / 1000);
    int observed_step_ms = std::max(1, pibt_time);
    int epibt_steps_completed = 0;
    int skipped_epibt_steps = 0;
    bool skipped_for_deadline = false;

    // Run EPIBT steps
    for (int step = 0; step < num_steps; step++) {
        const auto step_begin = std::chrono::steady_clock::now();
        if (step > 0) {
            const auto remaining_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                hard_deadline - step_begin).count();
            if (step_begin >= hard_deadline ||
                (deadline_fill_wait && remaining_ms <= observed_step_ms + step_safety_ms)) {
                skipped_epibt_steps = num_steps - step;
                skipped_for_deadline = true;
                break;
            }
        }

        if (step > 0) {
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

        // Sort by priority
        std::sort(ids_.begin(), ids_.end(), [&](int a, int b) {
            return local_priority.at(a) > local_priority.at(b);
        });
        std::fill(occupied_.begin(), occupied_.end(), false);

        // Run EPIBT
        for (int i : ids_) {
            if (decided_[i].state == DONE::NOT_DONE) continue;
            if (next_states_[i].location == -1) {
                assert(prev_states_[i].location >= 0 && prev_states_[i].location < (int)env->map.size());
                epibt_causal(i, -1);
            }
        }

        // Extract actions
        std::vector<Action> one_step_actions(env->num_of_agents);
        for (int id : ids_) {
            if (next_states_.at(id).location != -1)
                decision_.at(next_states_.at(id).location) = -1;
            if (next_states_.at(id).location >= 0)
                decided_.at(id) = DCR({next_states_.at(id).location, DONE::NOT_DONE});
            one_step_actions.at(id) = getAction(prev_states_.at(id), decided_.at(id).loc, env_);
            checked_.at(id) = false;
        }

        for (int id = 0; id < env->num_of_agents; id++) {
            if (!checked_.at(id) && one_step_actions.at(id) == Action::FW)
                moveCheck(id, checked_, decided_, one_step_actions, prev_decision_);
        }

        prev_states_ = next_states_;

        // Append actions + rollout
        for (int aid = 0; aid < env->num_of_agents; aid++)
            actions[aid].push_back(one_step_actions[aid]);

        std::vector<State> rolled(env->num_of_agents);
        for (int aid = 0; aid < env->num_of_agents; aid++)
            rolled[aid] = rollout_next_state(env->curr_states[aid], one_step_actions[aid], env);
        env->curr_states = rolled;
        env->curr_timestep += 1;

        epibt_steps_completed++;
        const int elapsed_step_ms = static_cast<int>(
            std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now() - step_begin).count());
        observed_step_ms = std::max(observed_step_ms, std::max(1, elapsed_step_ms));
    }

    int wait_filled_steps = 0;
    if (deadline_fill_wait) {
        for (int aid = 0; aid < env->num_of_agents; aid++) {
            while (static_cast<int>(actions[aid].size()) < num_steps) {
                actions[aid].push_back(Action::W);
            }
        }
        if (!actions.empty()) {
            wait_filled_steps = std::max(0, num_steps - epibt_steps_completed);
        }
    }

    if (deadline_fill_wait) {
        std::cerr << "[EPIBT] steps=" << epibt_steps_completed << "/" << num_steps;
        if (wait_filled_steps > 0) std::cerr << " wait_fill=" << wait_filled_steps;
        if (skipped_for_deadline) std::cerr << " deadline_skip=" << skipped_epibt_steps;
        std::cerr << " step_ms=" << observed_step_ms << "\n";
    }

    // Restore
    env->curr_states = original_states;
    env->curr_timestep = original_timestep;
    decided_ = original_decided;
}

} // namespace optsvr
