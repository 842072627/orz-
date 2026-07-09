#include "Executor.h"
#include "RuntimeConfig.h"
#include "MapTuner.h"
#include "GameExecutor.h"
#include <algorithm>
#include <climits>
#include <cmath>
#include <cstdlib>
#include <iomanip>
#include <list>
#include <string>
#include <sstream>

#include <boost/log/trivial.hpp>
#include <boost/log/sources/severity_logger.hpp>
#include <boost/log/sources/record_ostream.hpp>
namespace blog = boost::log;
extern blog::sources::severity_logger<blog::trivial::severity_level> lg;

// GameExecutor 执行器实例 (Game专用: 对流检测+截断, EXECUTOR_MODE=6)
static optsvr::GameExecutor g_game_executor;

namespace {

struct TpgTraceConfig {
    bool enabled = false;
    int row = 992;
    int col = 1624;
    int agent = -1;
    int max_lines = 20000;
};

TpgTraceConfig load_tpg_trace_config()
{
    TpgTraceConfig cfg;
    if (const char* v = std::getenv("OPT_TPG_TRACE")) {
        std::string s(v);
        cfg.enabled = (s == "1" || s == "true" || s == "TRUE" || s == "on" || s == "ON");
    }
    if (const char* v = std::getenv("OPT_TPG_TRACE_ROW")) cfg.row = std::atoi(v);
    if (const char* v = std::getenv("OPT_TPG_TRACE_COL")) cfg.col = std::atoi(v);
    if (const char* v = std::getenv("OPT_TPG_TRACE_AGENT")) cfg.agent = std::atoi(v);
    if (const char* v = std::getenv("OPT_TPG_TRACE_MAX_LINES")) cfg.max_lines = std::max(1, std::atoi(v));
    return cfg;
}

const TpgTraceConfig& tpg_trace_cfg()
{
    static TpgTraceConfig cfg = load_tpg_trace_config();
    return cfg;
}

bool tpg_trace_consume_line()
{
    static int lines = 0;
    const auto& cfg = tpg_trace_cfg();
    if (!cfg.enabled || lines >= cfg.max_lines) return false;
    lines++;
    return true;
}

char action_to_char(Action action)
{
    switch (action) {
        case Action::FW: return 'F';
        case Action::CR: return 'R';
        case Action::CCR: return 'C';
        case Action::W: return 'W';
        case Action::NA: return 'T';
        default: return '?';
    }
}

int trace_target_loc(const SharedEnvironment* env)
{
    const auto& cfg = tpg_trace_cfg();
    if (!cfg.enabled || env == nullptr || env->cols <= 0) return -1;
    return cfg.row * env->cols + cfg.col;
}

bool trace_agent_or_loc(const SharedEnvironment* env, int agent_id, int loc_a, int loc_b = -1)
{
    const auto& cfg = tpg_trace_cfg();
    if (!cfg.enabled) return false;
    if (cfg.agent >= 0 && cfg.agent != agent_id) return false;
    const int target = trace_target_loc(env);
    if (target < 0) return false;
    const int south = target + env->cols;
    return loc_a == target || loc_b == target || loc_a == south || loc_b == south;
}

std::string queue_summary(const std::list<int>& q)
{
    std::ostringstream os;
    os << "[";
    int shown = 0;
    for (int id : q) {
        if (shown > 0) os << ",";
        os << id;
        shown++;
        if (shown >= 4) break;
    }
    if ((int)q.size() > shown) os << ",...";
    os << "]";
    return os.str();
}

std::string loc_to_rc(const SharedEnvironment* env, int loc)
{
    if (env == nullptr || env->cols <= 0 || loc < 0) return "(-,-)";
    std::ostringstream os;
    os << "(" << loc / env->cols << "," << loc % env->cols << ")";
    return os.str();
}

} // namespace

void Executor::initialize(int preprocess_time_limit)
{
    optsvr::load_map_tuning(env);
    if (optsvr::cfg().skip_test) return;

    tpg.resize(env->map.size());
    previous_locations.resize(env->num_of_agents, -1);
    official_process_initialized_ = false;

    max_counter_ = std::max(1, env->max_counter);

    // MapTuner 内部做一次性保护；这里保留环境变量兜底，方便本地 A/B。
    int exec_mode = optsvr::cfg().executor_mode;
    if (const char* v = std::getenv("OPT_EXECUTOR_MODE")) exec_mode = std::atoi(v);

    int tmp = std::max(1, env->action_time * env->max_counter);
    action_ceil_size = (env->min_planner_communication_time + tmp - 1) / tmp;
    stage_action_max = max(action_ceil_size * 2, 5);

    if (process_plan_type == APPEND_WINDOW) {
        if ((exec_mode == 3 || exec_mode == 5) && optsvr::cfg().dh_executor_ceil_window) {
            window_size = (env->min_planner_communication_time + tmp - 1) / tmp + 1;
        } else {
            window_size = env->min_planner_communication_time / tmp + 1;
        }
        if (window_size < 1) window_size = 1;
    } else if (process_plan_type == APPEND_ONE) {
        window_size = 1;
    } else {
        window_size = INT_MAX;
    }

    // 初始化各模式执行器。Executor::initialize 与 Entry::initialize 并行执行。
    if (exec_mode == 6) {
        g_game_executor.initialize(this);
    }
}

bool Executor::is_valid_location(int location) const
{
    return location >= 0 &&
           location < (int)tpg.size() &&
           location < (int)env->map.size() &&
           env->map[location] == 0;
}

bool Executor::contains_agent_in_tpg(int location, int agent_id) const
{
    if (!is_valid_location(location)) return false;
    const auto& q = tpg[location];
    return std::find(q.begin(), q.end(), agent_id) != q.end();
}

bool Executor::erase_agent_from_tpg(int location, int agent_id)
{
    if (!is_valid_location(location)) return false;
    auto& q = tpg[location];
    auto it = std::find(q.begin(), q.end(), agent_id);
    if (it == q.end()) return false;
    q.erase(it);
    return true;
}

void Executor::rebuild_from_system_states()
{
    if ((int)previous_locations.size() != env->num_of_agents)
        previous_locations.assign(env->num_of_agents, -1);

    predicted_states = env->system_states;
    for (auto& q : tpg) q.clear();

    for (int i = 0; i < env->num_of_agents; i++) {
        const int loc = env->system_states[i].location;
        previous_locations[i] = loc;
        if (is_valid_location(loc)) tpg[loc].push_back(i);
    }
    official_process_initialized_ = true;
}

// ════════════════════════════════════════════════════════════════════
// Default 模式 process_new_plan：标准 append/window + TPG
// ════════════════════════════════════════════════════════════════════
vector<State> Executor::process_new_plan_official(int sync_time_limit, Plan& plan_struct,
                                                 vector<vector<Action>>& staged_actions)
{
    if (predicted_states.size() != (size_t)env->num_of_agents)
        predicted_states = env->system_states;

    bool need_rebuild = !official_process_initialized_ ||
                        (int)previous_locations.size() != env->num_of_agents;
    if (env->system_timestep == 0) need_rebuild = true;
    if (need_rebuild) rebuild_from_system_states();

    const std::vector<std::vector<Action>>& plan = plan_struct.actions;
    if (plan.empty() || (int)plan.size() != env->num_of_agents)
        return predicted_states;

    int plan_horizon = INT_MAX;
    for (int i = 0; i < env->num_of_agents; i++)
        plan_horizon = std::min(plan_horizon, (int)plan[i].size());
    if (plan_horizon == INT_MAX) plan_horizon = 0;

    auto curr_states = predicted_states;
    int moves[4] = {1, env->cols, -1, -env->cols};
    const int append_timesteps = std::min(plan_horizon, window_size);

    for (int timestep = 0; timestep < append_timesteps; timestep++) {
        for (int i = 0; i < (int)plan.size(); i++) {
            Action action = plan[i][timestep];
            int new_location = curr_states[i].location;
            int new_orientation = curr_states[i].orientation;
            bool executable_action = (action != Action::NA && action != Action::W);

            if (action == Action::FW) {
                const int candidate = new_location + moves[curr_states[i].orientation];
                if (is_valid_location(candidate)) {
                    new_location = candidate;
                    tpg[new_location].push_back(i);
                } else executable_action = false;
            } else if (action == Action::CR) {
                new_orientation = (curr_states[i].orientation + 1) % 4;
            } else if (action == Action::CCR) {
                new_orientation = (curr_states[i].orientation + 3) % 4;
            }

            curr_states[i].location = new_location;
            curr_states[i].orientation = new_orientation;
            curr_states[i].timestep += 1;
            if (executable_action) staged_actions[i].push_back(action);
        }
    }

    // MCP 模拟裁剪（Default 模式：无跟随）
    vector<int> action_index(env->num_of_agents, 0);
    curr_states = env->system_states;
    std::vector<std::list<int>> temp_tpg(env->map.size());

    for (int t = 0; t < window_size; t++) {
        auto pre_states = curr_states;
        for (int i = 0; i < env->num_of_agents; i++) {
            if ((int)staged_actions[i].size() <= action_index[i]) {
                if (trace_agent_or_loc(env, i, curr_states[i].location) && tpg_trace_consume_line()) {
                    const int loc = curr_states[i].location;
                    std::fprintf(stderr,
                        "[TPG_TRACE_PLAN_CROP] tick=%d step=%d agent=%d loc=%s reason=no_staged "
                        "staged=%zu idx=%d currQ=%s\n",
                        env->system_timestep, t, i, loc_to_rc(env, loc).c_str(),
                        staged_actions[i].size(), action_index[i],
                        (is_valid_location(loc) ? queue_summary(tpg[loc]).c_str() : "[]"));
                }
                curr_states[i].timestep++;
                continue;
            }
            Action curr_action = staged_actions[i][action_index[i]];
            int curr_location = curr_states[i].location;
            int curr_orientation = curr_states[i].orientation;
            int new_location = curr_location, new_orientation = curr_orientation;
            const bool at_tpg_front = is_valid_location(curr_location) &&
                !tpg[curr_location].empty() && tpg[curr_location].front() == i;

            if (curr_action == Action::FW) {
                const int candidate = curr_location + moves[curr_orientation];
                bool advanced = false;
                if (at_tpg_front && is_valid_location(candidate) &&
                    !tpg[candidate].empty() && tpg[candidate].front() == i) {
                    new_location = candidate; action_index[i]++; advanced = true;
                }
                if (trace_agent_or_loc(env, i, curr_location, candidate) && tpg_trace_consume_line()) {
                    const char* reason = "advance";
                    if (!at_tpg_front) reason = "blocked_curr_front";
                    else if (!is_valid_location(candidate)) reason = "invalid_next";
                    else if (tpg[candidate].empty()) reason = "empty_next_q";
                    else if (tpg[candidate].front() != i) reason = "blocked_next_front";
                    std::fprintf(stderr,
                        "[TPG_TRACE_PLAN_CROP] tick=%d step=%d agent=%d loc=%s next=%s action=F "
                        "result=%s advanced=%d staged=%zu idx=%d currQ=%s nextQ=%s\n",
                        env->system_timestep, t, i,
                        loc_to_rc(env, curr_location).c_str(),
                        loc_to_rc(env, candidate).c_str(),
                        reason, advanced ? 1 : 0,
                        staged_actions[i].size(), action_index[i],
                        (is_valid_location(curr_location) ? queue_summary(tpg[curr_location]).c_str() : "[]"),
                        (is_valid_location(candidate) ? queue_summary(tpg[candidate]).c_str() : "[]"));
                }
            } else if (curr_action == Action::CR) {
                new_orientation = (curr_orientation + 1) % 4;
                if (at_tpg_front) action_index[i]++;
                if (trace_agent_or_loc(env, i, curr_location) && tpg_trace_consume_line()) {
                    std::fprintf(stderr,
                        "[TPG_TRACE_PLAN_CROP] tick=%d step=%d agent=%d loc=%s action=R "
                        "result=%s staged=%zu idx=%d currQ=%s\n",
                        env->system_timestep, t, i,
                        loc_to_rc(env, curr_location).c_str(),
                        at_tpg_front ? "advance" : "blocked_curr_front",
                        staged_actions[i].size(), action_index[i],
                        (is_valid_location(curr_location) ? queue_summary(tpg[curr_location]).c_str() : "[]"));
                }
            } else if (curr_action == Action::CCR) {
                new_orientation = (curr_orientation + 3) % 4;
                if (at_tpg_front) action_index[i]++;
                if (trace_agent_or_loc(env, i, curr_location) && tpg_trace_consume_line()) {
                    std::fprintf(stderr,
                        "[TPG_TRACE_PLAN_CROP] tick=%d step=%d agent=%d loc=%s action=C "
                        "result=%s staged=%zu idx=%d currQ=%s\n",
                        env->system_timestep, t, i,
                        loc_to_rc(env, curr_location).c_str(),
                        at_tpg_front ? "advance" : "blocked_curr_front",
                        staged_actions[i].size(), action_index[i],
                        (is_valid_location(curr_location) ? queue_summary(tpg[curr_location]).c_str() : "[]"));
                }
            }
            curr_states[i].location = new_location;
            curr_states[i].orientation = new_orientation;
            curr_states[i].timestep += 1;
        }
        for (int i = 0; i < env->num_of_agents; i++) {
            const int prev_loc = pre_states[i].location, next_loc = curr_states[i].location;
            if (t == 0 && is_valid_location(prev_loc)) temp_tpg[prev_loc].push_back(i);
            if (prev_loc != next_loc) {
                if (is_valid_location(prev_loc)) erase_agent_from_tpg(prev_loc, i);
                if (is_valid_location(next_loc)) temp_tpg[next_loc].push_back(i);
            }
        }
    }

    for (int i = 0; i < env->num_of_agents; i++) {
        if (action_index[i] < (int)staged_actions[i].size()) staged_actions[i].resize(action_index[i]);
        previous_locations[i] = env->system_states[i].location;
    }
    predicted_states = curr_states;
    tpg.swap(temp_tpg);
    official_process_initialized_ = true;

    return predicted_states;
}

// ════════════════════════════════════════════════════════════════════
// 分流入口
// ════════════════════════════════════════════════════════════════════
vector<State> Executor::process_new_plan(int sync_time_limit, Plan& plan_struct,
                                         vector<vector<Action>>& staged_actions)
{
    if (optsvr::cfg().skip_test) {
        staged_actions.assign(env->num_of_agents, {});
        predicted_states = env->system_states;
        return predicted_states;
    }

    if (optsvr::cfg().executor_mode == 6) {
        if (!g_game_executor.is_initialized()) g_game_executor.initialize(this);
        return g_game_executor.process_new_plan(sync_time_limit, plan_struct, staged_actions);
    }
    return process_new_plan_official(sync_time_limit, plan_struct, staged_actions);
}

void Executor::next_command(int exec_time_limit, std::vector<ExecutionCommand>& agent_command)
{
    if (optsvr::cfg().skip_test) {
        agent_command.resize(env->num_of_agents);
        std::fill(agent_command.begin(), agent_command.end(), ExecutionCommand::STOP);
        return;
    }

    if (optsvr::cfg().executor_mode == 6) {
        if (!g_game_executor.is_initialized()) g_game_executor.initialize(this);
        g_game_executor.next_command(exec_time_limit, agent_command);
        return;
    }

    // 更新 TPG（Default/Origin/IncrementalTpg 模式共用）
    for (int i = 0; i < (int)env->system_states.size(); i++) {
        int prev_location = previous_locations[i];
        int curr_location = env->system_states[i].location;
        if (prev_location != curr_location) {
            erase_agent_from_tpg(prev_location, i);
            previous_locations[i] = curr_location;
            if (is_valid_location(curr_location)) {
                if (!contains_agent_in_tpg(curr_location, i))
                    tpg[curr_location].push_front(i);
            }
        }
    }

    // 按模式分流 MCP 决策
    // Default: 基础 MCP（无跟随）
    for (int i = 0; i < (int)env->system_states.size(); i++)
        mcp_default_executor(i, agent_command);
}

bool Executor::mcp_default_executor(int agent_id, std::vector<ExecutionCommand>& agent_command)
{
    int moves[4] = {1, env->cols, -1, -env->cols};
    int curr_location = env->system_states[agent_id].location;
    int curr_orientation = env->system_states[agent_id].orientation;
    int next_location = curr_location;
    if (curr_orientation >= 0 && curr_orientation < 4)
        next_location = curr_location + moves[curr_orientation];

    if (env->staged_actions[agent_id].empty()) {
        if (trace_agent_or_loc(env, agent_id, curr_location, next_location) && tpg_trace_consume_line()) {
            std::fprintf(stderr,
                "[TPG_TRACE_MCP] tick=%d agent=%d loc=%s next=%s ori=%d decision=STOP "
                "reason=staged_empty currQ=%s nextQ=%s\n",
                env->system_timestep, agent_id,
                loc_to_rc(env, curr_location).c_str(),
                loc_to_rc(env, next_location).c_str(),
                curr_orientation,
                (is_valid_location(curr_location) ? queue_summary(tpg[curr_location]).c_str() : "[]"),
                (is_valid_location(next_location) ? queue_summary(tpg[next_location]).c_str() : "[]"));
        }
        agent_command[agent_id] = ExecutionCommand::STOP; return false;
    } else if (env->staged_actions[agent_id].front() != Action::FW) {
        if (trace_agent_or_loc(env, agent_id, curr_location, next_location) && tpg_trace_consume_line()) {
            std::fprintf(stderr,
                "[TPG_TRACE_MCP] tick=%d agent=%d loc=%s next=%s ori=%d decision=GO "
                "reason=non_fw action=%c staged=%zu currQ=%s nextQ=%s\n",
                env->system_timestep, agent_id,
                loc_to_rc(env, curr_location).c_str(),
                loc_to_rc(env, next_location).c_str(),
                curr_orientation,
                action_to_char(env->staged_actions[agent_id].front()),
                env->staged_actions[agent_id].size(),
                (is_valid_location(curr_location) ? queue_summary(tpg[curr_location]).c_str() : "[]"),
                (is_valid_location(next_location) ? queue_summary(tpg[next_location]).c_str() : "[]"));
        }
        agent_command[agent_id] = ExecutionCommand::GO; return false;
    } else {
        if (!is_valid_location(next_location)) {
            if (trace_agent_or_loc(env, agent_id, curr_location, next_location) && tpg_trace_consume_line()) {
                std::fprintf(stderr,
                    "[TPG_TRACE_MCP] tick=%d agent=%d loc=%s next=%s ori=%d decision=STOP "
                    "reason=invalid_next staged=%zu currQ=%s\n",
                    env->system_timestep, agent_id,
                    loc_to_rc(env, curr_location).c_str(),
                    loc_to_rc(env, next_location).c_str(),
                    curr_orientation,
                    env->staged_actions[agent_id].size(),
                    (is_valid_location(curr_location) ? queue_summary(tpg[curr_location]).c_str() : "[]"));
            }
            agent_command[agent_id] = ExecutionCommand::STOP; return false;
        }
        if (tpg[next_location].empty() || tpg[next_location].front() == agent_id) {
            if (trace_agent_or_loc(env, agent_id, curr_location, next_location) && tpg_trace_consume_line()) {
                std::fprintf(stderr,
                    "[TPG_TRACE_MCP] tick=%d agent=%d loc=%s next=%s ori=%d decision=GO "
                    "reason=front_ok staged=%zu currQ=%s nextQ=%s\n",
                    env->system_timestep, agent_id,
                    loc_to_rc(env, curr_location).c_str(),
                    loc_to_rc(env, next_location).c_str(),
                    curr_orientation,
                    env->staged_actions[agent_id].size(),
                    (is_valid_location(curr_location) ? queue_summary(tpg[curr_location]).c_str() : "[]"),
                    queue_summary(tpg[next_location]).c_str());
            }
            agent_command[agent_id] = ExecutionCommand::GO; return true;
        } else {
            if (trace_agent_or_loc(env, agent_id, curr_location, next_location) && tpg_trace_consume_line()) {
                std::fprintf(stderr,
                    "[TPG_TRACE_MCP] tick=%d agent=%d loc=%s next=%s ori=%d decision=STOP "
                    "reason=blocked_next_front front=%d staged=%zu currQ=%s nextQ=%s\n",
                    env->system_timestep, agent_id,
                    loc_to_rc(env, curr_location).c_str(),
                    loc_to_rc(env, next_location).c_str(),
                    curr_orientation,
                    tpg[next_location].empty() ? -1 : tpg[next_location].front(),
                    env->staged_actions[agent_id].size(),
                    (is_valid_location(curr_location) ? queue_summary(tpg[curr_location]).c_str() : "[]"),
                    queue_summary(tpg[next_location]).c_str());
            }
            agent_command[agent_id] = ExecutionCommand::STOP; return false;
        }
    }
}
