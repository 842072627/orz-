#include "Entry.h"
#include "Tasks.h"
#include "utils.h"
#include "heuristics.h"
#include "RuntimeConfig.h"
#include "IdleParking.h"
#include "PerfStats.h"
#include "orz_highway.h"
#include <algorithm>
#include <chrono>
#include <cstdint>
#include <queue>

namespace {

bool game_directional_move_allowed(SharedEnvironment* env, int from, int to) {
    if (from == to) return true;
    if (env == nullptr || from < 0 || to < 0 ||
        from >= static_cast<int>(env->map.size()) ||
        to >= static_cast<int>(env->map.size()) ||
        env->map[from] != 0 || env->map[to] != 0) {
        return false;
    }
    const int fr = from / env->cols;
    const int fc = from % env->cols;
    const int tr = to / env->cols;
    const int tc = to % env->cols;
    if (std::abs(fr - tr) + std::abs(fc - tc) != 1) return false;

    int dir = -1;
    const int delta = to - from;
    if (delta == 1) dir = 0;
    else if (delta == env->cols) dir = 1;
    else if (delta == -1) dir = 2;
    else if (delta == -env->cols) dir = 3;
    if (dir < 0) return false;

    const uint8_t forbid = ORZ_Highway::get_forbid(from);
    if (dir == 0 && (forbid & 2)) return false;
    if (dir == 1 && (forbid & 8)) return false;
    if (dir == 2 && (forbid & 1)) return false;
    if (dir == 3 && (forbid & 4)) return false;
    return true;
}

uint32_t mix32(uint32_t x) {
    x ^= x >> 16;
    x *= 0x7feb352dU;
    x ^= x >> 15;
    x *= 0x846ca68bU;
    x ^= x >> 16;
    return x;
}

bool adjacent_to_highway(const SharedEnvironment* env, int loc) {
    if (env == nullptr || loc < 0 || env->cols <= 0) return false;
    const int map_size = static_cast<int>(env->map.size());
    if (loc >= map_size) return false;

    const int dirs[4] = {-env->cols, 1, env->cols, -1};
    const int r = loc / env->cols;
    const int c = loc % env->cols;
    for (int dir : dirs) {
        const int nb = loc + dir;
        if (nb < 0 || nb >= map_size) continue;
        const int nr = nb / env->cols;
        const int nc = nb % env->cols;
        if (std::abs(nr - r) + std::abs(nc - c) != 1) continue;
        if (ORZ_Highway::is_highway_cell(nb)) return true;
    }
    return false;
}

int nearby_random_non_highway_goal(SharedEnvironment* env, int start_loc, int agent_id,
                                   std::vector<unsigned char>& occupied) {
    if (env == nullptr || start_loc < 0 || env->cols <= 0) return -1;
    const int map_size = static_cast<int>(env->map.size());
    if (start_loc >= map_size || env->map[start_loc] != 0) return -1;

    static constexpr int kMinDepth = 8;
    static constexpr int kMaxDepth = 48;
    static constexpr int kMaxCandidates = 96;

    struct Node {
        int loc;
        int depth;
    };

    std::vector<unsigned char> seen(map_size, 0);
    std::queue<Node> q;
    q.push({start_loc, 0});
    seen[start_loc] = 1;

    const int base_dirs[4] = {-env->cols, 1, env->cols, -1};
    const int rotate = ((agent_id % 4) + 4) % 4;
    std::vector<int> preferred;
    std::vector<int> usable;
    int nearest_any = -1;

    while (!q.empty()) {
        const Node cur_node = q.front();
        q.pop();
        const int cur = cur_node.loc;
        const int depth = cur_node.depth;

        if (!ORZ_Highway::is_highway_cell(cur) && ORZ_Highway::get_sign(cur) == 0) {
            if (nearest_any < 0) nearest_any = cur;
            const bool free_cell = cur >= static_cast<int>(occupied.size()) || !occupied[cur];
            if (free_cell) {
                if (usable.empty()) usable.push_back(cur);
                if (depth >= kMinDepth) {
                    if (!adjacent_to_highway(env, cur)) {
                        preferred.push_back(cur);
                    } else if (preferred.empty()) {
                        usable.push_back(cur);
                    }
                }
            }
            if ((int)preferred.size() >= kMaxCandidates) break;
        }

        if (depth >= kMaxDepth && (!preferred.empty() || nearest_any >= 0)) continue;
        for (int k = 0; k < 4; ++k) {
            const int dir = base_dirs[(k + rotate) % 4];
            const int nb = cur + dir;
            if (nb < 0 || nb >= map_size || seen[nb]) continue;
            if (!game_directional_move_allowed(env, cur, nb)) continue;
            seen[nb] = 1;
            q.push({nb, depth + 1});
        }
    }

    const std::vector<int>& pool = !preferred.empty() ? preferred : usable;
    if (!pool.empty()) {
        const uint32_t key = mix32(static_cast<uint32_t>(agent_id) * 0x9e3779b9U ^
                                   static_cast<uint32_t>(start_loc) * 0x85ebca6bU);
        return pool[key % pool.size()];
    }
    return nearest_any;
}

void apply_game_directional_idle_relocation(
    SharedEnvironment* env, const std::vector<int>& proposed_schedule) {
    if (env == nullptr || nz::cfg().planner_mode != 16 ||
        !nz::cfg().game_idle_highway_relocate) return;

    std::vector<unsigned char> occupied(env->map.size(), 0);
    for (const auto& state : env->curr_states) {
        if (state.location >= 0 && state.location < static_cast<int>(occupied.size()))
            occupied[state.location] = 1;
    }

    const int n = std::min<int>(
        std::min(proposed_schedule.size(), env->curr_states.size()),
        std::min(env->goal_locations.size(), static_cast<size_t>(env->num_of_agents)));
    for (int i = 0; i < n; ++i) {
        if (proposed_schedule[i] != -1) continue;
        const int loc = env->curr_states[i].location;
        if (!ORZ_Highway::is_highway_cell(loc) && ORZ_Highway::get_sign(loc) == 0) continue;

        const int goal = nearby_random_non_highway_goal(env, loc, i, occupied);
        if (goal < 0) continue;
        if (goal < static_cast<int>(occupied.size())) occupied[goal] = 1;

        env->goal_locations[i].clear();
        env->goal_locations[i].push_back({goal, -1});
    }
}

}  // namespace

// The initialize function will be called by competition system at the preprocessing stage.
// Implement the initialize functions of the planner and scheduler to load or compute auxiliary data.
// Note that, this function runs untill preprocess_time_limit (in milliseconds) is reached.
// This is an offline step, after it completes then evaluation begins.
void Entry::initialize(int preprocess_time_limit)
{
    nz::perf().init();
    scheduler->initialize(preprocess_time_limit);
    planner->initialize(preprocess_time_limit);
}

//The compute function will be called by competition system on each timestep.
//It computes:
//  1. a schedule that specifies which agent complete which task.
//  2. a next action that specifies how each agent should move in the next timestep.
//NB: the parameter time_limit is specified in milliseconds.
void Entry::compute(int time_limit, Plan & plan, std::vector<int> & proposed_schedule)
{

    // 首轮 compute 时 task_pool 已填充，用 task/agent 比例做 skip_test 延迟判断
    // static bool first_compute = true;
    // if (first_compute) {
    //     first_compute = false;
    //     float task_ratio = static_cast<float>(env->task_pool.size())
    //                      / std::max(1, env->num_of_agents);
    //     if (task_ratio > 1.05) {
    //         nz::cfg().skip_test = true;
    //     }
    // }
    //nz::cfg().skip_test = false;

    // 跳过当前算例：不做任务分配和路径规划，所有 agent 保持静止
    if (nz::cfg().skip_test) {
        // 必须设置正确大小的空 plan，否则 Simulator 的 validate_staged_actions_prefix 校验会抛异常
        plan.actions.resize(env->num_of_agents);
        return;
    }

    using clk = std::chrono::steady_clock;
    auto t0 = clk::now();

    //call the task scheduler to assign tasks to agents
    {
        int sched_limit = time_limit;
        float ratio = nz::cfg().game_sched_time_ratio;
        if (ratio > 0.0f)
            sched_limit = (int)((float)time_limit * std::min(ratio, 1.0f));
        scheduler->plan(sched_limit, proposed_schedule);
    }

    auto t1 = clk::now();

    //then update the first unfinished errand/location of tasks for planner reference
    update_goal_locations(proposed_schedule);
    nz::idle_parking().apply_goal_locations(env);
    apply_game_directional_idle_relocation(env, proposed_schedule);

    //call the planner to compute the actions
    planner->plan(time_limit,plan);

    auto t2 = clk::now();

    // 记录性能统计
    auto us = [](auto d){ return std::chrono::duration_cast<std::chrono::microseconds>(d).count(); };
    nz::perf().record(us(t1-t0), us(t2-t1), us(t2-t0));

    // 每 100 tick 打印耗时（独立计时，不依赖 OPT_ENABLE_STATS）
    static auto t_last = clk::now();
    static int print_n = 0;
    print_n++;
    auto t_now = clk::now();
    auto tick_us = std::chrono::duration_cast<std::chrono::microseconds>(t_now - t_last).count();
    t_last = t_now;
    if (print_n % 100 == 0) {
        fprintf(stderr, "[PERF] tick=%d sched_ms=%.1f planner_ms=%.1f tick_us=%lld limit=%dms\n",
                nz::perf().call_count,
                std::chrono::duration_cast<std::chrono::milliseconds>(t1-t0).count(),
                std::chrono::duration_cast<std::chrono::milliseconds>(t2-t1).count(),
                (long long)tick_us, time_limit);
        fflush(stderr);
    }
}

// Set the next goal locations for each agent based on the proposed schedule
void Entry::update_goal_locations(std::vector<int> & proposed_schedule)
{
    // record the proposed schedule so that we can tell the planner
    env->curr_task_schedule = proposed_schedule;

    // The first unfinished errand/location of each task is the next goal for the assigned agent.
    for (size_t i = 0; i < proposed_schedule.size(); i++)
    {
        env->goal_locations[i].clear();
        int t_id = proposed_schedule[i];
        if (t_id == -1)
            continue;

        int i_loc = env->task_pool[t_id].idx_next_loc;
        env->goal_locations[i].push_back({env->task_pool[t_id].locations.at(i_loc), env->task_pool[t_id].t_revealed});
    }
    return;
}
