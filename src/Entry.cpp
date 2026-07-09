#include "Entry.h"
#include "Tasks.h"
#include "utils.h"
#include "heuristics.h"
#include "RuntimeConfig.h"
#include "IdleParking.h"
#include "orz_highway.h"
#include "MapTuner.h"
#include <algorithm>
#include <chrono>
#include <cstdint>
#include <queue>
#include <random>
#include <vector>

// Forward declarations
void apply_game_directional_idle_relocation(
    SharedEnvironment* env, const std::vector<int>& proposed_schedule);

void Entry::initialize(int preprocess_time_limit)
{
    optsvr::load_map_tuning(env);
    scheduler->initialize(preprocess_time_limit);
    planner->initialize(preprocess_time_limit);
}

void Entry::compute(int time_limit, Plan& plan, std::vector<int>& proposed_schedule)
{
    if (optsvr::cfg().skip_test) {
        plan.actions.resize(env->num_of_agents);
        return;
    }

    scheduler->plan(time_limit, proposed_schedule);

    update_goal_locations(proposed_schedule);
    optsvr::idle_parking().apply_goal_locations(env);
    apply_game_directional_idle_relocation(env, proposed_schedule);

    planner->plan(time_limit, plan);
}

void Entry::update_goal_locations(std::vector<int>& proposed_schedule)
{
    env->curr_task_schedule = proposed_schedule;
    for (size_t i = 0; i < proposed_schedule.size(); i++) {
        env->goal_locations[i].clear();
        int t_id = proposed_schedule[i];
        if (t_id == -1) continue;
        int i_loc = env->task_pool[t_id].idx_next_loc;
        env->goal_locations[i].push_back({env->task_pool[t_id].locations.at(i_loc), env->task_pool[t_id].t_revealed});
    }
}

// ====== ORZ-specific free functions (from game_13000 Entry.cpp) ======

static int nearby_random_non_highway_goal(SharedEnvironment* env, int loc, int agent_id,
    std::vector<unsigned char>& occupied)
{
    std::mt19937 rng(std::random_device{}());
    const int max_radius = 60;
    for (int r = 1; r <= max_radius; r += 3) {
        std::vector<int> candidates;
        for (int dr = -r; dr <= r; dr++) {
            for (int dc = -r; dc <= r; dc++) {
                int nl = loc + dr * env->cols + dc;
                if (nl < 0 || nl >= (int)env->map.size() || !env->map[nl]) continue;
                if (ORZ_Highway::is_highway_cell(nl)) continue;
                if (occupied[nl]) continue;
                candidates.push_back(nl);
            }
        }
        if (!candidates.empty()) {
            std::shuffle(candidates.begin(), candidates.end(), rng);
            return candidates.front();
        }
    }
    return -1;
}

void apply_game_directional_idle_relocation(
    SharedEnvironment* env, const std::vector<int>& proposed_schedule)
{
    if (env == nullptr || optsvr::cfg().planner_mode != 16 ||
        !optsvr::cfg().game_idle_highway_relocate) return;

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
        if (!ORZ_Highway::is_highway_cell(loc)) continue;

        const int goal = nearby_random_non_highway_goal(env, loc, i, occupied);
        if (goal < 0) continue;
        if (goal < static_cast<int>(occupied.size())) occupied[goal] = 1;

        env->goal_locations[i].clear();
        env->goal_locations[i].push_back({goal, -1});
    }
}
