#include "TaskScheduler.h"

#include "scheduler.h"
#include "const.h"

// ============================================================
// ARCH-L R4: Task-Age-Aware Scheduling
//
// Problem: Current scheduler always picks shortest distance.
// This can cause "starvation" where long tasks are never assigned.
//
// Enhancement: Consider task AGE (how long it's been waiting).
// Older tasks get a small priority boost to ensure fairness.
//
// Formula: effective_dist = distance + congestion + length_penalty
//                              - age_bonus
// Where age_bonus increases with task waiting time.
// ============================================================

namespace {
std::unordered_set<int> free_agents;
std::unordered_set<int> free_tasks;

constexpr int REGION_GRID_SIZE = 16;
constexpr double PICKUP_PENALTY = 7.3;
constexpr double DEST_PENALTY = 0.8;
constexpr double DECAY_FACTOR = 0.95;
constexpr double LENGTH_WEIGHT = 0.5;  // Best value

// Age bonus parameters
constexpr double AGE_BONUS_RATE = 0.15;  // Re-enable
constexpr double MAX_AGE_BONUS = 4.0;     // Cap the bonus

// ============================================================
// ARCH-N R1: Free Agent Proximity Penalty
//
// Problem: region_activity only tracks HISTORICAL assignments.
// It doesn't reflect where CURRENTLY FREE agents are located.
// In room-A, many free agents cluster at doorways → path conflict.
//
// Enhancement: Count how many free agents are currently in/near
// the pickup region, and add a proximity penalty.
// This spreads agents away from crowded areas proactively.
//
// Formula: effective_dist = distance + congestion_penalty
//                              + length_penalty - age_bonus
//                              + proximity_penalty
// Where proximity_penalty = FREE_AGENT_WEIGHT * count_free_near_pickup
// ============================================================

// Free agent proximity penalty weight
// Count of free agents in the same region as pickup adds this penalty
constexpr double FREE_AGENT_WEIGHT = 0.5;

std::vector<double> region_activity;
int num_regions = 0;
int region_rows = 0;
int region_cols = 0;
int map_cols = 0;

inline int get_region_id(int location) {
    if (num_regions == 0) return 0;
    int r = location / map_cols;
    int c = location % map_cols;
    int region_r = r / region_rows;
    int region_c = c / region_cols;
    return region_r * REGION_GRID_SIZE + region_c;
}

void init_spatial_tracking(int rows, int cols) {
    map_cols = cols;
    region_rows = std::max(1, (rows + REGION_GRID_SIZE - 1) / REGION_GRID_SIZE);
    region_cols = std::max(1, (cols + REGION_GRID_SIZE - 1) / REGION_GRID_SIZE);
    num_regions = REGION_GRID_SIZE * REGION_GRID_SIZE;
    region_activity.assign(num_regions, 0.0);
}

void decay_regional_activity() {
    for (auto& activity : region_activity) {
        activity *= DECAY_FACTOR;
    }
}

void record_assignment(int pickup_loc, int dest_loc) {
    int pickup_region = get_region_id(pickup_loc);
    int dest_region = get_region_id(dest_loc);
    if (pickup_region >= 0 && pickup_region < num_regions) {
        region_activity[pickup_region] += 1.0;
    }
    if (dest_region >= 0 && dest_region < num_regions) {
        region_activity[dest_region] += 1.0;
    }
}

double get_task_congestion_penalty(int pickup_loc, int dest_loc, const std::vector<int>& free_agent_region_counts) {
    int pickup_region = get_region_id(pickup_loc);
    int dest_region = get_region_id(dest_loc);
    double penalty = 0.0;
    if (pickup_region >= 0 && pickup_region < num_regions) {
        penalty += PICKUP_PENALTY * region_activity[pickup_region];
    }
    if (dest_region >= 0 && dest_region < num_regions) {
        penalty += DEST_PENALTY * region_activity[dest_region];
    }

    // Free Agent Proximity Penalty (ARCH-N): use precomputed region counts
    // This proactively spreads agents away from crowded areas
    if (pickup_region >= 0 && pickup_region < num_regions) {
        penalty += FREE_AGENT_WEIGHT * free_agent_region_counts[pickup_region];
    }

    return penalty;
}

// Calculate task age bonus
// Older tasks (waiting longer) get a small priority boost
double calc_age_bonus(SharedEnvironment* e, int task_id) {
    const auto& task = e->task_pool[task_id];
    // task.t_revealed is when the task was revealed
    // Current timestep is env->curr_timestep
    int age = e->curr_timestep - task.t_revealed;
    if (age < 0) age = 0;  // Safety
    
    double bonus = age * AGE_BONUS_RATE;
    return std::min(bonus, MAX_AGE_BONUS);
}
}

void TaskScheduler::initialize(int preprocess_time_limit)
{
    int limit = preprocess_time_limit/2 - DefaultPlanner::SCHEDULER_TIMELIMIT_TOLERANCE;
    DefaultPlanner::schedule_initialize(limit, env);

    init_spatial_tracking(env->rows, env->cols);
}

void TaskScheduler::plan(int time_limit, std::vector<int> & proposed_schedule)
{
    TimePoint endtime = std::chrono::steady_clock::now() + std::chrono::milliseconds(time_limit);

    free_agents.insert(env->new_freeagents.begin(), env->new_freeagents.end());
    free_tasks.insert(env->new_tasks.begin(), env->new_tasks.end());

    decay_regional_activity();

    // Precompute free agent region counts (ARCH-N optimization)
    // Count how many free agents are in each region — computed once, used for all task evaluations
    std::vector<int> free_agent_region_counts(num_regions, 0);
    for (int agent_id : free_agents) {
        if (agent_id >= 0 && agent_id < (int)env->curr_states.size()) {
            int agent_region = get_region_id(env->curr_states[agent_id].location);
            if (agent_region >= 0 && agent_region < num_regions) {
                free_agent_region_counts[agent_region]++;
            }
        }
    }

    std::unordered_set<int>::iterator it = free_agents.begin();
    while (it != free_agents.end())
    {
        if (std::chrono::steady_clock::now() > endtime)
            break;

        int i = *it;
        int min_task_i = -1;
        double min_effective_dist = 1e9;
        int min_task_id = INT_MAX;
        int count = 0;

        for (int t_id : free_tasks)
        {
            if (count % 10 == 0 && std::chrono::steady_clock::now() > endtime)
                break;

            int dist = 0;
            int c_loc = env->curr_states.at(i).location;
            const auto& task_locs = env->task_pool[t_id].locations;
            int task_length = task_locs.size();
            for (int loc : task_locs)
            {
                dist += DefaultPlanner::get_h(env, c_loc, loc);
                c_loc = loc;
            }

            int pickup_loc = task_locs.empty() ? 0 : task_locs[0];
            int dest_loc = task_locs.empty() ? 0 : task_locs.back();
            double congestion_penalty = get_task_congestion_penalty(pickup_loc, dest_loc, free_agent_region_counts);
            
            // Task length penalty (from ARCH-I)
            double length_penalty = task_length * LENGTH_WEIGHT;
            
            // Task age bonus (new - older tasks get priority)
            double age_bonus = calc_age_bonus(env, t_id);
            
            // Effective distance = raw metrics - age bonus
            // (Subtract because lower is better, age bonus increases priority)
            double effective_dist = dist + congestion_penalty + length_penalty - age_bonus;

            if (effective_dist < min_effective_dist)
            {
                min_effective_dist = effective_dist;
                min_task_i = t_id;
                min_task_id = t_id;
            }
            else if (effective_dist == min_effective_dist)
            {
                if (t_id < min_task_id)
                {
                    min_task_id = t_id;
                    min_task_i = t_id;
                }
            }
            count++;
        }

        if (min_task_i != -1)
        {
            proposed_schedule[i] = min_task_i;

            const auto& task_locs = env->task_pool[min_task_i].locations;
            int pickup_loc = task_locs.empty() ? 0 : task_locs[0];
            int dest_loc = task_locs.empty() ? 0 : task_locs.back();
            record_assignment(pickup_loc, dest_loc);

            it = free_agents.erase(it);
            free_tasks.erase(min_task_i);
        }
        else
        {
            proposed_schedule[i] = -1;
            ++it;
        }
    }

    for (int a_id : free_agents)
        proposed_schedule[a_id] = -1;
}
