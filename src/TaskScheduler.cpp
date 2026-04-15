#include "TaskScheduler.h"

#include "scheduler.h"
#include "const.h"

// ============================================================
// ARCH-B R1: Spatial Load Balancing Scheduler
// Core idea: Penalize tasks in "hot" regions (recently active areas)
// to spread agents out and reduce path congestion.
//
// Key insight from ARCH-A:
//   - min-Task-ID (R6) excelled at fulfill (0.640) but struggled at room/maze
//   - reverse greedy excelled at room/maze but failed at fulfill
//   - Neither handles spatial congestion properly
//
// ARCH-B approach:
//   - Divide map into a grid of regions
//   - Track task assignments per region with exponential decay
//   - Add congestion penalty to distance: effective_dist = dist + alpha * region_activity
//   - This spreads agents geographically, reducing convoy effects
//
// Map grid: 16x16 regions max (256 total)
// Region size = ceil(map_size / 16)
// ============================================================

namespace {
std::unordered_set<int> free_agents;
std::unordered_set<int> free_tasks;

// ARCH-B: Spatial load balancing state
constexpr int REGION_GRID_SIZE = 16;  // 16x16 regions
constexpr double CONGESTION_PENALTY = 5.0;  // Penalty per unit of regional activity
constexpr double DECAY_FACTOR = 0.95;  // Exponential decay for activity tracking

// Regional activity tracking (circular buffer of recent activity)
std::vector<double> region_activity;
int num_regions = 0;
int region_rows = 0;
int region_cols = 0;
int map_cols = 0;

// Get region ID for a location
inline int get_region_id(int location) {
    if (num_regions == 0) return 0;
    int r = location / map_cols;
    int c = location % map_cols;
    int region_r = r / region_rows;
    int region_c = c / region_cols;
    return region_r * REGION_GRID_SIZE + region_c;
}

// Initialize regional tracking
void init_spatial_tracking(int rows, int cols) {
    map_cols = cols;
    region_rows = std::max(1, (rows + REGION_GRID_SIZE - 1) / REGION_GRID_SIZE);
    region_cols = std::max(1, (cols + REGION_GRID_SIZE - 1) / REGION_GRID_SIZE);
    num_regions = REGION_GRID_SIZE * REGION_GRID_SIZE;
    region_activity.assign(num_regions, 0.0);
}

// Decay all regional activity (call once per planning round)
void decay_regional_activity() {
    for (auto& activity : region_activity) {
        activity *= DECAY_FACTOR;
    }
}

// Record task assignment in regional tracking
void record_assignment(int task_location) {
    int region = get_region_id(task_location);
    if (region >= 0 && region < num_regions) {
        region_activity[region] += 1.0;
    }
}

// Get congestion penalty for a task location
double get_congestion_penalty(int task_location) {
    int region = get_region_id(task_location);
    if (region >= 0 && region < num_regions) {
        return CONGESTION_PENALTY * region_activity[region];
    }
    return 0.0;
}
}

void TaskScheduler::initialize(int preprocess_time_limit)
{
    int limit = preprocess_time_limit/2 - DefaultPlanner::SCHEDULER_TIMELIMIT_TOLERANCE;
    DefaultPlanner::schedule_initialize(limit, env);

    // Initialize spatial tracking for this map
    init_spatial_tracking(env->rows, env->cols);
}

void TaskScheduler::plan(int time_limit, std::vector<int> & proposed_schedule)
{
    TimePoint endtime = std::chrono::steady_clock::now() + std::chrono::milliseconds(time_limit);

    free_agents.insert(env->new_freeagents.begin(), env->new_freeagents.end());
    free_tasks.insert(env->new_tasks.begin(), env->new_tasks.end());

    // Decay regional activity at start of each planning round
    decay_regional_activity();

    std::unordered_set<int>::iterator it = free_agents.begin();
    while (it != free_agents.end())
    {
        if (std::chrono::steady_clock::now() > endtime)
            break;

        int i = *it;
        int min_task_i = -1;
        double min_effective_dist = 1e9;
        int min_task_id = INT_MAX;  // deterministic tie-break: smallest task ID
        int count = 0;

        for (int t_id : free_tasks)
        {
            if (count % 10 == 0 && std::chrono::steady_clock::now() > endtime)
                break;

            int dist = 0;
            int c_loc = env->curr_states.at(i).location;
            for (int loc : env->task_pool[t_id].locations)
            {
                dist += DefaultPlanner::get_h(env, c_loc, loc);
                c_loc = loc;
            }

            // Get the pickup location (first location in task)
            int pickup_loc = env->task_pool[t_id].locations.empty() ? 0 : env->task_pool[t_id].locations[0];

            // Calculate effective distance with congestion penalty
            double congestion_penalty = get_congestion_penalty(pickup_loc);
            double effective_dist = dist + congestion_penalty;

            if (effective_dist < min_effective_dist)
            {
                min_effective_dist = effective_dist;
                min_task_i = t_id;
                min_task_id = t_id;
            }
            else if (effective_dist == min_effective_dist)
            {
                // Deterministic tie-break: pick smallest task ID
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

            // Record assignment for spatial tracking
            int pickup_loc = env->task_pool[min_task_i].locations.empty() ? 0 : env->task_pool[min_task_i].locations[0];
            record_assignment(pickup_loc);

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
