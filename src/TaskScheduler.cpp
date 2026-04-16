#include "TaskScheduler.h"

#include "scheduler.h"
#include "const.h"

// ============================================================
// ARCH-K R1: Completion-Time-Aware Scheduling
//
// Instead of LENGTH_WEIGHT which penalizes long tasks,
// use estimated completion time as the primary metric.
// This balances workload by considering when each agent will free up.
//
// Formula: completion_time = distance_to_pickup + task_steps + distance_to_final
// Then add congestion penalty
//
// This should better distribute work across agents.
// ============================================================

namespace {
std::unordered_set<int> free_agents;
std::unordered_set<int> free_tasks;

constexpr int REGION_GRID_SIZE = 16;
constexpr double PICKUP_PENALTY = 7.3;
constexpr double DEST_PENALTY = 0.8;
constexpr double DECAY_FACTOR = 0.95;
// No LENGTH_WEIGHT - use completion time instead

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

double get_task_congestion_penalty(int pickup_loc, int dest_loc) {
    int pickup_region = get_region_id(pickup_loc);
    int dest_region = get_region_id(dest_loc);
    double penalty = 0.0;
    if (pickup_region >= 0 && pickup_region < num_regions) {
        penalty += PICKUP_PENALTY * region_activity[pickup_region];
    }
    if (dest_region >= 0 && dest_region < num_regions) {
        penalty += DEST_PENALTY * region_activity[dest_region];
    }
    return penalty;
}

// Compute completion time estimate for an agent-task pair
// = distance to pickup + task length (num steps) + distance to final
double estimate_completion_time(SharedEnvironment* e, int agent_id, int task_id) {
    int c_loc = e->curr_states.at(agent_id).location;
    const auto& task_locs = e->task_pool[task_id].locations;
    
    if (task_locs.empty()) return 0.0;
    
    // Distance from agent to first pickup
    int dist_to_pickup = DefaultPlanner::get_h(e, c_loc, task_locs[0]);
    
    // Task length (number of waypoints to visit)
    int task_length = task_locs.size();
    
    // Distance from last waypoint to... nowhere (task ends there)
    // Actually, the last loc is already the final destination
    int dist_within_task = 0;
    int curr = task_locs[0];
    for (size_t i = 1; i < task_locs.size(); i++) {
        dist_within_task += DefaultPlanner::get_h(e, curr, task_locs[i]);
        curr = task_locs[i];
    }
    
    // Total: travel to pickup + task traversal
    // Plus small factor for task complexity (prefer simpler tasks when congested)
    double complexity_factor = task_length * 0.3; // Smaller than original LENGTH_WEIGHT=0.5
    return dist_to_pickup + dist_within_task + complexity_factor;
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

    std::unordered_set<int>::iterator it = free_agents.begin();
    while (it != free_agents.end())
    {
        if (std::chrono::steady_clock::now() > endtime)
            break;

        int i = *it;
        int min_task_i = -1;
        double min_value = 1e9;
        int min_task_id = INT_MAX;
        int count = 0;

        for (int t_id : free_tasks)
        {
            if (count % 10 == 0 && std::chrono::steady_clock::now() > endtime)
                break;

            // Calculate completion time instead of distance
            double completion_time = estimate_completion_time(env, i, t_id);
            
            const auto& task_locs = env->task_pool[t_id].locations;
            int pickup_loc = task_locs.empty() ? 0 : task_locs[0];
            int dest_loc = task_locs.empty() ? 0 : task_locs.back();
            
            // Add congestion penalty
            double congestion_penalty = get_task_congestion_penalty(pickup_loc, dest_loc);
            
            // Total value: completion time + congestion
            double total_value = completion_time + congestion_penalty;

            if (total_value < min_value)
            {
                min_value = total_value;
                min_task_i = t_id;
                min_task_id = t_id;
            }
            else if (total_value == min_value)
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
