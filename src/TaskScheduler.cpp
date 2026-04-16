#include "TaskScheduler.h"

#include "scheduler.h"
#include "const.h"

// ============================================================
// ARCH-E R3: Higher Pickup + Lower Dest (pickup=7.5, dest=0.8)
// R1 multiplicative penalty FAILED (fulfill dropped to 0.666)
// R2 tries: dest=0.8 (lower than R5's 1.0) to help agents complete destinations faster
// ============================================================

namespace {
std::unordered_set<int> free_agents;
std::unordered_set<int> free_tasks;

// ARCH-E: Spatial load balancing state
constexpr int REGION_GRID_SIZE = 16;
constexpr double PICKUP_PENALTY = 7.5;  // Slightly higher than R5's 7.0
constexpr double DEST_PENALTY = 0.8;     // Lower than R5's 1.0
constexpr double DECAY_FACTOR = 0.95;    // Keep best

// Regional activity tracking
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
            for (int loc : task_locs)
            {
                dist += DefaultPlanner::get_h(env, c_loc, loc);
                c_loc = loc;
            }

            int pickup_loc = task_locs.empty() ? 0 : task_locs[0];
            int dest_loc = task_locs.empty() ? 0 : task_locs.back();
            double congestion_penalty = get_task_congestion_penalty(pickup_loc, dest_loc);
            double effective_dist = dist + congestion_penalty;

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
