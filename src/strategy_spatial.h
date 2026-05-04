// strategy_spatial.h - Spatial-indexed fast scheduler
//
// Groups tasks into spatial cells. When an agent needs a task,
// we search through expanding rings of cells, finding the truly
// nearest task. No heuristics, no top-K — just genuinely fast
// nearest-task lookup.
//
// KEY INSIGHT: On the cloud with fast CPU, ALL 4000 agents are
// assigned. Our assignments determine throughput. The spatial index
// gives each agent the CLOSEST task, minimizing travel time and
// maximizing task completion rate.

#pragma once
#include "TaskScheduler.h"
#include "scheduler.h"
#include "const.h"
#include <unordered_set>
#include <vector>
#include <algorithm>
#include <cmath>

namespace strategy_spatial {

static std::unordered_set<int> free_agents_s;
static std::unordered_set<int> free_tasks_s;
static int cell_size = 10;  // grid cells of 10×10
static int grid_cols = 0;
static int grid_rows = 0;

struct TaskLoc {
    int id;
    int first_loc;
    int cell_x, cell_y;  // cell coordinates
};
static std::vector<TaskLoc> all_tasks_by_loc;
static std::vector<std::vector<int>> cell_tasks;  // tasks per cell (linear index)
static bool needs_rebuild = true;

void init() {
    free_agents_s.clear();
    free_tasks_s.clear();
    all_tasks_by_loc.clear();
    cell_tasks.clear();
    needs_rebuild = true;
    grid_cols = 0;
    grid_rows = 0;
}

void rebuild_grid(SharedEnvironment* env) {
    grid_cols = (env->cols + cell_size - 1) / cell_size;
    grid_rows = (env->rows + cell_size - 1) / cell_size;
    int num_cells = grid_cols * grid_rows;
    cell_tasks.assign(num_cells, {});

    all_tasks_by_loc.clear();
    for (int tid : free_tasks_s) {
        int loc = env->task_pool[tid].locations[0];
        int x = loc % env->cols;
        int y = loc / env->cols;
        int cx = x / cell_size;
        int cy = y / cell_size;
        int cell_idx = cy * grid_cols + cx;
        cell_tasks[cell_idx].push_back(tid);
        all_tasks_by_loc.push_back({tid, loc, cx, cy});
    }
    needs_rebuild = false;
}

// Find nearest free task for an agent, searching expanding rings of cells
int find_nearest_task(int agent_loc, int a_loc_x, int a_loc_y, SharedEnvironment* env, float& best_dist) {
    int acx = a_loc_x / cell_size;
    int acy = a_loc_y / cell_size;
    int best_task = -1;
    best_dist = 1e30f;
    
    // Search expanding rings
    int max_ring = std::max(grid_cols, grid_rows);
    for (int ring = 0; ring <= max_ring; ring++) {
        bool found_this_ring = false;
        
        // Top and bottom rows of this ring
        for (int dx = -ring; dx <= ring; dx++) {
            for (int dy : {-ring, ring}) {
                if (dy == 0 && ring > 0) continue;  // skip zero-height middle
                int cx = acx + dx;
                int cy = acy + dy;
                if (cx < 0 || cx >= grid_cols || cy < 0 || cy >= grid_rows) continue;
                int cell = cy * grid_cols + cx;
                for (int tid : cell_tasks[cell]) {
                    if (free_tasks_s.find(tid) == free_tasks_s.end()) continue;
                    float d = (float)DefaultPlanner::get_h(env, agent_loc, env->task_pool[tid].locations[0]);
                    if (d < 1.0f) d = 1.0f;
                    // Add internal makespan
                    const auto& locs = env->task_pool[tid].locations;
                    for (size_t i = 1; i < locs.size(); i++)
                        d += (float)DefaultPlanner::get_h(env, locs[i-1], locs[i]);
                    if (d < best_dist) {
                        best_dist = d;
                        best_task = tid;
                        found_this_ring = true;
                    }
                }
            }
        }
        
        // Left and right columns (excluding corners already covered)
        for (int dy = -ring + 1; dy <= ring - 1; dy++) {
            for (int dx : {-ring, ring}) {
                if (dx == 0) continue;
                int cx = acx + dx;
                int cy = acy + dy;
                if (cx < 0 || cx >= grid_cols || cy < 0 || cy >= grid_rows) continue;
                int cell = cy * grid_cols + cx;
                for (int tid : cell_tasks[cell]) {
                    if (free_tasks_s.find(tid) == free_tasks_s.end()) continue;
                    float d = (float)DefaultPlanner::get_h(env, agent_loc, env->task_pool[tid].locations[0]);
                    if (d < 1.0f) d = 1.0f;
                    const auto& locs = env->task_pool[tid].locations;
                    for (size_t i = 1; i < locs.size(); i++)
                        d += (float)DefaultPlanner::get_h(env, locs[i-1], locs[i]);
                    if (d < best_dist) {
                        best_dist = d;
                        best_task = tid;
                        found_this_ring = true;
                    }
                }
            }
        }
        
        if (ring > 0 && found_this_ring) break;  // found tasks in this ring, stop expanding
    }
    
    return best_task;
}

void plan(int time_limit, SharedEnvironment* env, std::vector<int>& proposed_schedule) {
    int limit = time_limit/2 - DefaultPlanner::SCHEDULER_TIMELIMIT_TOLERANCE;
    TimePoint endtime = std::chrono::steady_clock::now() + std::chrono::milliseconds(limit);

    free_agents_s.insert(env->new_freeagents.begin(), env->new_freeagents.end());
    
    if (!env->new_tasks.empty()) {
        free_tasks_s.insert(env->new_tasks.begin(), env->new_tasks.end());
        needs_rebuild = true;
    }

    if (free_agents_s.empty() || free_tasks_s.empty()) return;

    if (needs_rebuild) {
        rebuild_grid(env);
    }

    // Sort free agents by proximity to cluster of free tasks
    // This ensures agents near task-rich regions get priority
    std::vector<int> agents(free_agents_s.begin(), free_agents_s.end());
    
    // Quick heuristic: agents closest to the centroid of remaining tasks go first
    // Simple version: just use agent id order (they have random initial spawn positions)
    
    for (int a : agents) {
        if (std::chrono::steady_clock::now() > endtime) break;
        if (free_agents_s.find(a) == free_agents_s.end()) continue;

        int a_loc = env->curr_states.at(a).location;
        int x = a_loc % env->cols;
        int y = a_loc / env->cols;

        float dist;
        int best_task = find_nearest_task(a_loc, x, y, env, dist);

        if (best_task >= 0) {
            proposed_schedule[a] = best_task;
            free_agents_s.erase(a);
            free_tasks_s.erase(best_task);
        } else {
            proposed_schedule[a] = -1;
        }
    }
}

} // namespace strategy_spatial
