// Default scheduler baseline implementation.
//
// Keeps persistent sets of free agents and free tasks across timesteps, then
// greedily matches each free agent to the task with minimum heuristic distance
// makespan. This is intentionally simple and serves as a reference scheduler.
//
// Added: zone congestion penalty + reverse traffic (对冲) penalty for ORZ map.
// ORZ map: all 1800 agents start in rows 0-2 (northern corridor).
//          all task goals at row 0.
// Problem: agents going DOWN (corridor → below) collide head-on with
//          agents going UP (below → row 0) in the narrow corridor.
// Fix: penalize assigning DOWN tasks when too many DOWN agents already exist.

#include "my_scheduler.h"

namespace MyPlanner{

std::mt19937 mt;
std::unordered_set<int> free_agents;
std::unordered_set<int> free_tasks;

// Zone congestion penalty: spread agents across different first-location zones
static std::vector<int> zone_agent_count;
static constexpr int ZONE_SIZE = 24;
static constexpr int CONGESTION_PENALTY = 50000;
static int num_zones_x = 0;
static int num_zones_y = 0;

// Reverse traffic (对冲) penalty: avoid too many agents going DOWN from corridor
static constexpr int CORRIDOR_ROW = 3;    // rows 0-2 = narrow corridor
static constexpr int REVERSE_THRESHOLD = 5;   // penalty kicks in above this many DOWN agents
static constexpr int REVERSE_PENALTY = 200000;  // heavy penalty per excess DOWN agent

static int get_zone_id(int loc, int cols) {
    int x = loc % cols;
    int y = loc / cols;
    int zx = x / ZONE_SIZE;
    int zy = y / ZONE_SIZE;
    if (num_zones_x == 0 || num_zones_y == 0) return 0;
    zx = std::min(zx, num_zones_x - 1);
    zy = std::min(zy, num_zones_y - 1);
    return zy * num_zones_x + zx;
}

void schedule_initialize(int preprocess_time_limit, SharedEnvironment* env)
{
    mt.seed(0);
    
    // Initialize zone grid
    num_zones_x = (env->cols + ZONE_SIZE - 1) / ZONE_SIZE;
    num_zones_y = (env->rows + ZONE_SIZE - 1) / ZONE_SIZE;
    zone_agent_count.assign(num_zones_x * num_zones_y, 0);
    return;
}

void schedule_plan(int time_limit, std::vector<int> & proposed_schedule,  SharedEnvironment* env)
{
    TimePoint endtime = std::chrono::steady_clock::now() + std::chrono::milliseconds(time_limit);

    // Track free agents & tasks
    free_agents.insert(env->new_freeagents.begin(), env->new_freeagents.end());
    free_tasks.insert(env->new_tasks.begin(), env->new_tasks.end());
    
    // --- Phase 1: Compute congestion state ---
    std::fill(zone_agent_count.begin(), zone_agent_count.end(), 0);
    int down_flow = 0;  // agents in corridor heading DOWN to get new tasks
    int corridor_cols = env->cols;
    
    for (int ai = 0; ai < env->num_of_agents; ai++) {
        if (env->curr_task_schedule[ai] != -1) {
            int task_id = env->curr_task_schedule[ai];
            if (task_id >= 0 && task_id < (int)env->task_pool.size() && !env->task_pool[task_id].locations.empty()) {
                // Zone penalty: first task location
                int first_loc = env->task_pool[task_id].locations.front();
                int z = get_zone_id(first_loc, corridor_cols);
                if (z >= 0 && z < (int)zone_agent_count.size()) zone_agent_count[z]++;
                
                // Direction penalty: agent in corridor + task below = going DOWN
                int agent_loc = env->curr_states.at(ai).location;
                int agent_row = agent_loc / corridor_cols;
                if (agent_row < CORRIDOR_ROW) {
                    down_flow++;  // this agent is in corridor, heading down
                }
            }
        }
    }

    int min_task_i, min_task_makespan, dist, c_loc, count;

    // iterate over the free agents to decide which task to assign to each of them
    std::unordered_set<int>::iterator it = free_agents.begin();
    while (it != free_agents.end())
    {
        if (std::chrono::steady_clock::now() > endtime) break;
        
        int i = *it;
        assert(env->curr_task_schedule[i] == -1);
            
        min_task_i = -1;
        min_task_makespan = INT_MAX;
        count = 0;

        int agent_loc = env->curr_states.at(i).location;
        int agent_row = agent_loc / corridor_cols;
        bool agent_in_corridor = (agent_row < CORRIDOR_ROW);

        // iterate over all the unassigned tasks
        for (int t_id : free_tasks)
        {
            if (count % 10 == 0 && std::chrono::steady_clock::now() > endtime) break;
            
            dist = 0;
            c_loc = agent_loc;

            // Compute makespan to finish task
            for (int loc : env->task_pool[t_id].locations){
                dist += MyPlanner::get_h(env, c_loc, loc);
                c_loc = loc;
            }
            
            // --- Zone congestion penalty ---
            if (!env->task_pool[t_id].locations.empty()) {
                int first_loc = env->task_pool[t_id].locations.front();
                int z = get_zone_id(first_loc, corridor_cols);
                int zone_cnt = (z >= 0 && z < (int)zone_agent_count.size()) ? zone_agent_count[z] : 0;
                dist += CONGESTION_PENALTY * zone_cnt;
            }
            
            // --- Reverse traffic (对冲) penalty ---
            // If agent is in corridor and task first location is below corridor,
            // this agent will go DOWN. Penalize if too many are already going DOWN.
            if (agent_in_corridor) {
                int task_first_row = env->task_pool[t_id].locations.empty() ? 0 
                    : (env->task_pool[t_id].locations.front() / corridor_cols);
                if (task_first_row >= CORRIDOR_ROW) {
                    // This agent in corridor + task below = DOWN direction
                    // down_flow counts agents already going down (incl some from earlier in this loop)
                    int effective_down = down_flow;  // already updated from existing assignments
                    if (effective_down > REVERSE_THRESHOLD) {
                        dist += REVERSE_PENALTY * (effective_down - REVERSE_THRESHOLD);
                    }
                }
            }
            // If agent is BELOW corridor, it will naturally return UP. No penalty needed
            // because UP is the "return" direction and is necessary.

            if (dist < min_task_makespan){
                min_task_i = t_id;
                min_task_makespan = dist;
            }
            count++;            
        }

        // assign the best free task
        if (min_task_i != -1){
            proposed_schedule[i] = min_task_i;
            it = free_agents.erase(it);
            free_tasks.erase(min_task_i);
            
            // Update zone congestion count
            if (!env->task_pool[min_task_i].locations.empty()) {
                int first_loc = env->task_pool[min_task_i].locations.front();
                int z = get_zone_id(first_loc, corridor_cols);
                if (z >= 0 && z < (int)zone_agent_count.size()) zone_agent_count[z]++;
            }
            
            // Update down_flow for earlier agents in this loop
            if (agent_in_corridor) {
                int task_first_row = env->task_pool[min_task_i].locations.empty() ? 0 
                    : (env->task_pool[min_task_i].locations.front() / corridor_cols);
                if (task_first_row >= CORRIDOR_ROW) {
                    down_flow++;  // committed another down agent
                }
            }
        }
        else{
            proposed_schedule[i] = -1;
            it++;
        }
    }
    return;
}
}
