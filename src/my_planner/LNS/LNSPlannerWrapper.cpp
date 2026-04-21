// LNS Planner Wrapper - For integration testing
// This provides a simple interface to use LNS as an alternative planner

#include "LNS/GlobalManager.h"
#include "../../inc/SharedEnv.h"
#include <iostream>
#include <vector>

// Forward declaration
namespace LNS {
    class GlobalManager;
}

// Static LNS manager (for simplicity - lives for duration of program)
static std::shared_ptr<LNS::GlobalManager> g_lns_manager = nullptr;

// Initialize LNS with environment
extern "C" void init_lns_planner(SharedEnvironment* env) {
    if (env == nullptr || env->num_of_agents <= 0) return;
    
    std::vector<int> start_locs(env->num_of_agents);
    std::vector<int> start_orients(env->num_of_agents);
    std::vector<int> goal_locs(env->num_of_agents);
    
    for (int i = 0; i < env->num_of_agents; ++i) {
        start_locs[i] = env->curr_states[i].location;
        start_orients[i] = env->curr_states[i].orientation;
        if (!env->goal_locations[i].empty()) {
            goal_locs[i] = env->goal_locations[i].front().first;
        } else {
            goal_locs[i] = start_locs[i];  // Dummy goal
        }
    }
    
    // Create LNS manager
    g_lns_manager = std::make_shared<LNS::GlobalManager>(
        env->map,
        env->rows,
        env->cols,
        env->num_of_agents,
        start_locs,
        start_orients,
        goal_locs,
        10,   // window_size
        8     // neighbor_size
    );
    
    std::cout << "[LNS] Initialized with " << env->num_of_agents << " agents" << std::endl;
}

// Plan using LNS
extern "C" void plan_with_lns(
    SharedEnvironment* env,
    std::vector<int>& out_actions,
    int time_limit_ms
) {
    if (!g_lns_manager) {
        std::cerr << "[LNS] ERROR: LNS not initialized!" << std::endl;
        out_actions.assign(env->num_of_agents, 3);  // All wait
        return;
    }
    
    // Update LNS with current state
    std::vector<int> curr_locs(env->num_of_agents);
    std::vector<int> curr_orients(env->num_of_agents);
    std::vector<int> goal_locs(env->num_of_agents);
    
    for (int i = 0; i < env->num_of_agents; ++i) {
        curr_locs[i] = env->curr_states[i].location;
        curr_orients[i] = env->curr_states[i].orientation;
        if (!env->goal_locations[i].empty()) {
            goal_locs[i] = env->goal_locations[i].front().first;
        } else {
            goal_locs[i] = curr_locs[i];
        }
    }
    
    g_lns_manager->update(curr_locs, curr_orients, goal_locs);
    
    // Run LNS
    auto actions = g_lns_manager->run(time_limit_ms);
    
    out_actions = actions;
}

// Check if LNS is initialized
extern "C" bool is_lns_initialized() {
    return g_lns_manager != nullptr;
}
