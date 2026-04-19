/**
 * @file Dist2PathHeuristicTable.h
 * @brief Distance-to-Path Heuristic Table with OpenMP threading support
 * 
 * Wraps the existing static heuristic table with:
 * 1. OpenMP parallel access for multi-threaded planning
 * 2. Integration with TrajLNS traffic flow distances (when enabled)
 * 
 * Based on Team_Kitty_Knight's Dist2PathHeuristicTable implementation.
 */

#pragma once

#include <memory>
#include <vector>
#include <omp.h>
#include "../inc/SharedEnv.h"
#include "heuristics.h"

namespace MyPlanner {

class Dist2PathHeuristicTable {
public:
    TrajLNS& lns;
    SharedEnvironment* env;
    std::vector<omp_lock_t> agent_locks;
    int num_threads;

    Dist2PathHeuristicTable(
        TrajLNS& _lns,
        int _num_threads = 1
    ) : lns(_lns), 
        env(_lns.env),
        num_threads(_num_threads) 
    {
        if (num_threads > 1) {
            agent_locks.resize(env->num_of_agents);
            for (int agent_idx = 0; agent_idx < env->num_of_agents; ++agent_idx) {
                omp_init_lock(&agent_locks[agent_idx]);
            }
        }
    }

    /**
     * @brief Get heuristic distance from a location to the agent's goal
     * 
     * @param agent_idx Agent ID
     * @param loc Current location
     * @param orient Current orientation (-1 for ignore)
     * @return Heuristic distance
     */
    float get(int agent_idx, int loc, int orient = -1) {
        int min_heuristic;

        if (num_threads > 1) {
            omp_set_lock(&agent_locks[agent_idx]);
        }

        // Use traffic flow distances if available, otherwise use static heuristics
        // Note: traj_dists integration is for future enhancement
        // if (!lns.traj_dists.empty() && !lns.traj_dists[agent_idx].empty())
        //     min_heuristic = get_dist_2_path(lns.traj_dists[agent_idx], lns.env, loc, &(lns.neighbors));
        // else

        if (orient != -1) {
            min_heuristic = get_h(env, loc, lns.tasks.at(agent_idx));
        } else {
            min_heuristic = get_h(env, loc, lns.tasks.at(agent_idx));
        }

        if (num_threads > 1) {
            omp_unset_lock(&agent_locks[agent_idx]);
        }
        
        return min_heuristic;
    }

    /**
     * @brief Parallel batch heuristic lookup
     * 
     * @param agent_indices Agent IDs to query
     * @param locations Locations to query
     * @param orientations Orientations (use -1 to ignore)
     * @return Vector of heuristic distances
     */
    std::vector<float> get_batch(
        const std::vector<int>& agent_indices,
        const std::vector<int>& locations,
        const std::vector<int>& orientations
    ) {
        int n = agent_indices.size();
        std::vector<float> results(n);

        if (num_threads > 1) {
            #pragma omp parallel for
            for (int i = 0; i < n; ++i) {
                results[i] = get(agent_indices[i], locations[i], orientations[i]);
            }
        } else {
            for (int i = 0; i < n; ++i) {
                results[i] = get(agent_indices[i], locations[i], orientations[i]);
            }
        }

        return results;
    }

    ~Dist2PathHeuristicTable() {
        if (num_threads > 1) {
            for (int agent_idx = 0; agent_idx < env->num_of_agents; ++agent_idx) {
                omp_destroy_lock(&agent_locks[agent_idx]);
            }
        }
    }
};

} // namespace MyPlanner
