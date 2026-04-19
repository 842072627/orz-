#pragma once

#include <vector>
#include <algorithm>

#define NO_AGENT -1
#define MAX_TIMESTEP 10000

namespace LNS {

// Simple PathTable that tracks agent positions at each timestep
class PathTable {
public:
    int map_size;
    int window_size;
    std::vector<std::vector<int>> table;  // table[location][timestep] = agent_id or NO_AGENT
    std::vector<int> goals;  // goal[location] = timestep when agent reaches goal (or MAX_TIMESTEP if not yet)
    int makespan;

    PathTable(int map_size = 0, int window_size = -1) 
        : map_size(map_size), window_size(window_size), table(map_size), goals(map_size, MAX_TIMESTEP), makespan(0) {
        for (int i = 0; i < map_size; i++) {
            table[i].reserve(100);  // Pre-allocate some space
        }
    }

    void reset() {
        for (auto& col : table) {
            col.clear();
        }
        std::fill(goals.begin(), goals.end(), MAX_TIMESTEP);
        makespan = 0;
    }

    // Insert a path into the table
    // Path is a vector of locations (one per timestep starting from current_timestep)
    void insertPath(int agent_id, const std::vector<int>& path, int current_timestep = 0) {
        if (path.empty()) return;

        int T = (int)path.size();
        if (window_size > 0 && window_size + 1 < T) {
            T = window_size + 1;
        }

        for (int t = 0; t < T; t++) {
            int loc = path[t];
            int absolute_t = current_timestep + t;
            
            // Ensure table[loc] is large enough
            if ((int)table[loc].size() <= absolute_t) {
                table[loc].resize(absolute_t + 1, NO_AGENT);
            }
            table[loc][absolute_t] = agent_id;
        }

        // Update goal info
        if (!path.empty()) {
            int goal_loc = path.back();
            if (goal_loc >= 0 && goal_loc < map_size) {
                int arrival_time = current_timestep + path.size() - 1;
                if (goals[goal_loc] == MAX_TIMESTEP || arrival_time < goals[goal_loc]) {
                    goals[goal_loc] = arrival_time;
                }
            }
        }

        makespan = std::max(makespan, (int)path.size() - 1);
    }

    // Remove a path from the table
    void deletePath(int agent_id, const std::vector<int>& path, int current_timestep = 0) {
        if (path.empty()) return;

        int T = (int)path.size();
        if (window_size > 0 && window_size + 1 < T) {
            T = window_size + 1;
        }

        for (int t = 0; t < T; t++) {
            int loc = path[t];
            int absolute_t = current_timestep + t;
            
            if (absolute_t < (int)table[loc].size() && table[loc][absolute_t] == agent_id) {
                table[loc][absolute_t] = NO_AGENT;
            }
        }
    }

    // Check if a move from 'from' to 'to' at timestep 'to_time' would collide
    // Returns true if the move is constrained (would cause collision)
    bool constrained(int from, int to, int to_time) const {
        if (to < 0 || to >= map_size) return true;
        
        // Vertex conflict: is there an agent at 'to' at time 'to_time'?
        if ((int)table[to].size() > to_time && table[to][to_time] != NO_AGENT) {
            return true;  // Vertex collision
        }

        // Edge conflict: is there an agent at 'from' at time 'to_time' that's moving to 'to' at time 'to_time-1'?
        // (i.e., swapping positions)
        if (to_time > 0 && (int)table[to].size() > to_time - 1 && !table[to].empty() &&
            table[to][to_time - 1] != NO_AGENT && 
            (int)table[from].size() > to_time && table[from][to_time] != NO_AGENT &&
            table[from][to_time] == table[to][to_time - 1]) {
            return true;  // Edge collision (swap)
        }

        return false;
    }

    // Check if a location is occupied at a given timestep
    bool occupied(int location, int timestep) const {
        if (location < 0 || location >= map_size) return true;
        if ((int)table[location].size() <= timestep) return false;  // Not yet occupied
        return table[location][timestep] != NO_AGENT;
    }

    // Get agent ID at a location and timestep
    int getAgentAt(int location, int timestep) const {
        if (location < 0 || location >= map_size) return NO_AGENT;
        if ((int)table[location].size() <= timestep) return NO_AGENT;
        return table[location][timestep];
    }
};

}  // namespace LNS
