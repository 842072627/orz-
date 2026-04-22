#pragma once
#include <vector>
#include <algorithm>
#include <limits>

namespace DefaultPlanner {

// PathTable for O(1) conflict detection
// Tracks which agent occupies which location at which timestep
// This is a simplified version for integration into TrajLNS
class PathTable {
public:
    int map_size;
    int window_size;
    std::vector<std::vector<int>> table; // [location][timestep] = agent_id or -1
    
    PathTable() : map_size(0), window_size(0) {}
    
    PathTable(int map_size, int window_size) 
        : map_size(map_size), window_size(window_size) {
        table.assign(map_size, std::vector<int>(window_size + 1, -1));
    }
    
    void resize(int new_map_size, int new_window_size) {
        map_size = new_map_size;
        window_size = new_window_size;
        table.assign(map_size, std::vector<int>(window_size + 1, -1));
    }
    
    void reset() {
        for (auto& row : table) {
            std::fill(row.begin(), row.end(), -1);
        }
    }
    
    // Reserve a location at a timestep for an agent
    void reserve(int location, int timestep, int agent_id) {
        if (location < 0 || location >= map_size) return;
        if (timestep < 0) return;
        
        // Extend table if needed
        if (timestep >= (int)table[location].size()) {
            table[location].resize(timestep + 1, -1);
            if (timestep > window_size) window_size = timestep;
        }
        table[location][timestep] = agent_id;
    }
    
    // Check if a location at a timestep is reserved by another agent
    bool constrained(int location, int timestep) const {
        if (location < 0 || location >= map_size) return false;
        if (timestep < 0) return false;
        if (timestep >= (int)table[location].size()) return false;
        return table[location][timestep] != -1;
    }
    
    // Check if a location at a timestep is reserved by a specific agent
    bool constrained(int location, int timestep, int exclude_agent) const {
        if (location < 0 || location >= map_size) return false;
        if (timestep < 0) return false;
        if (timestep >= (int)table[location].size()) return false;
        int agent = table[location][timestep];
        return agent != -1 && agent != exclude_agent;
    }
    
    // Get the holding time: earliest timestep when location becomes free
    int getHoldingTime(int location, int earliest_timestep) const {
        if (location < 0 || location >= map_size) return earliest_timestep;
        if (earliest_timestep >= (int)table[location].size()) return earliest_timestep;
        
        for (int t = earliest_timestep; t < (int)table[location].size(); t++) {
            if (table[location][t] == -1) {
                return t;
            }
        }
        return table[location].size();
    }
    
    // Insert a path for an agent
    void insertPath(int agent_id, const std::vector<int>& path, int start_timestep = 0) {
        for (int t = 0; t < (int)path.size(); t++) {
            reserve(path[t], start_timestep + t, agent_id);
        }
    }
    
    // Insert a path from Traj (vector of locations)
    void insertTraj(int agent_id, const Traj& traj, int start_timestep = 0) {
        for (int t = 0; t < (int)traj.size(); t++) {
            reserve(traj[t], start_timestep + t, agent_id);
        }
    }
};

} // namespace DefaultPlanner
