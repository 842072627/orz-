#pragma once
#include <vector>
#include <limits>

namespace DefaultPlanner {

// Simple ReservationTable for SIPP-like time conflict detection
// Tracks which agent occupies which location at which timestep
class ReservationTable {
public:
    int map_size;
    int max_timestep;
    std::vector<std::vector<int>> table; // [location][timestep] = agent_id or -1
    
    ReservationTable() : map_size(0), max_timestep(0) {}
    
    ReservationTable(int map_size, int max_timestep) 
        : map_size(map_size), max_timestep(max_timestep) {
        table.assign(map_size, std::vector<int>(max_timestep + 1, -1));
    }
    
    void resize(int new_map_size, int new_max_timestep) {
        map_size = new_map_size;
        max_timestep = new_max_timestep;
        table.assign(map_size, std::vector<int>(max_timestep + 1, -1));
    }
    
    // Reserve a location at a timestep for an agent
    void reserve(int location, int timestep, int agent_id) {
        if (location < 0 || location >= map_size) return;
        if (timestep < 0 || timestep > max_timestep) {
            // Extend table if needed
            if (timestep > max_timestep) {
                for (auto& row : table) {
                    row.resize(timestep + 1, -1);
                }
                max_timestep = timestep;
            } else {
                return;
            }
        }
        table[location][timestep] = agent_id;
    }
    
    // Check if a location at a timestep is reserved by another agent
    bool is_reserved(int location, int timestep, int exclude_agent = -1) {
        if (location < 0 || location >= map_size) return false;
        if (timestep < 0) return false;
        if (timestep > max_timestep) return false; // Future timesteps are free
        int agent = table[location][timestep];
        return agent != -1 && agent != exclude_agent;
    }
    
    // Get the earliest timestep when a location becomes free starting from earliest_timestep
    int get_holding_time(int location, int earliest_timestep) {
        if (location < 0 || location >= map_size) return earliest_timestep;
        if (earliest_timestep > max_timestep) return earliest_timestep;
        
        for (int t = earliest_timestep; t <= max_timestep; t++) {
            if (table[location][t] == -1) {
                return t;
            }
        }
        return max_timestep + 1;
    }
    
    // Insert a path into the reservation table
    void insert_path(int agent_id, const std::vector<int>& path) {
        for (int t = 0; t < (int)path.size(); t++) {
            reserve(path[t], t, agent_id);
        }
    }
    
    // Clear all reservations
    void clear() {
        for (auto& row : table) {
            std::fill(row.begin(), row.end(), -1);
        }
    }
};

} // namespace DefaultPlanner
