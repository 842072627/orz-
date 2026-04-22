#pragma once
#include <vector>
#include <unordered_set>
#include <queue>
#include <chrono>

namespace DefaultPlanner {

// SIPP Node for time-extended search
struct SIPPNode {
    int location;      // Current location
    int orientation;   // Current orientation
    int timestep;     // Current timestep
    int g;           // Cost so far
    int h;           // Heuristic
    int f() const { return g + h; }
    SIPPNode* parent; // Parent node for path reconstruction
    
    SIPPNode(int loc, int orient, int t, int g, int h, SIPPNode* parent = nullptr)
        : location(loc), orientation(orient), timestep(t), g(g), h(h), parent(parent) {}
};

// Priority queue comparator for SIPP
struct SIPPComparator {
    bool operator()(const SIPPNode* a, const SIPPNode* b) const {
        if (a->f() == b->f()) {
            return a->g < b->g; // Prefer higher g (closer to goal)
        }
        return a->f() > b->f(); // Min f
    }
};

// ReservationTable for SIPP
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
    
    void reset() {
        for (auto& row : table) {
            std::fill(row.begin(), row.end(), -1);
        }
    }
    
    void ensure_capacity(int location, int timestep) {
        if (timestep < 0) return;
        if (location < 0 || location >= map_size) return;
        
        if (timestep > max_timestep) {
            for (auto& row : table) {
                row.resize(timestep + 1, -1);
            }
            max_timestep = timestep;
        }
    }
    
    // Reserve a location at a timestep for an agent
    void reserve(int location, int timestep, int agent_id) {
        if (location < 0 || location >= map_size) return;
        if (timestep < 0) return;
        ensure_capacity(location, timestep);
        table[location][timestep] = agent_id;
    }
    
    // Check if a location at a timestep is reserved by another agent
    bool is_reserved(int location, int timestep, int exclude_agent = -1) const {
        if (location < 0 || location >= map_size) return false;
        if (timestep < 0) return false;
        if (timestep > max_timestep) return false; // Future timesteps are free
        int agent = table[location][timestep];
        return agent != -1 && agent != exclude_agent;
    }
    
    // Check if edge (from -> to) at timestep is reserved
    bool is_edge_reserved(int from, int to, int timestep, int agent_id = -1) const {
        // Edge conflict: another agent at 'to' at 'timestep'
        return is_reserved(to, timestep, agent_id);
    }
    
    // Get the earliest timestep when a location becomes free
    int get_holding_time(int location, int earliest_timestep) const {
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
    void insert_path(int agent_id, const std::vector<int>& path, int start_timestep = 0) {
        for (int t = 0; t < (int)path.size(); t++) {
            reserve(path[t], start_timestep + t, agent_id);
        }
    }
};

} // namespace DefaultPlanner
