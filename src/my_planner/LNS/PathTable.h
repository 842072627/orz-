#pragma once
#include <vector>
#include <set>
#include "../Types.h"

#ifndef MAX_TIMESTEP
#define MAX_TIMESTEP 10000
#endif

#define NO_AGENT -1

namespace LNS {

// Minimal PathTable for O(1) conflict detection
// table[location][timestep] = agent_id who occupies that location at that time
class PathTable
{
public:
    int makespan = 0;
    int window_size = -1;
    std::vector< std::vector<int> > table; // [location][time] = agent_id
    std::vector<int> goals; // goal location -> timestep when agent reaches goal

    PathTable() = default;
    explicit PathTable(int map_size, int window_size = -1) : 
        table(map_size), 
        goals(map_size, MAX_TIMESTEP), 
        window_size(window_size) {}

    void reset() {
        for (auto& row : table) {
            std::fill(row.begin(), row.end(), NO_AGENT);
        }
        std::fill(goals.begin(), goals.end(), MAX_TIMESTEP);
        makespan = 0;
    }

    void resize(int map_size, int new_window_size = -1) {
        table.assign(map_size, std::vector<int>(new_window_size + 1, NO_AGENT));
        goals.assign(map_size, MAX_TIMESTEP);
        window_size = new_window_size;
    }

    // Insert a path for an agent
    // Path format: vector of (location, orientation) pairs
    void insertPath(int agent_id, const std::vector<std::pair<int, int>>& path) {
        int t = 0;
        for (const auto& node : path) {
            int loc = node.first;
            if (loc < 0 || loc >= (int)table.size()) continue;
            if (t >= (int)table[loc].size()) {
                table[loc].resize(t + 1, NO_AGENT);
            }
            table[loc][t] = agent_id;
            t++;
        }
        makespan = std::max(makespan, t - 1);
    }

    // Check if a location at a timestep is occupied by another agent
    bool constrained(int location, int timestep) const {
        if (location < 0 || location >= (int)table.size()) return false;
        if (timestep < 0 || timestep >= (int)table[location].size()) return false;
        return table[location][timestep] != NO_AGENT;
    }

    // Check if edge (from -> to) at timestep is occupied
    bool edgeConstrained(int from, int to, int timestep) const {
        // For edge conflicts, check if the target location at next timestep
        // is occupied by an agent that came from a different location
        if (to < 0 || to >= (int)table.size()) return false;
        if (timestep < 0 || timestep >= (int)table[to].size()) return false;
        return table[to][timestep] != NO_AGENT;
    }

    // Get all agents that conflict with a given location at a timestep
    void getConflictingAgents(int location, int timestep, std::set<int>& conflicting_agents) const {
        if (location < 0 || location >= (int)table.size()) return;
        if (timestep < 0 || timestep >= (int)table[location].size()) return;
        int agent_id = table[location][timestep];
        if (agent_id != NO_AGENT) {
            conflicting_agents.insert(agent_id);
        }
    }

    // Get holding time: when does the location become free starting from earliest_timestep
    int getHoldingTime(int location, int earliest_timestep) const {
        if (location < 0 || location >= (int)table.size()) return earliest_timestep;
        for (size_t t = earliest_timestep; t < table[location].size(); ++t) {
            if (table[location][t] == NO_AGENT) {
                return (int)t;
            }
        }
        return MAX_TIMESTEP;
    }
};

} // namespace LNS
