#pragma once
#include <vector>
#include <string>
#include "../../inc/SharedEnv.h"

namespace LNS {

// Wrapper around SharedEnvironment to provide LNS interface
class Instance
{
public:
    int num_of_agents;
    int num_of_rows;
    int num_of_cols;
    int map_size;
    std::vector<int> my_map;  // true = obstacle
    
    std::vector<int> start_locations;
    std::vector<int> start_orientations;
    std::vector<int> goal_locations;
    
    // Constructor from SharedEnvironment
    Instance() = default;
    
    Instance(const SharedEnvironment& env) {
        num_of_agents = env.num_of_agents;
        num_of_rows = env.rows;
        num_of_cols = env.cols;
        map_size = env.map.size();
        my_map = env.map;
        
        start_locations.resize(num_of_agents);
        start_orientations.resize(num_of_agents);
        goal_locations.resize(num_of_agents);
        
        for (int i = 0; i < num_of_agents; ++i) {
            start_locations[i] = env.curr_states[i].location;
            start_orientations[i] = env.curr_states[i].orientation;
        }
        
        // Goal locations from env.goal_locations (may be empty for some agents)
        for (int i = 0; i < num_of_agents; ++i) {
            if (!env.goal_locations[i].empty()) {
                goal_locations[i] = env.goal_locations[i].front().first;
            } else {
                goal_locations[i] = start_locations[i];  // Dummy goal
            }
        }
    }
    
    // Set starts and goals from vectors
    void set_starts_and_goals(
        const std::vector<State>& starts,
        const std::vector<int>& goals
    ) {
        num_of_agents = starts.size();
        start_locations.resize(num_of_agents);
        start_orientations.resize(num_of_agents);
        goal_locations = goals;
        
        for (int i = 0; i < num_of_agents; ++i) {
            start_locations[i] = starts[i].location;
            start_orientations[i] = starts[i].orientation;
        }
    }
    
    // Get neighbors of a location
    std::list<int> getNeighbors(int loc) const {
        std::list<int> neighbors;
        
        int row = loc / num_of_cols;
        int col = loc % num_of_cols;
        
        // North (row - 1)
        if (row > 0 && !my_map[loc - num_of_cols]) {
            neighbors.push_back(loc - num_of_cols);
        }
        // South (row + 1)
        if (row < num_of_rows - 1 && !my_map[loc + num_of_cols]) {
            neighbors.push_back(loc + num_of_cols);
        }
        // West (col - 1)
        if (col > 0 && !my_map[loc - 1]) {
            neighbors.push_back(loc - 1);
        }
        // East (col + 1)
        if (col < num_of_cols - 1 && !my_map[loc + 1]) {
            neighbors.push_back(loc + 1);
        }
        
        return neighbors;
    }
    
    // Check if move is valid
    inline bool validMove(int curr, int next) const {
        if (next < 0 || next >= map_size) return false;
        if (my_map[next]) return false;
        return getManhattanDistance(curr, next) < 2;
    }
    
    // Manhattan distance
    inline int getManhattanDistance(int loc1, int loc2) const {
        int row1 = loc1 / num_of_cols;
        int col1 = loc1 % num_of_cols;
        int row2 = loc2 / num_of_cols;
        int col2 = loc2 % num_of_cols;
        return abs(row1 - row2) + abs(col1 - col2);
    }
    
    // Get row/col from location
    inline int getRowCoordinate(int id) const { return id / num_of_cols; }
    inline int getColCoordinate(int id) const { return id % num_of_cols; }
    inline std::pair<int, int> getCoordinate(int id) const { 
        return {getRowCoordinate(id), getColCoordinate(id)};
    }
    
    // Check if obstacle
    inline bool isObstacle(int loc) const { return my_map[loc] != 0; }
    
    // Random walk
    int randomWalk(int loc, int steps) const {
        std::mt19937 mt(0);
        std::uniform_int_distribution<int> dist(0, 3);
        
        for (int i = 0; i < steps; ++i) {
            auto neighbors = getNeighbors(loc);
            if (neighbors.empty()) break;
            
            std::vector<int> neighbor_list(neighbors.begin(), neighbors.end());
            std::uniform_int_distribution<int> neighbor_dist(0, (int)neighbor_list.size() - 1);
            loc = neighbor_list[neighbor_dist(mt)];
        }
        return loc;
    }
};

} // namespace LNS
