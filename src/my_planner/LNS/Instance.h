#pragma once
#include <vector>
#include <string>
#include "../../Types.h"

namespace LNS {

class Instance {
public:
    int num_of_cols;
    int num_of_rows;
    int map_size;
    std::vector<char> my_map;
    std::vector<int> start_locations;
    std::vector<int> goal_locations;
    int num_of_agents;
    
    Instance() = default;
    
    // Create from SharedEnvironment
    Instance(const SharedEnvironment& env);
    
    bool isObstacle(int loc) const { return my_map[loc] != 0; }
    
    int getDegree(int loc) const;
    
    int getManhattanDistance(int loc1, int loc2) const {
        int row1 = loc1 / num_of_cols;
        int col1 = loc1 % num_of_cols;
        int row2 = loc2 / num_of_cols;
        int col2 = loc2 % num_of_cols;
        return abs(row1 - row2) + abs(col1 - col2);
    }
    
    std::vector<int> getNeighbors(int loc) const;
};

}
