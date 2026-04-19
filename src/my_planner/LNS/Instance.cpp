#include "Instance.h"

namespace LNS {

Instance::Instance(const SharedEnvironment& env) {
    num_of_cols = env.cols;
    num_of_rows = env.rows;
    map_size = num_of_cols * num_of_rows;
    num_of_agents = env.num_of_agents;
    
    my_map.resize(map_size);
    for (int i = 0; i < map_size; i++) {
        my_map[i] = env.map[i];
    }
    
    start_locations.resize(num_of_agents);
    start_orientations.resize(num_of_agents);
    goal_locations.resize(num_of_agents);
    
    for (int i = 0; i < num_of_agents; i++) {
        start_locations[i] = env.curr_states[i].location;
        start_orientations[i] = env.curr_states[i].orientation;
        goal_locations[i] = env.goal_locations[i][0].first;
    }
}

int Instance::getDegree(int loc) const {
    if (isObstacle(loc)) return 0;
    
    int degree = 0;
    int row = loc / num_of_cols;
    int col = loc % num_of_cols;
    
    // Check 4 neighbors
    if (row > 0 && !isObstacle(loc - num_of_cols)) degree++;
    if (row < num_of_rows - 1 && !isObstacle(loc + num_of_cols)) degree++;
    if (col > 0 && !isObstacle(loc - 1)) degree++;
    if (col < num_of_cols - 1 && !isObstacle(loc + 1)) degree++;
    
    return degree;
}

std::vector<int> Instance::getNeighbors(int loc) const {
    std::vector<int> neighbors;
    
    if (isObstacle(loc)) return neighbors;
    
    int row = loc / num_of_cols;
    int col = loc % num_of_cols;
    
    if (row > 0 && !isObstacle(loc - num_of_cols)) neighbors.push_back(loc - num_of_cols);
    if (row < num_of_rows - 1 && !isObstacle(loc + num_of_cols)) neighbors.push_back(loc + num_of_cols);
    if (col > 0 && !isObstacle(loc - 1)) neighbors.push_back(loc - 1);
    if (col < num_of_cols - 1 && !isObstacle(loc + 1)) neighbors.push_back(loc + 1);
    
    return neighbors;
}

}
