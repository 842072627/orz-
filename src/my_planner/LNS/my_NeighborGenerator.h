#pragma once
#include <random>
#include <unordered_set>
#include <set>
#include "my_PathTable.h"

namespace LNS {

// Minimal NeighborGenerator for LNS
// Generates neighborhoods of agents to destroy/replan
class NeighborGenerator
{
public:
    int neighbor_size;  // number of agents to destroy
    int num_agents;
    std::mt19937 MT;

    NeighborGenerator() = default;
    
    NeighborGenerator(int num_agents, int neighbor_size, int random_seed = 0)
        : num_agents(num_agents), neighbor_size(neighbor_size), MT(random_seed) {}

    // Generate a neighborhood using RANDOMWALK strategy
    // Returns a set of agent IDs to be destroyed/replanned
    void generateRandomWalk(std::unordered_set<int>& agents_to_destroy) {
        agents_to_destroy.clear();
        
        // Pick a random starting agent
        std::uniform_int_distribution<int> dist(0, num_agents - 1);
        int start_agent = dist(MT);
        
        // Random walk to find conflicting neighbors
        int current_agent = start_agent;
        std::set<int> visited;
        
        for (int i = 0; i < neighbor_size && visited.size() < (size_t)num_agents; ++i) {
            if (visited.count(current_agent) == 0) {
                agents_to_destroy.insert(current_agent);
                visited.insert(current_agent);
            }
            
            // Find a neighbor (agents with intersecting paths)
            // For simplicity, just pick a random agent not yet visited
            int next_agent;
            int attempts = 0;
            do {
                next_agent = dist(MT);
                attempts++;
            } while (visited.count(next_agent) > 0 && attempts < 10);
            
            if (attempts < 10) {
                current_agent = next_agent;
            } else {
                break;  // Can't find more unique agents
            }
        }
    }

    // Generate neighborhood using simple random selection
    // (fallback if RANDOMWALK fails)
    void generateRandom(std::unordered_set<int>& agents_to_destroy) {
        agents_to_destroy.clear();
        
        std::uniform_int_distribution<int> dist(0, num_agents - 1);
        
        while ((int)agents_to_destroy.size() < neighbor_size) {
            agents_to_destroy.insert(dist(MT));
        }
    }
};

} // namespace LNS
