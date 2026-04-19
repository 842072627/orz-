#pragma once
#include <vector>
#include <memory>
#include <random>
#include "../Instance.h"
#include "../../TrajLNS.h"
#include "../../Types.h"

namespace LNS {

enum destroy_heuristic {
    RANDOM,
    CONGESTION,
    DEVIATION,
    RANDOMWALK,
    COUNT
};

class Neighbor {
public:
    std::vector<int> agents;
    int selected_neighbor;
    double old_sum_of_costs;
    double sum_of_costs;
    bool succ;
    std::vector<MyPlanner::Traj> m_old_paths;
    std::vector<MyPlanner::Traj> m_paths;
    
    Neighbor() : selected_neighbor(0), old_sum_of_costs(0), sum_of_costs(0), succ(false) {}
};

class NeighborGenerator {
public:
    Instance& instance;
    MyPlanner::TrajLNS& lns;
    int neighbor_size;
    destroy_heuristic destroy_strategy;
    bool ALNS;
    double decay_factor;
    double reaction_factor;
    std::vector<double> destroy_weights;
    std::mt19937 MT;
    
    std::vector<int> intersections;
    
    NeighborGenerator(
        Instance& instance,
        MyPlanner::TrajLNS& lns,
        int neighbor_size,
        destroy_heuristic destroy_strategy,
        bool ALNS,
        double decay_factor,
        double reaction_factor
    );
    
    void reset();
    Neighbor generate();
    
private:
    int findMostDelayedAgent();
    void randomWalk(int start_agent, int start_time, std::vector<int>& neighbors);
    double calculateDelay(int agent_id);
};

}
