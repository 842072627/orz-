#pragma once
#include <memory>
#include <vector>
#include "../Instance.h"
#include "NeighborGenerator.h"
#include "../../TrajLNS.h"
#include "../../heuristics.h"

namespace LNS {

class LocalOptimizer {
public:
    Instance& instance;
    MyPlanner::TrajLNS& lns;
    std::string replan_algo_name;
    int window_size_for_PATH;
    std::vector<Neighbor> neighbors;
    
    LocalOptimizer(
        Instance& instance,
        MyPlanner::TrajLNS& lns,
        std::string replan_algo_name,
        int window_size_for_PATH
    );
    
    void reset();
    bool runPP(Neighbor& neighbor);
    void optimize(Neighbor& neighbor);
};

}
