#pragma once
#include <memory>
#include <string>
#include "Instance.h"
#include "Parallel/GlobalManager.h"
#include "../TrajLNS.h"
#include "../../inc/SharedEnv.h"

namespace LNS {

class PLNSSolver {
public:
    int num_of_agents;
    int window_size_for_PATH;
    int num_threads;
    int max_iterations;
    
    std::shared_ptr<Instance> instance;
    std::shared_ptr<MyPlanner::TrajLNS> lns;
    std::shared_ptr<GlobalManager> global_manager;
    
    PLNSSolver(
        int num_of_agents,
        int window_size_for_PATH,
        int num_threads,
        int max_iterations
    );
    
    void setEnvironment(const SharedEnvironment& env);
    void run();
};

}
