#include "PLNSSolver.h"

namespace LNS {

PLNSSolver::PLNSSolver(
    int num_of_agents,
    int window_size_for_PATH,
    int num_threads,
    int max_iterations
) : num_of_agents(num_of_agents),
    window_size_for_PATH(window_size_for_PATH),
    num_threads(num_threads),
    max_iterations(max_iterations)
{
}

void PLNSSolver::setEnvironment(const SharedEnvironment& env) {
    // Create instance from environment
    instance = std::make_shared<Instance>(env);
    
    // Create TrajLNS
    lns = std::make_shared<TrajLNS>();
    lns->env = const_cast<SharedEnvironment*>(&env);
    lns->init_mem();
    
    // Create GlobalManager
    global_manager = std::make_shared<Parallel::GlobalManager>(
        *instance,
        *lns,
        num_threads,
        max_iterations,
        8,  // neighbor_size
        Parallel::destroy_heuristic::RANDOMWALK,
        true,  // ALNS
        0.01,  // decay_factor
        0.1   // reaction_factor
    );
}

void PLNSSolver::run() {
    if (global_manager) {
        global_manager->run();
    }
}

}
