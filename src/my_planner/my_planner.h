#ifndef MY_PLANNER
#define MY_PLANNER

// Default planner baseline used by src/MAPFPlanner.cpp.
//
// Implements Traffic Flow Optimised Guided MY_PIBT: an anytime planner that first
// optimises traffic-flow guide paths, then runs MY_PIBT to produce collision-free
// multi-step actions. More planning time yields higher-quality solutions.
//
// References:
//   Chen, Z., Harabor, D., Li, J., & Stuckey, P. J. (2024). Traffic flow
//   optimisation for lifelong multi-agent path finding. AAAI Conference on
//   Artificial Intelligence, Vol. 38, No. 18, pp. 20674-20682.
//   https://ojs.aaai.org/index.php/AAAI/article/view/30054/31856
//
//   Okumura, K., et al. (2022). Priority Inheritance with Backtracking (MY_PIBT)
//   for iterative multi-agent path finding. Artificial Intelligence, Vol. 310.

#include "my_Types.h"
#include "my_TrajLNS.h"
#include <random>


namespace MyPlanner{

    
    void initialize(int preprocess_time_limit, SharedEnvironment* env);

    void plan(int time_limit, std::vector<std::vector<Action>> & actions,
                        SharedEnvironment* env, int num_steps=10);
    
}
#endif
