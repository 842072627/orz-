#include "LacamPIBTPlanner.h"

#include "RuntimeConfig.h"

#include <iostream>

namespace optsvr {

void LacamPIBTPlanner::initialize(int preprocess_time_limit, SharedEnvironment* env) {
    const auto& c = cfg();
    std::cerr << "[LacamPIBTPlanner] initialized"
              << " flow_vertex_mode=" << c.flow_vertex_cost_mode
              << " flow_vertex_w=" << c.flow_vertex_cost_weight
              << " epibt_flow_vertex_mode=" << c.epibt_flow_vertex_mode
              << " epibt_flow_vertex_w=" << c.epibt_flow_vertex_weight
              << "\n";
    epibt_.initialize(preprocess_time_limit, env);
}

void LacamPIBTPlanner::plan(int time_limit, std::vector<std::vector<Action>>& actions,
                            SharedEnvironment* env, int num_steps) {
    epibt_.plan(time_limit, actions, env, num_steps);
}

} // namespace optsvr
