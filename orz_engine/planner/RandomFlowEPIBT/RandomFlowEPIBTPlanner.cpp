#include "RandomFlowEPIBTPlanner.h"

#include "RuntimeConfig.h"

#include <iostream>

namespace orz_engine {

void RandomFlowEPIBTPlanner::initialize(int preprocess_time_limit, SharedEnvironment* env) {
    const auto& c = cfg();
    std::cerr << "[RandomFlowEPIBT] initialized"
              << " flow_vertex_mode=" << c.flow_vertex_cost_mode
              << " flow_vertex_w=" << c.flow_vertex_cost_weight
              << " flow_vertex_base_w=" << c.flow_vertex_base_weight
              << " flow_vertex_sens_decay=" << c.flow_vertex_sens_decay
              << " epibt_flow_vertex_mode=" << c.epibt_flow_vertex_mode
              << " epibt_flow_vertex_w=" << c.epibt_flow_vertex_weight
              << "\n";
    epibt_.initialize(preprocess_time_limit, env);
}

void RandomFlowEPIBTPlanner::plan(int time_limit, std::vector<std::vector<Action>>& actions,
                                  SharedEnvironment* env, int num_steps) {
    epibt_.plan(time_limit, actions, env, num_steps);
}

} // namespace orz_engine
