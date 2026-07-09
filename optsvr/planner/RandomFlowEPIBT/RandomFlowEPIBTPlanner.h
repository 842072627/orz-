#pragma once

#include "../EPIBT/EPIBTPlanner.h"

namespace optsvr {

// Random-domain submit line: EPIBT with FlowDecay guide-flow vertex congestion.
// Tuned weights are supplied by MapTuner/RuntimeConfig so FlowDecay mode 1
// remains a clean baseline.
class RandomFlowEPIBTPlanner {
public:
    void initialize(int preprocess_time_limit, SharedEnvironment* env);
    void plan(int time_limit, std::vector<std::vector<Action>>& actions,
              SharedEnvironment* env, int num_steps);

private:
    EPIBTPlanner epibt_;
};

} // namespace optsvr
