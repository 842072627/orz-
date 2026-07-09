#pragma once

#include "../EPIBT/EPIBTPlanner.h"

namespace optsvr {

class LacamPIBTPlanner {
public:
    void initialize(int preprocess_time_limit, SharedEnvironment* env);
    void plan(int time_limit, std::vector<std::vector<Action>>& actions,
              SharedEnvironment* env, int num_steps);

private:
    EPIBTPlanner epibt_{true};
};

} // namespace optsvr
