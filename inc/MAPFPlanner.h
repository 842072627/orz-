#pragma once
#include <ctime>
#include "SharedEnv.h"
#include "ActionModel.h"
#include "Plan.h"

namespace optsvr { class OptPlanner; }

class MAPFPlanner
{
public:
    SharedEnvironment* env;

	MAPFPlanner(SharedEnvironment* env): env(env){};
    MAPFPlanner(){env = new SharedEnvironment();};
	virtual ~MAPFPlanner();


    virtual void initialize(int preprocess_time_limit);

    // return next states for all agents
    virtual void plan(int time_limit, Plan & plan);

private:
    optsvr::OptPlanner* opt_planner_ = nullptr;
};
