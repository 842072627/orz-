#pragma once

#include "ActionModel.h"
#include "Plan.h"
#include "SharedEnv.h"

class Executor {
public:
    SharedEnvironment* env;

    explicit Executor(SharedEnvironment* shared_env) : env(shared_env) {}
    Executor() : env(new SharedEnvironment()) {}
    virtual ~Executor() = default;

    virtual void initialize(int preprocess_time_limit);
    virtual vector<State> process_new_plan(
        int sync_time_limit, Plan& plan,
        vector<vector<Action>>& staged_actions);
    virtual void next_command(
        int exec_time_limit,
        std::vector<ExecutionCommand>& agent_command);
};
