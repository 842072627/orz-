// Executor.cpp - Default executor (瘦包装)
#include "Executor.h"
#include "default_executor.h"

void Executor::initialize(int preprocess_time_limit)
{
    DefaultPlanner::execute_initialize(preprocess_time_limit, env);
}

vector<State> Executor::process_new_plan(int sync_time_limit, Plan& plan, vector<vector<Action>>& staged_actions)
{
    return DefaultPlanner::execute_process_new_plan(sync_time_limit, plan, staged_actions, env);
}

void Executor::next_command(int exec_time_limit, std::vector<ExecutionCommand>& agent_command)
{
    DefaultPlanner::execute_next_command(exec_time_limit, agent_command, env);
}
