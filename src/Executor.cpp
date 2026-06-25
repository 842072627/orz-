#include "Executor.h"
#include "my_executor.h"
#include "executor_heat_lock.h"

void Executor::initialize(int preprocess_time_limit)
{
    MyExecutor::execute_initialize(preprocess_time_limit, env);
    heat_lock_init();
}

vector<State> Executor::process_new_plan(int sync_time_limit, Plan& plan_struct, vector<vector<Action>> & staged_actions)
{
    return MyExecutor::execute_process_new_plan(sync_time_limit, plan_struct, staged_actions, env);
}

void Executor::next_command(int exec_time_limit, std::vector<ExecutionCommand> & agent_command)
{
    // 方向锁已在process_new_plan的window simulation里处理
    // 这里只用标准TPG+MCP，没有方向锁override
    MyExecutor::execute_next_command(exec_time_limit, agent_command, env);
}
