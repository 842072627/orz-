#include "Executor.h"
#include "default_executor.h"
#include "executor_heat_lock.h"

void Executor::initialize(int preprocess_time_limit)
{
    DefaultPlanner::execute_initialize(preprocess_time_limit, env);
    heat_lock_init();
}

vector<State> Executor::process_new_plan(int sync_time_limit, Plan& plan_struct, vector<vector<Action>> & staged_actions)
{
    return DefaultPlanner::execute_process_new_plan(sync_time_limit, plan_struct, staged_actions, env);
}

void Executor::next_command(int exec_time_limit, std::vector<ExecutionCommand> & agent_command)
{
    // 1) 原版 TPG 更新 + 原版 mcp（先用原版 logic 做基础 GO/STOP）
    DefaultPlanner::execute_next_command(exec_time_limit, agent_command, env);

    // 2) 方向锁：热力更新 + 锁决策
    heat_lock_update(env);

    // 3) 方向锁覆盖：把逆行的 GO 改成 STOP
    int moves[4] = {1, env->cols, -1, -env->cols};
    for (int i = 0; i < (int)env->system_states.size(); i++) {
        // 只有原版给了 GO 才需要检查方向锁
        if (agent_command[i] != ExecutionCommand::GO) continue;
        if (env->staged_actions[i].empty()) continue;
        if (env->staged_actions[i].front() != Action::FW) continue; // 转向永远放行

        int curr_loc = env->system_states[i].location;
        int curr_orient = env->system_states[i].orientation;

        if (heat_lock_check(i, curr_loc, curr_orient, env)) {
            agent_command[i] = ExecutionCommand::STOP;
        }
    }
}
