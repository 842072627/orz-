#include "Executor.h"
#include "my_executor.h"
#include "my_executor_orz.h"
#include "executor_heat_lock.h"
#include <string>

// 判断当前地图是否需要 orz 专用 executor
static inline bool is_orz_map(const SharedEnvironment* env) {
    if (!env) return false;
    return env->map_name.find("orz") != std::string::npos;
}

void Executor::initialize(int preprocess_time_limit)
{
    if (is_orz_map(env)) {
        MyExecutorOrz::execute_initialize(preprocess_time_limit, env);
    } else {
        MyExecutor::execute_initialize(preprocess_time_limit, env);
    }
    heat_lock_init();
}

vector<State> Executor::process_new_plan(int sync_time_limit, Plan& plan_struct, vector<vector<Action>> & staged_actions)
{
    if (is_orz_map(env)) {
        return MyExecutorOrz::execute_process_new_plan(sync_time_limit, plan_struct, staged_actions, env);
    }
    return MyExecutor::execute_process_new_plan(sync_time_limit, plan_struct, staged_actions, env);
}

void Executor::next_command(int exec_time_limit, std::vector<ExecutionCommand> & agent_command)
{
    if (is_orz_map(env)) {
        // orz: 实时occupancy方向锁（在next_command里STOP逆流）
        MyExecutorOrz::execute_next_command(exec_time_limit, agent_command, env);
    } else {
        // 其他图: 方向锁在process_new_plan的window simulation里裁剪plan
        MyExecutor::execute_next_command(exec_time_limit, agent_command, env);
    }
}
